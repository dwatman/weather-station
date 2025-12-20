#include <stdio.h>
#include <string.h>
#include "ir_decode.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_dma.h"

// Global IR decoder instance
IR_Decoder_t ir_decode = {0};

// Helper to calculate pulse width with timer overflow
static uint16_t IR_Get_Pulse_Width(uint16_t start_tick, uint16_t end_tick) {
	if (end_tick >= start_tick) return end_tick - start_tick;
	return (0xFFFF - start_tick) + end_tick + 1;
}

// Start DMA capture
void IR_Start_DMA_Capture(IR_Decoder_t *ir) {
	// Disable TIM1 DMA request first to stop new triggers
	LL_TIM_DisableDMAReq_CC1(TIM1);

	// Clear TIM1 CC1 flag to prevent stale triggers
	LL_TIM_ClearFlag_CC1(TIM1);

	// Robust DMA disable sequence
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_1);
	LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_1);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);

	// Wait for disable with force-abort on timeout
	uint32_t timeout = 1000;
	while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1) && timeout--) {
		if (timeout == 0) {
			// Force abort by clearing EN bit directly
			DMA1_Stream1->CR &= ~DMA_SxCR_EN;
		}
	}

	// Configure DMA
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)&TIM1->CCR1);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)(ir->capture_buffer));
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, IR_BUFFER_SIZE);
	LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);

	LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_HALFWORD);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);

	// Clear all DMA flags
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);
	LL_DMA_ClearFlag_DME1(DMA1);
	LL_DMA_ClearFlag_FE1(DMA1);

	// Enable interrupts
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_1);

	// Enable DMA stream then TIM1 DMA request
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
	LL_TIM_EnableDMAReq_CC1(TIM1);
}

// Initialize IR decoder
void IR_Decoder_Init(IR_Decoder_t *ir) {
	memset(ir, 0, sizeof(IR_Decoder_t));

	// Enable TIM1 channel 1 capture
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

	// Clear any pending flags
	LL_TIM_ClearFlag_CC1(TIM1);
	LL_TIM_ClearFlag_UPDATE(TIM1);

	// Enable CC1 interrupt for edge detection (overflow counter reset)
	LL_TIM_EnableIT_CC1(TIM1);

	// Enable the timer counter
	LL_TIM_EnableCounter(TIM1);

	// Start DMA capture
	IR_Start_DMA_Capture(ir);
}

// DMA IRQ handler
void IR_Decoder_DMA_IRQHandler(IR_Decoder_t *ir) {
	// Transfer complete
	if (LL_DMA_IsActiveFlag_TC1(DMA1)) {
		LL_DMA_ClearFlag_TC1(DMA1);
		ir->new_data_available = true;
	}

	// Transfer error
	if (LL_DMA_IsActiveFlag_TE1(DMA1)) {
		LL_DMA_ClearFlag_TE1(DMA1);
		printf("INT IR TE\n");
		ir->decode_error = true;
		IR_Start_DMA_Capture(ir);
	}

	// Disable overflow counter until first edge
	ir->dma_active = 0;
	LL_TIM_DisableIT_UPDATE(TIM1);    // Overflow
}

// TIM1 CC1 capture interrupt: reset overflow counter on each edge
void IR_TIM1_CC_IRQHandler(IR_Decoder_t *ir) {
	// DMA already clears the CC1 flag, so we can't check it

	// Enable overflow counter on first edge of frame
	if (ir->dma_active == 0) {
		ir->dma_active = 1;
		ir->overflow_count = 0;
		LL_TIM_ClearFlag_UPDATE(TIM1);
		LL_TIM_EnableIT_UPDATE(TIM1);
	}

	// Reset overflow counter on any edge (activity detected)
	ir->overflow_count = 0;
}

// TIM1 update (overflow) interrupt: detect frame timeout
void IR_TIM1_UP_IRQHandler(IR_Decoder_t *ir) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
		LL_TIM_ClearFlag_UPDATE(TIM1);
		ir->overflow_count++;

		// Timeout after 2 overflows (~26ms with no edges)
		if (ir->overflow_count >= 2) {
			printf("IR frame timeout\n");
			LL_TIM_DisableIT_UPDATE(TIM1);

			// Clear stale flags and restart
			ir->new_data_available = false;
			ir->decode_error = false;
			ir->dma_active = 0;
			ir->overflow_count = 0;

			IR_Start_DMA_Capture(ir);
		}
	}
}

// Decode captured IR frame into 32-bit code
// Return true if decode successful, false on error
bool IR_Decode_Frame(IR_Decoder_t *ir) {
	uint16_t *buf = ir->capture_buffer;
	uint32_t code = 0;

	if (!buf) return false;

//	printf("Periods (us): ");
//	for(int i = 0; i < 8; i++)
//		printf("%u ", TICKS_TO_US(IR_Get_Pulse_Width(buf[i], buf[i+1])));
//	printf("\n");

	// Validate start pulse (LOW then HIGH)
	uint16_t start_low = TICKS_TO_US(IR_Get_Pulse_Width(buf[0], buf[1]));
	uint16_t start_high = TICKS_TO_US(IR_Get_Pulse_Width(buf[1], buf[2]));

	if (start_low < START_PULSE_LOW_MIN || start_low > START_PULSE_LOW_MAX ||
		start_high < START_PULSE_HIGH_MIN || start_high > START_PULSE_HIGH_MAX) {
		printf("IR decode error: invalid start pulse\n");
		return false;
	}

	// Decode 32 bits (starting from edge index 2)
	for (int bit = 0; bit < 32; bit++) {
		int idx = 2 + bit * 2;
		uint16_t low_pulse = TICKS_TO_US(IR_Get_Pulse_Width(buf[idx], buf[idx + 1]));
		uint16_t high_space = TICKS_TO_US(IR_Get_Pulse_Width(buf[idx + 1], buf[idx + 2]));

		// Validate LOW pulse width
		if (low_pulse < BIT_LOW_MIN || low_pulse > BIT_LOW_MAX) {
			printf("IR decode error: invalid low pulse at bit %d\n", bit);
			return false;
		}

		// Decode HIGH space: logic 0 (short) or logic 1 (long)
		if (high_space >= LOGIC_0_SPACE_MIN && high_space <= LOGIC_0_SPACE_MAX) {
			code &= ~(1UL << bit);  // Logic 0
		} else if (high_space >= LOGIC_1_SPACE_MIN && high_space <= LOGIC_1_SPACE_MAX) {
			code |= (1UL << bit);   // Logic 1
		} else {
			printf("IR decode error: invalid high pulse at bit %d\n", bit);
			return false;  // Invalid pulse width
		}
	}

	ir->decoded_code = code;
	return true;
}

// Validate and decode a 32-bit Samsung/NEC code
// Returns true if valid, false if checksum fails
bool IR_CheckAndDecode(IR_Decoder_t *ir) {

	uint8_t addr_lsb = (uint8_t)(ir->decoded_code & 0xFF);
	uint8_t addr_msb = (uint8_t)((ir->decoded_code >> 8) & 0xFF);
	uint8_t cmd      = (uint8_t)((ir->decoded_code >> 16) & 0xFF);
	uint8_t cmd_inv  = (uint8_t)((ir->decoded_code >> 24) & 0xFF);

	// Validate by checking that it matches its inverse
	bool valid = ((cmd ^ cmd_inv) == 0xFF);

	if (!valid) {
		printf("IR code invalid: command checksum mismatch\n");
		printf("AddrLSB=0x%02X AddrMSB=0x%02X Cmd=0x%02X Inv=0x%02X\n",
			   addr_lsb, addr_msb, cmd, cmd_inv);
		return false;
	}

	// Combine address bytes (little endian)
	uint16_t addr = (uint16_t)(addr_msb << 8 | addr_lsb);

	printf("IR valid frame:");
	printf("  Address: 0x%02X", addr);
	printf("  Command: 0x%02X", cmd);
	ir->decoded_address = addr;
	ir->decoded_command = cmd;

	// Interpret known command codes
	switch (cmd) {
		case 0x02: printf("  -> Power\n"); break;
		case 0x07: printf("  -> Volume Up\n"); break;
		case 0x0B: printf("  -> Volume Down\n"); break;
		case 0x12: printf("  -> Channel Up\n"); break;
		case 0x10: printf("  -> Channel Down\n"); break;
		default:   printf("  -> Unknown command\n"); break;
	}

	return true;
}

