// Samsung IR Remote Decoder
// Timer: TIM1 at 5 MHz (240 MHz / 48 prescaler)
// Resolution: 200 ns per tick
// DMA captures 66 edges per frame in linear mode

#pragma once

#include <stdint.h>
#include <stdbool.h>

// Configuration
#define IR_BUFFER_SIZE          68      // 2 start edges + 66 data edges
#define TIMER_FREQ_MHZ          5       // 5 MHz after prescaler
#define TICKS_PER_US            5       // 5 ticks per microsecond

// Timing thresholds (in us)
#define START_PULSE_LOW_MIN     4000    // 4.5ms ±0.5ms tolerance
#define START_PULSE_LOW_MAX     5000
#define START_PULSE_HIGH_MIN    4000
#define START_PULSE_HIGH_MAX    5000

#define BIT_LOW_MIN             360     // 560µs ±200µs
#define BIT_LOW_MAX             760

#define LOGIC_0_SPACE_MIN       360     // Logic 0 space: 560µs ±200µs
#define LOGIC_0_SPACE_MAX       760

#define LOGIC_1_SPACE_MIN       1490    // Logic 1 space: 1690µs ±200µs
#define LOGIC_1_SPACE_MAX       1890

// Convert microseconds to timer ticks
#define US_TO_TICKS(us)         ((us) * TICKS_PER_US)
#define TICKS_TO_US(ticks)      ((ticks) / TICKS_PER_US)

typedef struct {
	uint16_t capture_buffer[IR_BUFFER_SIZE];  // DMA buffer for edge timestamps
	uint32_t decoded_code;                    // 32-bit decoded IR code (LSB first)
	volatile uint8_t overflow_count;          // TIM1 overflow counter
	volatile bool new_data_available;         // True when valid frame captured
	volatile bool decode_error;               // True on DMA or decode failure
	volatile bool dma_active;                 // True after first edge when DMA is active
} IR_Decoder_t;

// Global instance
extern IR_Decoder_t ir_decode;

void IR_Decoder_Init(IR_Decoder_t *ir);
void IR_Start_DMA_Capture(IR_Decoder_t *ir);
void IR_Decoder_DMA_IRQHandler(IR_Decoder_t *ir);
void IR_TIM1_CC_IRQHandler(IR_Decoder_t *ir);
void IR_TIM1_UP_IRQHandler(IR_Decoder_t *ir);
bool IR_Decode_Frame(IR_Decoder_t *ir);
