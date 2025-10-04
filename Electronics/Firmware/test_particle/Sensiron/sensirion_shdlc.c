/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// NOTE: Heavily modified

#include "sensirion_shdlc.h"
#include "sensirion_common.h"
#include "usart_dma.h"

extern uart_dma_tx_t uart2_dma_tx;
extern uart_dma_rx_t uart2_dma_rx;

static uint8_t sensirion_shdlc_checksum(uint8_t header_sum, uint8_t data_len, const uint8_t* data) {
    header_sum += data_len;

    while (data_len--)
        header_sum += *(data++);

    return ~header_sum;
}

static uint16_t sensirion_shdlc_stuff_data(uint8_t data_len, const uint8_t* data, uint8_t* stuffed_data) {
    uint16_t output_data_len = 0;

    while (data_len--) {
        uint8_t c = *(data++);
        switch (c) {
            case 0x11:
            case 0x13:
            case 0x7d:
            case 0x7e:
                /* byte stuffing is done by inserting 0x7d and inverting bit 5
                 */
                *(stuffed_data++) = 0x7d;
                *(stuffed_data++) = c ^ (1 << 5);
                output_data_len += 2;
                break;
            default:
                *(stuffed_data++) = c;
                output_data_len += 1;
        }
    }
    return output_data_len;
}

static inline uint8_t sensirion_shdlc_check_unstuff(uint8_t data) {
    return data == 0x7d;
}

static uint8_t sensirion_shdlc_unstuff_byte(uint8_t data) {
    switch (data) {
        case 0x31:
            return 0x11;
        case 0x33:
            return 0x13;
        case 0x5d:
            return 0x7d;
        case 0x5e:
            return 0x7e;
        default:
            return data;
    }
}


// Efficient skip-to-next-marker: find next marker byte and advance tail up to (but not including) marker.
// Because 0x7E is never present except as unescaped markers in SHDLC, we don't need to interpret escapes here.
// Returns number of bytes skipped (0 if marker is already at tail or none present).
static size_t uart_rx_skip_to_next_marker(uart_dma_rx_t *h) {
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    uint32_t head = h->head;
    uint32_t tail = h->tail;
    uint32_t cnt = head - tail;
    if (cnt == 0U) {
        __set_PRIMASK(prim);
        return 0U;
    }

    // if first byte already marker, do not skip
    uint8_t first = h->buf[tail & UART_RX_MASK];
    if (first == SHDLC_START) {
        __set_PRIMASK(prim);
        return 0U;
    }

    size_t skip_count = 0;
    for (uint32_t i = 0; i < cnt; ++i) {
        uint8_t b = h->buf[(tail + i) & UART_RX_MASK];
        if (b == SHDLC_START) { skip_count = (size_t)i; break; }
    }

    if (skip_count == 0) {
        // either first was marker (handled above) or no marker found -> skip all
        skip_count = (size_t)cnt;
    }

    if (skip_count > 0) {
        h->tail = h->tail + (uint32_t)skip_count;
        h->new_data = (h->head != h->tail) ? 1U : 0U;
    }

    __set_PRIMASK(prim);
    return skip_count;
}

/*
int16_t sensirion_shdlc_xcv(uint8_t addr, uint8_t cmd, uint8_t tx_data_len,
                            const uint8_t* tx_data, uint8_t max_rx_data_len,
                            struct sensirion_shdlc_rx_header* rx_header,
                            uint8_t* rx_data) {
    int16_t ret;

    ret = sensirion_shdlc_tx(addr, cmd, tx_data_len, tx_data);
    if (ret != 0)
        return ret;

    sensirion_uart_hal_sleep_usec(RX_DELAY_US);
    return sensirion_shdlc_rx(max_rx_data_len, rx_header, rx_data);
}*/

// Transmit a data packet
int16_t sensirion_shdlc_tx(uart_dma_tx_t *h, uint8_t addr, uint8_t cmd, uint8_t data_len, const uint8_t* data) {
    uint16_t len = 0;
    int16_t ret;
    uint8_t crc;
    uint8_t tx_frame_buf[SHDLC_FRAME_MAX_TX_FRAME_SIZE];

    crc = sensirion_shdlc_checksum(addr + cmd, data_len, data);

    tx_frame_buf[len++] = SHDLC_START;
    len += sensirion_shdlc_stuff_data(1, &addr, tx_frame_buf + len);
    len += sensirion_shdlc_stuff_data(1, &cmd, tx_frame_buf + len);
    len += sensirion_shdlc_stuff_data(1, &data_len, tx_frame_buf + len);
    len += sensirion_shdlc_stuff_data(data_len, data, tx_frame_buf + len);
    len += sensirion_shdlc_stuff_data(1, &crc, tx_frame_buf + len);
    tx_frame_buf[len++] = SHDLC_STOP;

    ret = uart_dma_tx_send(h, tx_frame_buf, len);
    if (ret < 0)
        return ret;
    if (ret != len)
        return SENSIRION_SHDLC_ERR_TX_INCOMPLETE;
    return 0;
}

// Process a receive buffer looking for valid packets
int16_t sensirion_shdlc_rx(uart_dma_rx_t *h, uint8_t max_data_len, sensirion_shdlc_rx_t *rx) {
    // ensure we start at a marker: consume garbage before first start
    uart_rx_skip_to_next_marker(h);

    // re-check availability and start marker
    size_t avail = uart_rx_available(h);
    if (avail < SHDLC_MIN_RX_FRAME_SIZE) return 0;
    if (uart_rx_peek(h, 0) != SHDLC_START) return 0;

    // snapshot up to the frame max
    uint8_t raw[SHDLC_FRAME_MAX_RX_STUFFED_FRAME_SIZE];
    size_t raw_copied = uart_rx_snapshot(h, raw, SHDLC_FRAME_MAX_RX_STUFFED_FRAME_SIZE);
    if (raw_copied == 0U) return 0;

    // scan raw[] once while un-stuffing into header and data buffers as we go
    size_t ri = 0; // raw index
    if (raw[ri] != SHDLC_START) {
        // recover: consume start and skip
        uart_rx_skip(h, 1);
        uart_rx_skip_to_next_marker(h);
        return SENSIRION_SHDLC_ERR_MISSING_START;
    }
    ri++; // move past START

    // un-stuff header into a temporary array then memcpy to rxh (safer than aliasing)
    uint8_t hdr_tmp[4];
    uint8_t un_next = 0;
    uint32_t hdr_i = 0;
    while (hdr_i < sizeof(hdr_tmp)) {
        if (ri >= raw_copied) return 0; // need more raw bytes
        uint8_t b = raw[ri++];
        if (un_next) {
            hdr_tmp[hdr_i++] = sensirion_shdlc_unstuff_byte(b);
            un_next = 0;
        } else if (sensirion_shdlc_check_unstuff(b)) {
            un_next = 1;
        } else {
            if (b == SHDLC_STOP) {
            	uart_rx_skip(h, 1);
                uart_rx_skip_to_next_marker(h);
                return SENSIRION_SHDLC_ERR_ENCODING_ERROR;
            }
            hdr_tmp[hdr_i++] = b;
        }
    }
    // Copy header into rx
    rx->addr = hdr_tmp[0];
    rx->cmd = hdr_tmp[1];
    rx->state = hdr_tmp[2];
    rx->length = hdr_tmp[3];

    // Check data_len
    if (rx->length > max_data_len) {
    	uart_rx_skip(h, 1);
        uart_rx_skip_to_next_marker(h);
        return SENSIRION_SHDLC_ERR_FRAME_TOO_LONG;
    }

    // un-stuff data directly into 'data' buffer
    uint32_t data_collected = 0;
    un_next = 0;
    while (data_collected < rx->length) {
        if (ri >= raw_copied) return 0; // wait for more raw bytes
        uint8_t b = raw[ri++];
        if (un_next) {
        	rx->data[data_collected++] = sensirion_shdlc_unstuff_byte(b);
            un_next = 0;
        } else if (sensirion_shdlc_check_unstuff(b)) {
            un_next = 1;
        } else {
            if (b == SHDLC_STOP) {
            	uart_rx_skip(h, 1);
                uart_rx_skip_to_next_marker(h);
                return SENSIRION_SHDLC_ERR_ENCODING_ERROR;
            }
            rx->data[data_collected++] = b;
        }
    }

    // parse CRC (one unstuffed byte)
    uint8_t crc_frame;
    if (ri >= raw_copied) return 0; // wait
    uint8_t b = raw[ri++];
    if (sensirion_shdlc_check_unstuff(b)) {
        if (ri >= raw_copied) return 0; // wait
        crc_frame = sensirion_shdlc_unstuff_byte(raw[ri++]);
    } else {
        crc_frame = b;
    }

    // next raw must be STOP marker (unescaped)
    if (ri >= raw_copied) return 0; // wait
    if (raw[ri] != SHDLC_STOP) {
    	uart_rx_skip(h, 1);
        uart_rx_skip_to_next_marker(h);
        return SENSIRION_SHDLC_ERR_MISSING_STOP;
    }

    // compute checksum and compare
    uint8_t header_sum = (uint8_t)(rx->addr + rx->cmd + rx->state);
    uint8_t calc = sensirion_shdlc_checksum(header_sum, rx->length, rx->data);
    if (calc != crc_frame) {
    	uart_rx_skip(h, 1);
        uart_rx_skip_to_next_marker(h);
        return SENSIRION_SHDLC_ERR_CRC_MISMATCH;
    }

    // success: we have a valid frame and ri points to the STOP index; raw length = ri+1 (inclusive stop)
    size_t raw_len = ri + 1U;
    // advance tail by raw_len (consume frame)
    uart_rx_skip(h, raw_len);

    return 1;
}
/*
int16_t sensirion_shdlc_rx(uint8_t max_data_len, struct sensirion_shdlc_rx_header* rxh, uint8_t* data) {
    int16_t len;
    uint16_t i;
    uint8_t rx_frame[SHDLC_FRAME_MAX_RX_FRAME_SIZE];
    uint8_t* rx_header = (uint8_t*)rxh;
    uint8_t j;
    uint8_t crc;
    uint8_t unstuff_next;

    //len = sensirion_uart_hal_rx(2 + (5 + (uint16_t)max_data_len) * 2, rx_frame);
    if (len < 1 || rx_frame[0] != SHDLC_START)
        return SENSIRION_SHDLC_ERR_MISSING_START;

    for (unstuff_next = 0, i = 1, j = 0; j < sizeof(*rxh) && i < len - 2; ++i) {
        if (unstuff_next) {
            rx_header[j++] = sensirion_shdlc_unstuff_byte(rx_frame[i]);
            unstuff_next = 0;
        } else {
            unstuff_next = sensirion_shdlc_check_unstuff(rx_frame[i]);
            if (!unstuff_next)
                rx_header[j++] = rx_frame[i];
        }
    }
    if (j != sizeof(*rxh) || unstuff_next)
        return SENSIRION_SHDLC_ERR_ENCODING_ERROR;

    if (max_data_len < rxh->data_len)
        return SENSIRION_SHDLC_ERR_FRAME_TOO_LONG; // more data than expected

    for (unstuff_next=0, j=0; (j < rxh->data_len) && (i < len - 2); ++i) {
        if (unstuff_next) {
            data[j++] = sensirion_shdlc_unstuff_byte(rx_frame[i]);
            unstuff_next = 0;
        } else {
            unstuff_next = sensirion_shdlc_check_unstuff(rx_frame[i]);
            if (!unstuff_next)
                data[j++] = rx_frame[i];
        }
    }

    if (unstuff_next)
        return SENSIRION_SHDLC_ERR_ENCODING_ERROR;

    if (j < rxh->data_len)
        return SENSIRION_SHDLC_ERR_ENCODING_ERROR;

    crc = rx_frame[i++];
    if (sensirion_shdlc_check_unstuff(crc))
        crc = sensirion_shdlc_unstuff_byte(rx_frame[i++]);

    if (sensirion_shdlc_checksum(rxh->addr + rxh->cmd + rxh->state,
                                 rxh->data_len, data) != crc)
        return SENSIRION_SHDLC_ERR_CRC_MISMATCH;

    if (i >= len || rx_frame[i] != SHDLC_STOP)
        return SENSIRION_SHDLC_ERR_MISSING_STOP;

    if (0x7F & rxh->state) {
        return SENSIRION_SHDLC_ERR_EXECUTION_FAILURE;
    }

    return 0;
}*/

/*
static void sensirion_shdlc_stuff_byte(struct sensirion_shdlc_buffer* tx_frame, uint8_t data) {
    switch (data) {
        case 0x11:
        case 0x13:
        case 0x7d:
        case 0x7e:
            // byte stuffing is done by inserting 0x7d and inverting bit 5
            tx_frame->data[tx_frame->offset++] = 0x7d;
            tx_frame->data[tx_frame->offset++] = data ^ (1 << 5);
            return;
        default:
            tx_frame->data[tx_frame->offset++] = data;
            return;
    }
}*/
/*
void sensirion_shdlc_add_uint8_t_to_frame(struct sensirion_shdlc_buffer* tx_frame, uint8_t data) {
    sensirion_shdlc_stuff_byte(tx_frame, data);
    tx_frame->checksum += data;
}

void sensirion_shdlc_begin_frame(struct sensirion_shdlc_buffer* tx_frame,
                                 uint8_t* buffer, uint8_t command,
                                 uint8_t address, uint8_t data_length) {
    tx_frame->data = buffer;
    tx_frame->checksum = 0;
    tx_frame->offset = 0;
    tx_frame->data[tx_frame->offset++] = SHDLC_START;
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, address);
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, command);
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, data_length);
}

void sensirion_shdlc_add_bool_to_frame(struct sensirion_shdlc_buffer* tx_frame, bool data) {
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, data);
}

void sensirion_shdlc_add_uint32_t_to_frame(struct sensirion_shdlc_buffer* tx_frame, uint32_t data) {
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, (uint8_t)((data & 0xFF000000) >> 24));
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, (uint8_t)((data & 0x00FF0000) >> 16));
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, (uint8_t)((data & 0x0000FF00) >> 8));
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, (uint8_t)((data & 0x000000FF) >> 0));
}

void sensirion_shdlc_add_int32_t_to_frame(struct sensirion_shdlc_buffer* tx_frame, int32_t data) {
    sensirion_shdlc_add_uint32_t_to_frame(tx_frame, (uint32_t)data);
}

void sensirion_shdlc_add_uint16_t_to_frame(struct sensirion_shdlc_buffer* tx_frame, uint16_t data) {
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, (uint8_t)((data & 0xFF00) >> 8));
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, (uint8_t)((data & 0x00FF) >> 0));
}

void sensirion_shdlc_add_int16_t_to_frame(struct sensirion_shdlc_buffer* tx_frame, int16_t data) {
    sensirion_shdlc_add_uint16_t_to_frame(tx_frame, (uint16_t)data);
}

void sensirion_shdlc_add_float_to_frame(struct sensirion_shdlc_buffer* tx_frame, float data) {
    union {
        uint32_t uint32_data;
        float float_data;
    } convert;

    convert.float_data = data;
    sensirion_shdlc_add_uint32_t_to_frame(tx_frame, convert.uint32_data);
}

void sensirion_shdlc_add_bytes_to_frame(struct sensirion_shdlc_buffer* tx_frame,
                                        const uint8_t* data,
                                        uint16_t data_length) {
    uint16_t i;

    for (i = 0; i < data_length; i++) {
        sensirion_shdlc_add_uint8_t_to_frame(tx_frame, data[i]);
    }
}

void sensirion_shdlc_finish_frame(struct sensirion_shdlc_buffer* tx_frame) {
    sensirion_shdlc_add_uint8_t_to_frame(tx_frame, ~(tx_frame->checksum));
    tx_frame->data[tx_frame->offset++] = SHDLC_STOP;
}*/
/*
int16_t sensirion_shdlc_tx_frame(struct sensirion_shdlc_buffer* tx_frame) {
    int16_t tx_length;

    tx_length = sensirion_uart_hal_tx(tx_frame->offset, tx_frame->data);
    if (tx_length < 0) {
        return tx_length;
    }
    if (tx_length != tx_frame->offset) {
        return SENSIRION_SHDLC_ERR_TX_INCOMPLETE;
    }
    return NO_ERROR;
}

static uint8_t sensirion_shdlc_unstuff_next_byte(struct sensirion_shdlc_buffer* rx_frame) {
    uint8_t data = rx_frame->data[rx_frame->offset++];

    if (data == 0x7d) {
        data = rx_frame->data[rx_frame->offset++];
        data = data ^ (1 << 5);
    }
    rx_frame->checksum += data;
    return data;
}*/
/*
int16_t sensirion_shdlc_rx_inplace(struct sensirion_shdlc_buffer* rx_frame,
                                   uint8_t expected_data_length,
                                   struct sensirion_shdlc_rx_header* header) {
    int16_t rx_length;
    uint16_t i;
    rx_frame->offset = 0;
    rx_frame->checksum = 0;

    rx_length = sensirion_uart_hal_rx(
        2 + (5 + (uint16_t)expected_data_length) * 2, rx_frame->data);
    if (rx_length < 1 || rx_frame->data[rx_frame->offset++] != SHDLC_START) {
        return SENSIRION_SHDLC_ERR_MISSING_START;
    }

    header->addr = sensirion_shdlc_unstuff_next_byte(rx_frame);
    header->cmd = sensirion_shdlc_unstuff_next_byte(rx_frame);
    header->state = sensirion_shdlc_unstuff_next_byte(rx_frame);
    header->data_len = sensirion_shdlc_unstuff_next_byte(rx_frame);

    if (expected_data_length < header->data_len) {
        return SENSIRION_SHDLC_ERR_FRAME_TOO_LONG; // more data than expected
    }

    for (i = 0; i < header->data_len && rx_frame->offset < rx_length - 2; i++) {
        rx_frame->data[i] = sensirion_shdlc_unstuff_next_byte(rx_frame);
    }

    if (i < header->data_len) {
        return SENSIRION_SHDLC_ERR_ENCODING_ERROR;
    }

    sensirion_shdlc_unstuff_next_byte(rx_frame);

    // (CHECKSUM + ~CHECKSUM) = 0xFF
    if (rx_frame->checksum != 0xFF) {
        return SENSIRION_SHDLC_ERR_CRC_MISMATCH;
    }

    if (rx_frame->offset >= rx_length ||
        rx_frame->data[rx_frame->offset] != SHDLC_STOP) {
        return SENSIRION_SHDLC_ERR_MISSING_STOP;
    }

    if (0x7F & header->state) {
        return SENSIRION_SHDLC_ERR_EXECUTION_FAILURE;
    }

    return NO_ERROR;
}
*/
