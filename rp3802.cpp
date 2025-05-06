/*
 * Copyright (c) 2025 Hirokuni Yano
 *
 * Released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "fifo.h"

#include "rp3802.pio.h"

//
// GPIO assignment
//
//  GPIO 0-2        A0-A2
//  GPIO 3-10       D0-D7
//  GPIO 11         /CS
//  GPIO 12         /RD
//  GPIO 13         /WR
//  GPIO 14         /IC
//  GPIO 15         /IRQ
//  GPIO 16         UART TX
//  GPIO 17         UART RX

#define GPIO_ADDR_BUS_WIDTH     (3)
#define GPIO_DATA_BUS_WIDTH     (8)
#define GPIO_CTRL_BUS_WIDTH     (4)
#define GPIO_BIT_WIDTH          (GPIO_ADDR_BUS_WIDTH + GPIO_DATA_BUS_WIDTH + GPIO_CTRL_BUS_WIDTH)

#define GPIO_BASE               (0)
#define GPIO_ADDR_BUS           (GPIO_BASE)
#define GPIO_DATA_BUS           (GPIO_ADDR_BUS + GPIO_ADDR_BUS_WIDTH)
#define GPIO_CTRL_BUS           (GPIO_DATA_BUS + GPIO_DATA_BUS_WIDTH)
#define GPIO_IRQ                (GPIO_CTRL_BUS + GPIO_CTRL_BUS_WIDTH)
#define GPIO_ALL_MASK           ((1<<GPIO_BIT_WIDTH) - 1)

#define UART_ID                 (uart0)
#define BAUD_RATE               (312500)
#define UART_TX_PIN             (16)
#define UART_RX_PIN             (17)


FIFO<uint32_t, 16> fifo_tx;
FIFO<uint32_t, 128> fifo_rx;
FIFO<uint32_t, 4> fifo_itx;
FIFO<uint32_t, 4> fifo_irx;

// registers
uint8_t reg[16 * 0x10];
// for DMA/PIO read access
uint8_t reg_dma[16] __attribute__ ((aligned (16)));

// timer
uint64_t timer_gp_next_us = 0;

// lock for multicore
spin_lock_t *lock;

// YM3802 bus access log (FIFO)
#define RP3802_ACCESS_BUFFER_SHIFT (6)
#define RP3802_ACCESS_BUFFER_COUNT (1 << RP3802_ACCESS_BUFFER_SHIFT)
static uint32_t rp3802_access_buffer[RP3802_ACCESS_BUFFER_COUNT] __attribute__ ((aligned (sizeof(uint32_t) * RP3802_ACCESS_BUFFER_COUNT)));
static uint32_t rp3802_access_buffer_count = RP3802_ACCESS_BUFFER_COUNT;
static int rp3802_access_dma_ch = 0;
static volatile uint32_t *rp3802_access_buffer_wp_ptr = NULL;
static uint32_t rp3802_access_buffer_rp = 0;


static void rp3802_reg_read_init(PIO pio, uint sm, uint offset, uint pin, uint8_t *reg_ptr)
{
    const uint pin_addr = pin + rp3802_reg_read_addr_bus_offset;
    const uint pin_data = pin + rp3802_reg_read_data_bus_offset;
    for (uint i = 0; i < rp3802_reg_read_data_bus_width; i++)
    {
        pio_gpio_init(pio, pin_data + i);
    }
    pio_sm_set_consecutive_pindirs(pio, sm, pin_addr, rp3802_reg_read_addr_bus_width, false);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_data, rp3802_reg_read_data_bus_width, false);
    pio_sm_config c = rp3802_reg_read_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin_addr);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_out_pins(&c, pin_data, rp3802_reg_read_data_bus_width);
    sm_config_set_out_shift(&c, true, true, 8);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio, sm, offset + rp3802_reg_read_offset_entry_point, &c);
    pio_sm_set_enabled(pio, sm, true);

    uint32_t romaddr_top = (uint32_t)reg_ptr >> rp3802_reg_read_addr_bus_width;
    pio_sm_put_blocking(pio, sm, romaddr_top);
}

static void rp3802_oe_init(PIO pio, uint sm, uint offset, uint pin)
{
    const uint pin_ctrl = pin + rp3802_oe_ctrl_bus_offset;
    const uint pin_data = pin + rp3802_oe_data_bus_offset;
    pio_sm_set_consecutive_pindirs(pio, sm, pin_ctrl, rp3802_oe_ctrl_bus_width, false);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_data, rp3802_oe_data_bus_width, false);
    pio_sm_config c = rp3802_oe_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin_ctrl);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_pins(&c, pin_data, rp3802_oe_data_bus_width);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio, sm, offset + rp3802_oe_offset_entry_point, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static void rp3802_access_init(PIO pio, uint sm, uint offset, uint pin)
{
    pio_sm_set_consecutive_pindirs(pio, sm, pin, rp3802_access_bit_width, false);
    pio_sm_config c = rp3802_access_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_in_shift(&c, true, false, 32);
    sm_config_set_clkdiv_int_frac(&c, 1, 0);
    pio_sm_init(pio, sm, offset + rp3802_access_offset_entry_point, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline uint32_t rp3802_access_buffer_wp(void)
{
    return (*rp3802_access_buffer_wp_ptr / 4) % RP3802_ACCESS_BUFFER_COUNT;
}

void rp3802_access_start(void)
{
    rp3802_access_buffer_rp = rp3802_access_buffer_wp();
}

bool rp3802_access_is_empty(void)
{
    return (rp3802_access_buffer_rp == rp3802_access_buffer_wp());
}

uint32_t rp3802_access_pop(void)
{
    const uint32_t ret = rp3802_access_buffer[rp3802_access_buffer_rp];
    rp3802_access_buffer_rp = (rp3802_access_buffer_rp + 1) % RP3802_ACCESS_BUFFER_COUNT;
    return ret;
}


static void rp3802_init(PIO pio, uint pin, uint8_t *reg_ptr)
{
    dma_channel_config c;

    // RP3802 REGISTER READ
    uint offset_rp3802_reg_read = pio_add_program(pio, &rp3802_reg_read_program);
    uint sm_rp3802_reg_read = pio_claim_unused_sm(pio, true);
    int dma_addr = dma_claim_unused_channel(true);
    int dma_data = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(dma_addr);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_rp3802_reg_read, false));
    dma_channel_configure(
        dma_addr,
        &c,
        &dma_hw->ch[dma_data].al3_read_addr_trig,
        &pio->rxf[sm_rp3802_reg_read],
        1,
        false
    );

    c = dma_channel_get_default_config(dma_data);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, dma_addr);
    dma_channel_configure(
        dma_data,
        &c,
        &pio->txf[sm_rp3802_reg_read],
        NULL,
        1,
        false
    );

    dma_channel_start(dma_addr);

    rp3802_reg_read_init(pio, sm_rp3802_reg_read, offset_rp3802_reg_read, pin, reg_ptr);


    // RP2802 OUTPUT ENABLE
    uint offset_rp3802_oe = pio_add_program(pio, &rp3802_oe_program);
    uint sm_rp3802_oe = pio_claim_unused_sm(pio, true);
    rp3802_oe_init(pio, sm_rp3802_oe, offset_rp3802_oe, pin);


    // RP2802 ACCESS
    uint offset_rp3802_access = pio_add_program(pio, &rp3802_access_program);
    uint sm_rp3802_access = pio_claim_unused_sm(pio, true);
    int dma_access_trig = dma_claim_unused_channel(true);
    int dma_access_buff = dma_claim_unused_channel(true);

    c = dma_channel_get_default_config(dma_access_trig);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    dma_channel_configure(
        dma_access_trig,
        &c,
        &dma_hw->ch[dma_access_buff].al1_transfer_count_trig,
        &rp3802_access_buffer_count,
        1,
        false
    );

    c = dma_channel_get_default_config(dma_access_buff);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, RP3802_ACCESS_BUFFER_SHIFT + 2);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_rp3802_access, false));
    channel_config_set_chain_to(&c, dma_access_trig);
    dma_channel_configure(
        dma_access_buff,
        &c,
        rp3802_access_buffer,
        &pio->rxf[sm_rp3802_access],
        RP3802_ACCESS_BUFFER_COUNT,
        false
    );

    rp3802_access_dma_ch = dma_access_buff;
    rp3802_access_buffer_wp_ptr = &dma_hw->ch[rp3802_access_dma_ch].write_addr;
    dma_channel_start(dma_access_trig);

    rp3802_access_init(pio, sm_rp3802_access, offset_rp3802_access, pin);
}




static inline uint8_t ym3802_reg_group(void)
{
    return reg_dma[1] & 0x0f;
}

static inline void ym3802_write(uint8_t addr, uint8_t data)
{
    reg_dma[addr] = data;
    if (addr == 1)
    {
        uint8_t base = ym3802_reg_group() * 16;
        reg_dma[4] = reg[base + 4];
        reg_dma[5] = reg[base + 5];
        reg_dma[6] = reg[base + 6];
        reg_dma[7] = reg[base + 7];
    }
    else if (addr >= 4)
    {
        uint8_t base = ym3802_reg_group() * 16;
        reg[base + addr] = data;
    }
}

static inline uint8_t ym3802_read(uint8_t addr)
{
    if (addr < 4)
    {
        return reg_dma[addr];
    }
    else
    {
        uint8_t base = ym3802_reg_group() * 16;
        return reg[base + addr];
    }
}

static inline void ym3802_reg_update(uint8_t regno, uint8_t data)
{
    // for R04~
    uint8_t group = (regno >> 4) & 0x0f;
    uint8_t group_cur = ym3802_reg_group();
    if (group == group_cur)
    {
        reg_dma[regno & 0x07] = reg[regno];
    }
    reg[regno] = data;
}

static inline uint8_t ym3802_reg_value(uint8_t regno)
{
    // for R04~
    return reg[regno];
}

static inline uint32_t bus_get_addr(uint32_t bus)
{
    return bus & ((1 << GPIO_ADDR_BUS_WIDTH) - 1);
}

static inline uint32_t bus_get_data(uint32_t bus)
{
    return (bus >> GPIO_ADDR_BUS_WIDTH) & ((1 << GPIO_DATA_BUS_WIDTH) - 1);
}

static inline uint32_t get_bus_data(void)
{
    while (rp3802_access_is_empty())
    {
        // wait for access
    }
    return rp3802_access_pop();
}

static inline void ym3802_update_rx_status(void)
{
    uint32_t irq_state = spin_lock_blocking(lock);
    uint8_t status = ym3802_reg_value(0x34) & 0x3f;
    if (!fifo_rx.is_full())
    {
        // RxOV (0:-, 1:overflow detected)
        status |= 0x40;
    }
    if (!fifo_rx.is_empty())
    {
        // RxRDY (0:empty, 1:data ready)
        status |= 0x80;

        // pre-fetch top data in FIFO-Rx and set to RDR
        uint32_t data;
        fifo_rx.peek(data);
        ym3802_reg_update(0x36, data);  // RDR
    }
    ym3802_reg_update(0x34, status);
    spin_unlock(lock, irq_state);
}

static inline void ym3802_update_tx_status(void)
{
    uint32_t irq_state = spin_lock_blocking(lock);
    uint8_t status = ym3802_reg_value(0x54) & 0x3f;
    if (!fifo_tx.is_full())
    {
        // TxRDY (0:FIFO-Tx full, 1:ready)
        status |= 0x40;
    }
    if (!fifo_tx.is_empty())
    {
        // TxEMP (0:data exist, 1:empty)
        status |= 0x80;
    }
    ym3802_reg_update(0x54, status);
    spin_unlock(lock, irq_state);
}

static inline void ym3802_set_irq(uint8_t irq)
{
    uint32_t irq_state = spin_lock_blocking(lock);
    reg_dma[0x02] |= (irq & reg[0x06]);

    uint8_t status = reg_dma[0x02];
    uint32_t irqno;

	for (irqno = 0; irqno < 8; irqno++)
	{
		if (status & (1 << irqno))
			break;
	}
	reg_dma[0x00] = (reg[04] & 0xe0) | (irqno << 1);

    if (status != 0)
    {
        // Assert IRQ line
        gpio_put(GPIO_IRQ, 0);
    }
    spin_unlock(lock, irq_state);
}

static inline void ym3802_clr_irq(uint8_t irq)
{
    uint32_t irq_state = spin_lock_blocking(lock);
    reg_dma[0x02] &= ~irq;

    uint8_t status = reg_dma[0x02];
    if (status == 0)
    {
        // Clear IRQ line
        gpio_put(GPIO_IRQ, 1);
    }
    spin_unlock(lock, irq_state);
}

static inline uint64_t ym3802_gp_timer_count(void)
{
    return ((reg[0x85] & 0x3f) << 8) | reg[0x84];
}

static inline void ym3802_gp_timer_load(uint64_t now_us, uint64_t next_us)
{
    uint32_t irq_state = spin_lock_blocking(lock);
    timer_gp_next_us = now_us + next_us;
    spin_unlock(lock, irq_state);
}

static void access_write(uint32_t bus)
{
    const uint32_t data = bus_get_data(bus);
    const uint32_t addr = bus_get_addr(bus);

    switch (addr)
    {
    case 0x00:
        // IVR:R: IRQ vector
        break;
    case 0x01:
        // RGR:RW: system control
        {
            reg_dma[1] = data;
            uint8_t base = ym3802_reg_group() * 16;
            reg_dma[4] = reg[base + 4];
            reg_dma[5] = reg[base + 5];
            reg_dma[6] = reg[base + 6];
            reg_dma[7] = reg[base + 7];
        }
        break;
    case 0x02:
        // ISR:R: IRQ status
        break;
    case 0x03:
        // ICR:W: IRQ clear request
        break;
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
        uint8_t regno = ym3802_reg_group() * 16 + addr;
        switch (regno)
        {
        case 0x04:
            // IOR:W: IRQ vector offset request
            ym3802_reg_update(regno, data);
            // update IVR
            reg_dma[0x00] = (reg_dma[0x00] & 0x1f) | (data & 0xe0);
            break;
        case 0x05:
            // IMR:W: IRQ mode control
            ym3802_reg_update(regno, data);
            break;
        case 0x06:
            // IER:W: IRQ enable request
            ym3802_reg_update(regno, data);
            break;

        case 0x14:
            // DMR:W: MIDI reqltime message control
            ym3802_reg_update(regno, data);
            break;
        case 0x15:
            // DCR:W: MIDI realtime message request
            ym3802_reg_update(regno, data);
            break;
        case 0x16:
            // DSR:R: FIFO-IRx data
            break;
        case 0x17:
            // DNR:W: FIFO-IRx control
            ym3802_reg_update(regno, data);
            break;

        case 0x24:
            // RRR:W: Rx communication rate
            ym3802_reg_update(regno, data);
            break;
        case 0x25:
            // RMR:W: Rx communication mode
            ym3802_reg_update(regno, data);
            break;
        case 0x26:
            // AMR:W: Address-hunter control
            ym3802_reg_update(regno, data);
            break;
        case 0x27:
            // ADR:W: Address-hunter control
            ym3802_reg_update(regno, data);
            break;

        case 0x34:
            // RSR:R: FIFO-Rx status
            break;
        case 0x35:
            // RCR:W: FIFO-Rx control
            ym3802_reg_update(regno, data);
            break;
        case 0x36:
            // RDR:R: FIFO-Rx data
            break;

        case 0x44:
            // TRR:W: Tx communication rate
            ym3802_reg_update(regno, data);
            break;
        case 0x45:
            // TMR:W: Tx communication mode
            ym3802_reg_update(regno, data);
            break;

        case 0x54:
            // TSR:R: FIFO-Tx status
            {
                ym3802_update_tx_status();
            }
            break;
        case 0x55:
            // TCR:W: FIFO-Tx control
            ym3802_reg_update(regno, data);
            break;
        case 0x56:
            // TDR:W: FIFO-Tx data
            {
                // IRQ-6(Clear): When the FIFO-Tx is load with data.
                ym3802_clr_irq(1 << 6);
                // send UART and update status
                fifo_tx.push(data);
                ym3802_update_tx_status();
            }
            break;

        case 0x64:
            // FSR:R: FSK status
            break;
        case 0x65:
            // FCR:W FSK control
            ym3802_reg_update(regno, data);
            break;
        case 0x66:
            // CCR:W: Click counter control
            ym3802_reg_update(regno, data);
            break;
        case 0x67:
            // CDR:W: Click counter load value
            ym3802_reg_update(regno, data);
            break;

        case 0x74:
            // SRR:R: Recording counter current value
            break;
        case 0x75:
            // SCR:W: Sequencer control
            ym3802_reg_update(regno, data);
            break;
        case 0x76:
            // SPRL:W: Play-back counter value
            ym3802_reg_update(regno, data);
            break;
        case 0x77:
            // SPRH:W: Play-back counter value
            ym3802_reg_update(regno, data);
            break;

        case 0x84:
            // GTRL:W: General timer value
            ym3802_reg_update(regno, data);
            break;
        case 0x85:
            // GTRH:W: General timer value
            ym3802_reg_update(regno, data);
            if (data & 0x80)
            {
                uint64_t now_us = time_us_64();
                uint64_t count = ym3802_gp_timer_count();
                ym3802_gp_timer_load(now_us, timer_gp_next_us + count * 8);
            }
            break;
        case 0x86:
            // MTRL:W: MIDI-clock timer value
            ym3802_reg_update(regno, data);
            break;
        case 0x87:
            // MTRH:W: MIDI-clock timer value
            ym3802_reg_update(regno, data);
            break;

         case 0x94:
            // EDR:W: External I/O direction
            ym3802_reg_update(regno, data);
            break;
        case 0x95:
            // EOR:W: External I/O output data
            ym3802_reg_update(regno, data);
            break;
        case 0x96:
            // EIR:R: External I/O input data
            break;
        }
        break;
    }
}

static void access_read(uint32_t bus)
{
    const uint32_t data = bus_get_data(bus);
    const uint32_t addr = bus_get_addr(bus);

    switch (addr)
    {
    case 0x00:
        // IVR:R: IRQ vector
        break;
    case 0x02:
        // ISR:R: IRQ status
        break;
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
        uint8_t regno = ym3802_reg_group() * 16 + addr;
        switch (regno)
        {
        case 0x16:
            // DSR:R: FIFO-IRx data
            break;
        case 0x34:
            // RSR:R: FIFO-Rx status
            ym3802_update_rx_status();
            break;
        case 0x36:
            // RDR:R: FIFO-Rx data
            {
                // set the top data in FIFO-Rx to RDR
                if (!fifo_rx.is_empty())
                {
                    uint32_t data;
                    fifo_rx.pop(data);
                    ym3802_update_rx_status();

                    if ((reg[0x34] & 0x80) == 0)
                    {
                        // IRQ-5(Clear): When the FIFO-Rx becomes empty.
                        ym3802_clr_irq(1 << 5);
                    }
                }
            }
            break;
        case 0x54:
            // TSR:R: FIFO-Tx status
            ym3802_update_tx_status();
            break;
        case 0x64:
            // FSR:R: FSK status
            break;
        case 0x74:
            // SRR:R: Recording counter current value
            break;
        case 0x96:
            // EIR:R: External I/O input data
            break;
        }
        break;
    }
}

void process_ym3802_access(void)
{
    enum {
        CTRL_SHIFT = GPIO_ADDR_BUS_WIDTH + GPIO_DATA_BUS_WIDTH,
        CSWR_MASK = 0x03, // CS(0x01) | WR(0x02)
        CSRD_MASK = 0x05, // CS(0x01) | RD(0x04)
        LOW4_MASK = 0x0f,
    };

    // Initialize
    {
        // IVR:R: IRQ vector
        reg_dma[0x00] = 0x00;
        // RGR:RW: system control
        reg_dma[0x01] = 0x00;
        // ISR:R: IRQ status
        reg_dma[0x02] = 0x00;
        // DSR:R: FIFO-IRx data
        ym3802_reg_update(0x16, 0x00);
        // RSR:R: FIFO-Rx status
        ym3802_reg_update(0x34, 0x00);
        // RDR:R: FIFO-Rx data
        ym3802_reg_update(0x36, 0x00);
        // TSR:R: FIFO-Tx status
        ym3802_reg_update(0x54, 0xc0);
        // FSR:R: FSK status
        ym3802_reg_update(0x64, 0x00);
        // SRR:R: Recording counter current value
        ym3802_reg_update(0x74, 0x00);
        // EIR:R: External I/O input data
        ym3802_reg_update(0x96, 0xff);
    }

    // Main loop
    for (;;)
    {
        const uint32_t bus = get_bus_data();
        const uint32_t ctrl_bits = (bus >> CTRL_SHIFT);
        const uint32_t curr = (ctrl_bits >> 0) & LOW4_MASK;
        const uint32_t prev = (ctrl_bits >> 4) & LOW4_MASK;

        // アクティブ状態検出(負論理: 0がアクティブ)
        const bool cswr_edge = ((prev & CSWR_MASK) == 0) & ((curr & CSWR_MASK) != 0);
        const bool csrd_edge = ((prev & CSRD_MASK) == 0) & ((curr & CSRD_MASK) != 0);

        if (cswr_edge)
        {
            access_write(bus);
        }
        else if (csrd_edge)
        {
            access_read(bus);
        }
        tight_loop_contents();
    }
}

int main(int argc, char *argv[])
{
    stdio_init_all();

    printf("Tiny-YM3802 Emulator\n");

    // SPIN LOCK
    lock = spin_lock_instance(0);

    // GPIO
    gpio_init_mask(GPIO_ALL_MASK);
    // IRQ line
    gpio_put(GPIO_IRQ, 1);
    gpio_set_dir(GPIO_IRQ, true);

    // UART
    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, true);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // PIO
    rp3802_init(pio0, GPIO_BASE, reg_dma);

    // Register accessing routine
    multicore_launch_core1(process_ym3802_access);

    // Main loop
    for (;;)
    {
        // UART handling
        if (!fifo_tx.is_empty())
        {
            uint32_t data;
            fifo_tx.pop(data);
            uart_putc_raw(UART_ID, data & 0xff);
            ym3802_update_tx_status();
            if (fifo_tx.is_empty())
            {
                // IRQ-6: When the FIFO-Tx becomes empty through the data extraction by the transmitter.
                ym3802_set_irq(1 << 6);
            }

            printf("Tx: %02x\n", data);
        }
        if (uart_is_readable(UART_ID) && !fifo_rx.is_full())
        {
            uint32_t data = (uint8_t)uart_getc(UART_ID);
            bool from_empty = fifo_rx.is_empty();
            fifo_rx.push(data);
            ym3802_update_rx_status();
            if (from_empty)
            {
                // IRQ-5: When the empty FIFO-Rx is loaded with data.
                ym3802_set_irq(1 << 5);
            }

            printf("Rx: %02x\n", data);
        }

        // Timer handling
        if (reg[0x06] & 0x80)
        {
            uint64_t now_us = time_us_64();
            if (now_us >= timer_gp_next_us)
            {
                uint64_t count = ym3802_gp_timer_count();
                if (timer_gp_next_us == 0)
                {
                    timer_gp_next_us = now_us;
                }
                ym3802_gp_timer_load(now_us, timer_gp_next_us + count * 8);
                // IRQ-7: When the timer reaches a count of zero.
                ym3802_set_irq(1 << 7);
            }
        }

        tight_loop_contents();
    }

    return 0;
}