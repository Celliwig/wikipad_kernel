/*
 *  linux/arch/arm/kernel/debug.S
 *
 *  Copyright (C) 1994-1999 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  32-bit debugging code
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "../mach-tegra/iomap.h"

		.text

/*
 * Some debugging routines (useful if you've got MM problems and
 * printk isn't working).  For DEBUGGING ONLY!!!  Do not leave
 * references to these in a production kernel!
 */

#if !defined(CONFIG_DEBUG_SEMIHOSTING)
#include CONFIG_DEBUG_LL_INCLUDE
#endif

#ifdef CONFIG_MMU
		.macro	addruart_current, rx, tmp1, tmp2
		addruart	\tmp1, \tmp2, \rx
		mrc		p15, 0, \rx, c1, c0
		tst		\rx, #1
		moveq		\rx, \tmp1
		movne		\rx, \tmp2
		.endm

#else /* !CONFIG_MMU */
		.macro	addruart_current, rx, tmp1, tmp2
		addruart	\rx, \tmp1, \tmp2
		.endm

#endif /* CONFIG_MMU */

/*
 * Useful debugging routines
 */
ENTRY(printhex8)
		mov	r1, #8
		b	printhex
ENDPROC(printhex8)

ENTRY(printhex4)
		mov	r1, #4
		b	printhex
ENDPROC(printhex4)

ENTRY(printhex2)
		mov	r1, #2
printhex:	adr	r2, hexbuf_rel
		ldr	r3, [r2]
		add	r2, r2, r3
		add	r3, r2, r1
		mov	r1, #0
		strb	r1, [r3]
1:		and	r1, r0, #15
		mov	r0, r0, lsr #4
		cmp	r1, #10
		addlt	r1, r1, #'0'
		addge	r1, r1, #'a' - 10
		strb	r1, [r3, #-1]!
		teq	r3, r2
		bne	1b
		mov	r0, r2
		b	printascii
ENDPROC(printhex2)

		.pushsection .bss
hexbuf_addr:	.space 16
		.popsection
		.align
hexbuf_rel:	.long	hexbuf_addr - .

		.ltorg

#ifndef CONFIG_DEBUG_SEMIHOSTING

ENTRY(printascii)
//		addruart_current r3, r1, r2
1:		teq	r0, #0
		ldrneb	r1, [r0], #1
		teqne	r1, #0
		reteq	lr
2:		teq     r1, #'\n'
		bne	3f
		mov	r1, #'\r'
//		waituart r2, r3
//		senduart r1, r3
//		busyuart r2, r3
		stmfd	sp!, {lr}						// Save lr
		bl	dbg_serial_bitbang_char
		ldmfd	sp!, {lr}						// Restore lr
		mov	r1, #'\n'
3:
//		waituart r2, r3
//		senduart r1, r3
//		busyuart r2, r3
		stmfd	sp!, {lr}						// Save lr
		bl	dbg_serial_bitbang_char
		ldmfd	sp!, {lr}						// Restore lr
		b	1b
ENDPROC(printascii)

ENTRY(printch)
//		addruart_current r3, r1, r2
		mov	r1, r0
		mov	r0, #0
		b	2b
ENDPROC(printch)

#ifdef CONFIG_MMU
ENTRY(debug_ll_addr)
		addruart r2, r3, ip
		str	r2, [r0]
		str	r3, [r1]
		ret	lr
ENDPROC(debug_ll_addr)
#endif

#else

ENTRY(printascii)
		mov	r1, r0
		mov	r0, #0x04		@ SYS_WRITE0
	ARM(	svc	#0x123456	)
#ifdef CONFIG_CPU_V7M
	THUMB(	bkpt	#0xab		)
#else
	THUMB(	svc	#0xab		)
#endif
		ret	lr
ENDPROC(printascii)

ENTRY(printch)
		adr	r1, hexbuf_rel
		ldr	r2, [r1]
		add	r1, r1, r2
		strb	r0, [r1]
		mov	r0, #0x03		@ SYS_WRITEC
	ARM(	svc	#0x123456	)
#ifdef CONFIG_CPU_V7M
	THUMB(	bkpt	#0xab		)
#else
	THUMB(	svc	#0xab		)
#endif
		ret	lr
ENDPROC(printch)

ENTRY(debug_ll_addr)
		mov	r2, #0
		str	r2, [r0]
		str	r2, [r1]
		ret	lr
ENDPROC(debug_ll_addr)

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define		DBG_GPIO_OFFSET_ADDR		0xD000
#define		DBG_GPIO_BASE_ADDR		IO_PPSB_PHYS + DBG_GPIO_OFFSET_ADDR
#define		DBG_GPIO_BASE_VADDR		IO_PPSB_VIRT + DBG_GPIO_OFFSET_ADDR
#define		DBG_GPIO_PORT_CNF		0x00
#define		DBG_GPIO_PORT_OE		0x10
#define		DBG_GPIO_PORT_OUT		0x20
#define		DBG_GPIO_PORT_MSK_CNF		0x80
#define		DBG_GPIO_PORT_MSK_OUT		0XA0
#define		DBG_GPIO_PORT_B			0x004
#define		DBG_GPIO_PORT_B_SPKR_EN		(1<<1)
#define		DBG_GPIO_PORT_H			0x10C
#define		DBG_GPIO_PORT_H_LCD_EN		(1<<2)
#define		DBG_GPIO_PORT_H_BACKLIGHT	(1<<3)
#define		DBG_GPIO_PORT_K 		0x208
#define		DBG_GPIO_PORT_K_TS_RST 		(1<<7)
#define		DBG_GPIO_PORT_J 		0x204
#define		DBG_GPIO_PORT_W			0x508
#define		DBG_GPIO_PORT_W_LCD_VDDS	(1<<1)

#define		DBG_CLK_RESET_BASE			0x60006000
//#define	DBG_CLK_RESET_OFFSET			0x6000
//#define	DBG_CLK_RESET_BASE			(IO_PPSB_PHYS+DBG_CLK_RESET_OFFSET)
//#define	DBG_CLK_RESET_BASE			(IO_PPSB_VIRT+DBG_CLK_RESET_OFFSET)
#define		DBG_RST_DEVICES_H_OFFSET		0x08
#define		DBG_SWR_I2C2_RST			(1<<22)
#define		DBG_RST_DEVICES_U_OFFSET		0x0c
#define		DBG_SWR_UART4_RST			(1<<1)

#define		DBG_CLK_OUT_ENB_H_OFFSET		0x14
#define		DBG_CLK_ENB_I2C2			(1<<22)
#define		DBG_CLK_OUT_ENB_U_OFFSET		0x18
#define		DBG_CLK_ENB_UART4			(1<<1)

#define		DBG_CLK_RST_CONTROLLER_CLK_SOURCE_I2C2_0	0x198
#define		DBG_CLK_RST_CONTROLLER_CLK_SOURCE_UARTD_0	0x1c0

#define 	DBG_ADDR_PBASE				0x70000000
//#define 	DBG_ADDR_PBASE				IO_APB_PHYS
//#define 	DBG_ADDR_VBASE				IO_APB_VIRT

#define		DBG_PINMUX_PINMUX			(3<<0)
#define		DBG_PINMUX_PUPD				(3<<2)
#define		DBG_PINMUX_TRISTATE			(1<<4)
#define		DBG_PINMUX_EINPUT			(1<<5)
#define		DBG_PINMUX_BASE				0x70003000
//#define	DBG_PINMUX_GEN2_I2C_SCL			0x3250
//#define	DBG_PINMUX_GEN2_I2C_SDA			0x3254
//#define 	DBG_PINMUX_GMI_A16			0x3230			// UARTD TX
//#define	DBG_PINMUX_GMI_A17			0x3234			// UARTD RX
//#define	DBG_PINMUX_GMI_A19			0x323c			// TS reset
#define 	DBG_PINMUX_GMI_A18			0x70003238		// Speaker enable
#define		DBG_PINMUX_GMI_A19			0x7000323c		// TS reset
#define		DBG_PINMUX_GEN2_I2C_SCL			0x70003250
#define		DBG_PINMUX_GEN2_I2C_SDA			0x70003254

// Prints platform configuration
ENTRY(dbg_print_config)
		stmfd   sp!, {r0 - r7, lr}
		mov	r0, #'\n'
		bl	printch
		mov	r0, #'\n'
		bl	printch
		bl	dbg_print_car
		mov	r0, #'\n'
		bl	printch
		bl	dbg_print_pinmux
		mov	r0, #'\n'
		bl	printch
		bl	dbg_print_gpio
		ldmfd   sp!, {r0 - r7, pc}
dbg_print_config_loop:
		b	dbg_print_config_loop
ENDPROC(dbg_print_config)

// Print GPIO config registers
ENTRY(dbg_print_gpio)
		stmfd   sp!, {r0 - r7, lr}
		adr	r0, str_gpio1
		bl	printascii
		ldr	r6, =DBG_GPIO_BASE_ADDR
		ldr	r7, =(DBG_GPIO_BASE_ADDR+0x800)
dbg_print_gpio_loop:
		mov	r4, r6
		bl	dbg_print_memloc
		add	r6, #4
		cmp	r6, r7
		ble	dbg_print_gpio_loop
		ldmfd   sp!, {r0 - r7, pc}
ENDPROC(dbg_print_gpio)
str_gpio1:	.asciz	"Print GPIO registers.\n"

// Print PinMux config registers
ENTRY(dbg_print_pinmux)
		stmfd   sp!, {r0 - r7, lr}
		adr	r0, str_pinmux1
		bl	printascii
		ldr	r6, =DBG_PINMUX_BASE
		ldr	r7, =(DBG_PINMUX_BASE+0x3e0)
dbg_print_pinmux_loop:
		mov	r4, r6
		bl	dbg_print_memloc
		add	r6, #4
		cmp	r6, r7
		ble	dbg_print_pinmux_loop
		ldmfd   sp!, {r0 - r7, pc}
ENDPROC(dbg_print_pinmux)
str_pinmux1:	.asciz	"Print PINMUX registers.\n"

// Print Clock And Reset config registers
ENTRY(dbg_print_car)
		stmfd   sp!, {r0 - r7, lr}
		adr	r0, str_car1
		bl	printascii
		ldr	r6, =DBG_CLK_RESET_BASE
		ldr	r7, =(DBG_CLK_RESET_BASE+0x18)
dbg_print_car_loop:
		mov	r4, r6
		bl	dbg_print_memloc
		add	r6, #4
		cmp	r6, r7
		ble	dbg_print_car_loop
		ldmfd   sp!, {r0 - r7, pc}
ENDPROC(dbg_print_car)
str_car1:	.asciz	"Print CAR registers.\n"

// Read a memory location and print it
// r4 - memory loction to read
ENTRY(dbg_print_memloc)
		stmfd   sp!, {r0 - r6, lr}
		mov	r0, #'\t'
		bl	printch
		ldr	r5, [r4]
		mov	r0, r4
		bl	printhex8
		mov	r0, #':'
		bl	printch
		mov	r0, #' '
		bl	printch
		mov	r0, r5
		bl	printhex8
		mov	r0, #'\n'
		bl	printch
		ldmfd   sp!, {r0 - r6, pc}
ENDPROC(dbg_print_memloc)

// Prints 'hello world' to bitbang serial interface
ENTRY(dbg_serial_bitbang_hw)
		mov	r1, #'H'
		bl	dbg_serial_bitbang_char
		mov	r1, #'e'
		bl	dbg_serial_bitbang_char
		mov	r1, #'l'
		bl	dbg_serial_bitbang_char
		mov	r1, #'l'
		bl	dbg_serial_bitbang_char
		mov	r1, #'o'
		bl	dbg_serial_bitbang_char
		mov	r1, #' '
		bl	dbg_serial_bitbang_char
		mov	r1, #'W'
		bl	dbg_serial_bitbang_char
		mov	r1, #'o'
		bl	dbg_serial_bitbang_char
		mov	r1, #'r'
		bl	dbg_serial_bitbang_char
		mov	r1, #'l'
		bl	dbg_serial_bitbang_char
		mov	r1, #'d'
		bl	dbg_serial_bitbang_char
		mov	r1, #'\n'
		bl	dbg_serial_bitbang_char
		mov	r1, #'\r'
		bl	dbg_serial_bitbang_char

		b	dbg_serial_bitbang_hw
ENDPROC(dbg_serial_bitbang_hw)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bit-bang serial interface
// https://www.seetron.com/apps/app_serial_coms.html

ENTRY(dbg_out_setup)
		stmfd   sp!, {r0 - r6, lr}

// GPIO
// DBG_GPIO_PORT_K - 1 1 1 1 1 1 1 3 = 0x00000002
		ldr	r4, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_CNF)		// Load base address
		ldr	r1, [r4]								// Get current state
		orr	r1, #DBG_GPIO_PORT_K_TS_RST						// Enable GPIO pin
		str	r1, [r4]								// Save config

		ldr	r4, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OE)		// Load base address
		ldr	r1, [r4]								// Get current state
		orr	r1, #DBG_GPIO_PORT_K_TS_RST						// Enable GPIO pin output
		str	r1, [r4]								// Save config

//		ldr	r4, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OE)		// Load base address
//		ldr	r0, [r4]								// Get current state
//		bl	dbg_display_flash_number32
//test_loop:
//		b	test_loop

// Pin Mux
//		ldr	r4, =DBG_PINMUX_GMI_A19
//		ldr	r0, [r4]
//		bl	dbg_display_flash_number32
//test_loop:
//		b	test_loop
// DBG_PINMUX_GMI_A19 - 1 1 1 1 1 1 1 1 = 0x00000000

		//bl	dbg_serial_set_baud_early
		adr	r0, bitbang_baud_rate_count
		mov	r1, #0
		str	r1, [r0]								// Clear baud rate setting

		adr	r0, bitbang_gpio_addr
		ldr	r1, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OUT)
		str	r1, [r0]

		bl	dbg_tsreset_off								// Set line to idle

		ldmfd   sp!, {r0 - r6, pc}
ENDPROC(dbg_out_setup)

// Send character using serial protocol by bitbanging line
// Nop when baud rate isn't set
// r1 - character to transmit
ENTRY(dbg_serial_bitbang_char)
		stmfd   sp!, {r0 - r6, lr}

		adr	r0, bitbang_baud_rate_count				// Get baud rate address
		ldr	r0, [r0]						// Get current baud rate
		cmp	r0, #0							// Check if it's set
		beq	dbg_serial_bitbang_char_finish				// Just finish if it's not

		bl	dbg_tsreset_on						// Start bit
		bl	dbg_serial_bitbang_delay

		mov	r2, #8							// Bit count
dbg_serial_bitbang_char_loop:
		bl	dbg_serial_bitbang					// Output bit
		bl	dbg_serial_bitbang_delay				// Delay for required period
		mov	r1, r1, lsr #1						// Rotate 1 bit so next bit is prepared for transmit
		subs	r2, r2, #1						// Decrement counter
		bne	dbg_serial_bitbang_char_loop				// Loop with there are remaining bits

		bl	dbg_tsreset_on						// Stop bit
		bl	dbg_serial_bitbang_delay
		bl	dbg_tsreset_off						// Return line to idle
		bl	dbg_serial_bitbang_delay

dbg_serial_bitbang_char_finish:
		ldmfd   sp!, {r0 - r6, pc}
ENDPROC(dbg_serial_bitbang_char)

// Tests lsb and sets line as appropriate
// r1 - Variable
ENTRY(dbg_serial_bitbang)
		stmfd   sp!, {lr}
		tst	r1, #1							// Test lsb
		bne	dbg_serial_bitbang_1					// Jump if bit was 1
		bl	dbg_tsreset_on						// Set line high
		ldmfd   sp!, {pc}
dbg_serial_bitbang_1:
		bl	dbg_tsreset_off						// Set line low
		ldmfd   sp!, {pc}
ENDPROC(dbg_serial_bitbang)

// Pulse line (used to configure timing)
ENTRY(dbg_tsreset_pulse)
		bl	dbg_tsreset_off
		bl	dbg_serial_bitbang_delay

		bl	dbg_tsreset_on
		bl	dbg_serial_bitbang_delay

		b	dbg_tsreset_pulse
ENDPROC(dbg_tsreset_pulse)

ENTRY(dbg_serial_set_baud)
		stmfd   sp!, {r0 - r6, lr}

		adr	r0, bitbang_baud_rate_count
		ldr	r1, =105987						// 9600
//		mov	r1, #2465						// 115200
		str	r1, [r0]

		adr	r0, bitbang_gpio_addr
		ldr	r1, =(DBG_GPIO_BASE_VADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OUT)
		str	r1, [r0]

		ldmfd   sp!, {r0 - r6, pc}
ENDPROC(dbg_serial_set_baud)

ENTRY(dbg_serial_set_baud_early)
		stmfd   sp!, {r0 - r6, lr}

		adr	r0, bitbang_baud_rate_count
		mov	r1, #9157						// 9600
//		mov	r1, #2465						// 115200
		str	r1, [r0]

		adr	r0, bitbang_gpio_addr
		ldr	r1, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OUT)
		str	r1, [r0]

		ldmfd   sp!, {r0 - r6, pc}
ENDPROC(dbg_serial_set_baud_early)

bitbang_baud_rate_count:	.space	4							// Store count for serial bitbang delay

// Serial time delay
// Pulse width
// 9600 = 104.17us
// 115200 = 8.68us
dbg_serial_bitbang_delay:
		adr	r3, bitbang_baud_rate_count						// Get the address of the delay
		ldr	r3, [r3]								// Get the stored delay value
dbg_serial_bitbang_delay_loop:
		subs	r3, #1
		bne	dbg_serial_bitbang_delay_loop
		mov	pc, lr


bitbang_gpio_addr:		.space	4							// Store for GPIO base address
												// Variable so can cope with change between phyiscal and virtual

// Set bitbang line low
dbg_tsreset_off:
		adr	r6, bitbang_gpio_addr							// Get GPIO address store
		ldr	r6, [r6]								// Load saved address
dbg_tsreset_off_do:
		ldrb	r5, [r6]								// Get current state
		mvn	r4, #(DBG_GPIO_PORT_K_TS_RST)						// Set mask
		and	r5, r4									// Mask current value
		strb	r5, [r6]								// Write value to GPIO register

		mov	pc, lr

// Set bitbang line high
dbg_tsreset_on:
		adr	r6, bitbang_gpio_addr							// Get GPIO address store
		ldr	r6, [r6]								// Load saved address
dbg_tsreset_on_do:
		ldrb	r5, [r6]								// Get current state
		orr	r5, #(DBG_GPIO_PORT_K_TS_RST)						// Update value
		strb	r5, [r6]								// Write value to GPIO register

		mov	pc, lr

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UART hookup

#define DBG_UARTD_ADDR			0x70006300
#define	DBG_UART_OFFSET_THR		0x00			// Transmitter Holding Register (Write - LSR.DLAB clear)
#define	DBG_UART_OFFSET_RBR		0x00			// Receiver Buffer Register (Read - LSR.DLAB clear)
#define	DBG_UART_OFFSET_DLL_LSB		0x00			// Divisor Latch LSByte Register (R/W - LSR.DLAB set)
#define	DBG_UART_OFFSET_IER		0x04			// Interrupt Enable Register (R/W - LSR.DLAB clear)
#define	DBG_UART_OFFSET_DLL_MSB		0x04			// Divisor Latch MSByte Register (R/W  - LSR.DLAB set)
#define	DBG_UART_OFFSET_IIR		0x08			// Interrupt Identification Register (Read)
#define	DBG_UART_OFFSET_FCR		0x08			// FIFO Control Register (Write)
#define DBG_UART_OFFSET_LCR		0x0c			// Line Control Register (R/W)
#define DBG_UART_OFFSET_MCR		0x10			// Modem Control Register (R/W)
#define DBG_UART_OFFSET_LSR		0x14			// Line Status Register (Read)
#define DBG_UART_OFFSET_MSR		0x18			// Modem Status Register (R/W)
#define DBG_UART_OFFSET_SPR		0x1c			// Scratch Pad Register (R/W)
#define	DBG_UART_OFFSET_RX_FIFO_CFG	0x24			// (R/W)
#define	DBG_UART_OFFSET_ASR		0x3c			// Auto Sense Baud Register (R/W)

#define	DBG_UART_MCR_RTS_EN		(1<<6)
#define	DBG_UART_MCR_CTS_EN		(1<<5)
#define	DBG_UART_MCR_LOOPBK		(1<<4)
#define	DBG_UART_MCR_OUT2		(1<<3)
#define	DBG_UART_MCR_OUT1		(1<<2)
#define	DBG_UART_MCR_RTS		(1<<1)
#define	DBG_UART_MCR_DTR		(1<<0)

#define	DBG_UART_LCR_BITS_5		0
#define	DBG_UART_LCR_BITS_6		1
#define	DBG_UART_LCR_BITS_7		2
#define	DBG_UART_LCR_BITS_8		3
#define	DBG_UART_LCR_STOP_ONE		0
#define	DBG_UART_LCR_STOP_TWO		4
#define	DBG_UART_LCR_PARITY_NONE	0
#define	DBG_UART_LCR_PARITY_ODD		8
#define	DBG_UART_LCR_PARITY_MARK	20
#define	DBG_UART_LCR_PARITY_EVEN	24
#define	DBG_UART_LCR_PARITY_SPACE	28
#define	DBG_UART_LCR_DLAB		128

#define	DBG_UART_LSR_THRE		(1<<5)

//ENTRY(dbg_uart_setup)
//		stmfd   sp!, {r0 - r6, lr}
//
////// Pin Mux
////		ldr	r4, =DBG_PINMUX_GMI_A18
////		ldr	r0, [r4]
////		bl	dbg_display_flash_number32
////test_looppm:
////		b	test_looppm
//// DBG_PINMUX_GMI_A18 - 1 1 1 1 1 1 3 1 = 0x00000020
//
//// GPIO
////		ldr	r4, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_CNF)		// Load base address
////		ldr	r1, [r4]								// Get current state
////		orr	r1, #DBG_GPIO_PORT_K_TS_RST						// Enable GPIO pin
////		str	r1, [r4]								// Save config
////
////		ldr	r4, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OE)		// Load base address
////		ldr	r1, [r4]								// Get current state
////		orr	r1, #DBG_GPIO_PORT_K_TS_RST						// Enable GPIO pin output
////		str	r1, [r4]								// Save config

////		ldr	r4, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_K | DBG_GPIO_PORT_OE)		// Load base address
////		ldr	r0, [r4]								// Get current state
////		bl	dbg_display_flash_number32
////test_loop:
////		b	test_loop

//// CAR
////		ldr	r4, =(DBG_CLK_RESET_BASE + DBG_CLK_RST_CONTROLLER_CLK_SOURCE_UARTD_0)
////		ldr	r0, [r4]
////		bl	dbg_display_flash_number32
////test_loop:
////		b	test_loop
//// 1 1 1 1 1 1 1 1 = 0x00000000

////		ldr	r4, =(DBG_CLK_RESET_BASE + DBG_CLK_OUT_ENB_U_OFFSET)
////		ldr	r0, [r4]
////		bl	dbg_display_flash_number32
////test_loop:
////		b	test_loop
//// 1 4 16 1 1 11 1 3 = 0x03f00a02

////		ldr	r4, =(DBG_CLK_RESET_BASE + DBG_RST_DEVICES_U_OFFSET)
////		ldr	r0, [r4]
////		bl	dbg_display_flash_number32
////test_loop:
////		b	test_loop
//// 16 16 16 9 7 1 1 1 = 0xfff86000

//		ldr	r4, =DBG_UARTD_ADDR					// UART base address
//		ldr	r0, [r4, #DBG_UART_OFFSET_MCR]				// Get current MCR state
//		mvn	r1, #(DBG_UART_MCR_RTS_EN | DBG_UART_MCR_CTS_EN)	// Create mask
//		and	r0, r1							// Disable hardware flow control
//		str	r0, [r4, #DBG_UART_OFFSET_MCR]				// Set MCR
//
//		mov	r0, #DBG_UART_LCR_DLAB
//		str	r0, [r4, #DBG_UART_OFFSET_LCR]				// Set DLAB bit to set baud
//
//		mov	r0, #0
//		str	r0, [r4, #DBG_UART_OFFSET_DLL_MSB]
//		mov	r0, #13
//		str	r0, [r4, #DBG_UART_OFFSET_DLL_LSB]			// Set baud 115200
//
//		mov	r0, #(DBG_UART_LCR_BITS_8 | DBG_UART_LCR_PARITY_NONE | DBG_UART_LCR_STOP_ONE)
//		str	r0, [r4, #DBG_UART_OFFSET_LCR]				// Set serial config 8 bits, no parity, 1 stop bit
//
//		ldr	r0, [r4, #DBG_UART_OFFSET_FCR]				// Get current FCM state
//		mvn	r1, #1							// Create mask
//		and	r0, r1							// Clear FCR_EN_FIFO
//		str	r0, [r4, #DBG_UART_OFFSET_FCR]
//
//		mov	r0, #0
//		str	r0, [r4, #DBG_UART_OFFSET_IER]				// Disable all interrupts
//
//test_loop:
//		bl	dbg_display_flash
//
//		ldr	r4, =DBG_UARTD_ADDR					// UART base address
//		mov	r0, #'A'
//		str	r0, [r4, #DBG_UART_OFFSET_THR]				// Write character
//test_loop_test:
//		ldr	r0, [r4, #DBG_UART_OFFSET_LSR]
//		tst	r0, #DBG_UART_LSR_THRE					// See if transmit buffer empty
//		bne	test_loop
//		b	test_loop_test
//
//test_loopa:
//		b	test_loopa
//
//		ldmfd   sp!, {r0 - r6, pc}
//ENDPROC(dbg_uart_setup)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Crude interface to flash lcd display to extract information when no serial console available

// Flash out number
// r0 - Number
ENTRY(dbg_display_flash_number32)
		stmfd   sp!, {r0 - r6, lr}
		mov	r4, r0

		mov	r3, r4, lsr #28
		bl	dbg_display_flash_hexdigit				// 8th digit
		mov	r3, r4, lsr #24
		bl	dbg_display_flash_hexdigit				// 7th digit
		mov	r3, r4, lsr #20
		bl	dbg_display_flash_hexdigit				// 6th digit
		mov	r3, r4, lsr #16
		bl	dbg_display_flash_hexdigit				// 5th digit

		mov	r3, r4, lsr #12
		bl	dbg_display_flash_hexdigit				// 4th digit
		mov	r3, r4, lsr #8
		bl	dbg_display_flash_hexdigit				// 3rd digit
		mov	r3, r4, lsr #4
		bl	dbg_display_flash_hexdigit				// 2nd digit
		mov	r3, r4
		bl	dbg_display_flash_hexdigit				// 1st digit

		ldmfd   sp!, {r0 - r6, pc}
ENDPROC(dbg_display_flash_number32)

// Flash out a hex digit
// r3 - Hex digit
dbg_display_flash_hexdigit:
		push	{lr}
		and	r3, #0x0f						// Make sure we are dealing with one digit
		add	r3, #0x1						// Add 1, so we can easily determine zero
		bl	dbg_lcd_off
		mov	r0, #2000						// 2s delay
		bl	dbg_delay_ms
dbg_display_flash_hexdigit_loop:
		bl	dbg_display_flash
		subs	r3, #1
		bne	dbg_display_flash_hexdigit_loop
		pop	{lr}
		mov	pc, lr

ENTRY(dbg_display_flash_repeat)
dbg_display_flash_repeat:
		bl	dbg_display_flash
		b	dbg_display_flash_repeat
ENDPROC(dbg_display_flash_repeat)

// Turn the display on, then off for a fixed period of time
ENTRY(dbg_display_flash)
		push	{lr}
		bl	dbg_lcd_on
		mov	r0, #750						// 1s delay
		bl	dbg_delay_ms
		bl	dbg_lcd_off
		mov	r0, #750						// 1s delay
		bl	dbg_delay_ms
		pop	{lr}
		mov	pc, lr
ENDPROC(dbg_display_flash)

// Crude programable delay
// r0 - Time delay in millisecs
dbg_delay_ms:
//		mov	r1, #0x30000000						// 1.4 GHz
//		mov	r1, #0x15000						// Pre CPU fixup
		mov	r1, #0x150000						// Post CPU fixup
dbg_delay_ms_loop:
		subs	r1, #1
		bne	dbg_delay_ms_loop
		subs	r0, #1
		bne	dbg_delay_ms
		mov	pc, lr

dbg_lcd_off:
//		ldr	r0, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_H | DBG_GPIO_PORT_OUT)		// Load base address
		ldr	r0, =(DBG_GPIO_BASE_VADDR | DBG_GPIO_PORT_H | DBG_GPIO_PORT_OUT)	// Load base address
		ldrb	r1, [r0]								// Get current state
		mvn	r2, #(DBG_GPIO_PORT_H_LCD_EN | DBG_GPIO_PORT_H_BACKLIGHT)		// Get mask
		and	r1, r2									// Apply mask
		strb	r1, [r0]								// Write value to GPIO register

//		ldr	r0, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_W | DBG_GPIO_PORT_OUT)		// Load base address
		ldr	r0, =(DBG_GPIO_BASE_VADDR | DBG_GPIO_PORT_W | DBG_GPIO_PORT_OUT)	// Load base address
		ldrb	r1, [r0]								// Get current state
		mvn	r2, #(DBG_GPIO_PORT_W_LCD_VDDS)						// Set bits
		and	r1, r2									// Save value
		strb	r1, [r0]								// Write value to GPIO register

		mov	pc, lr

dbg_lcd_on:
//		ldr	r0, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_W | DBG_GPIO_PORT_OUT)		// Load base address
		ldr	r0, =(DBG_GPIO_BASE_VADDR | DBG_GPIO_PORT_W | DBG_GPIO_PORT_OUT)	// Load base address
		ldrb	r1, [r0]								// Get current state
		orr	r1, #(DBG_GPIO_PORT_W_LCD_VDDS)						// Get mask
		strb	r1, [r0]								// Write value to GPIO register

//		ldr	r0, =(DBG_GPIO_BASE_ADDR | DBG_GPIO_PORT_H | DBG_GPIO_PORT_OUT)		// Load base address
		ldr	r0, =(DBG_GPIO_BASE_VADDR | DBG_GPIO_PORT_H | DBG_GPIO_PORT_OUT)	// Load base address
		ldrb	r1, [r0]								// Get current state
		orr	r1, #(DBG_GPIO_PORT_H_LCD_EN | DBG_GPIO_PORT_H_BACKLIGHT)		// Save value
		strb	r1, [r0]								// Write value to GPIO register

		mov	pc, lr

