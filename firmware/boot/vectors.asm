; Reset first makes SCR and the relay drive safe, and stops what a stray jump to 0 would leave
; running. Each interrupt vector jumps to the same vector in the application, at APP_BASE.

	.module vectors
	.globl	__sdcc_gsinit_startup

	.include	"vectors.inc"

	.macro	vector	addr
	.org	addr
	ljmp	APP_BASE + addr
	.endm

	.area	VECT	(ABS,CODE)

	.org	0x0000
	ljmp	safe_reset

	vector	0x0003		; INT0
	vector	0x000B		; Timer 0
	vector	0x0013		; INT1
	vector	0x001B		; Timer 1
	vector	0x0023		; UART1
	vector	0x002B		; ADC
	vector	0x0033		; LVD
	vector	0x003B		; PCA
	vector	0x0043		; UART2
	vector	0x004B		; SPI
	vector	0x0053		; INT2
	vector	0x005B		; INT3
	vector	0x0063		; Timer 2
	vector	0x006B
	vector	0x0073
	vector	0x007B
	vector	0x0083		; INT4
	vector	0x008B		; UART3
	vector	0x0093		; UART4
	vector	0x009B		; Timer 3
	vector	0x00A3		; Timer 4
	vector	0x00AB		; Comparator

	SAFE_RESET = 0x00B0
	.org	SAFE_RESET
safe_reset:
	clr	EA
	mov	CCAPM0,#0x00	; before the gate pin is cleared
	clr	CR
	clr	PIN_SCR
	clr	PIN_RELAY
	orl	P3M0,#(1 << (PIN_SCR & 7)) | (1 << (PIN_RELAY & 7))	; push-pull: their P3M1 bits stay 0
	ljmp	__sdcc_gsinit_startup
safe_reset_end:
	.iflt	BOOT_CODE_BASE - SAFE_RESET - (safe_reset_end - safe_reset)
	.error	1		; safe_reset runs into the C code at BOOT_CODE_BASE
	.endif
