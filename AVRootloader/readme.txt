changes for building avrootloader firmware for rx3s vs aquastar

1. AVRootloader.asmproj 
project/avrootloader properties/device/atmega168pa 
vs 
project/avrootloader properties/device/atmega8

---

2. AVR\AVRootloader.asm

; RX3S  
;.include "m168PAdef.inc"				; ATmega168PA
vs
; Aquastar
.include "m8Adef.inc"					; ATmega8A   

; RX3S
;.equ	RX_PORT				= PORTD		; Receive port and pin
;.equ	RX					= PD7
;.equ	TX_PORT				= PORTD		; Transmit port and pin
;.equ	TX					= PD7
vs
; Aquastar
.equ	RX_PORT				= PORTD		; Receive port and pin
.equ	RX					= PD0
.equ	TX_PORT				= PORTD		; Transmit port and pin
.equ	TX					= PD0

---

3. AVR\AVRootloader.inc

; RX3S
;		jmp	FLASHEND +1							; run application
vs
; Aquastar
    rjmp FLASHEND +1							; run application