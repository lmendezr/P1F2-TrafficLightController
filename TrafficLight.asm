;******************************************************************************;
;	INEL4206 - MICROPROCESSORS AND EMBEDDED SYSTEMS
;	P1-Phase 2: Traffic Lights Control System
; Description: Software that controls a corresponding circuit to emulate traffic 
;	lights at an intersection. Such traffic lights are represented by LEDs 
;	connected in a matrix-style circuit which permits the control of such 
;	LEDs by selecting which rows and columns are to be activated at a spe-
;	cific time. Each traffic light includes the three conventional lights 
;	(red, yellow, and green), as well as an arrow to turn left (blue LED).  
;	The street running from north to south is considered the main street 
;	and the one running from west to east is considered secondary.
;
;	Using the internal timer of the MCU, this software is able to analize
;	several factors every second with timed interruptions. These include
;	the current state of the system, the values in the timer registers of 
;	the state, and the values stored from the input in the P2 ports, which
;	simulate the presence of a car in that specific side.
; Traffic Lights Interface Diagram:
;	    | Primary Lights   | Secondary Lights |
;	     _______   _______   _______   _______
;	    |Light 1| |Light 2| |Light 3| |Light 4|
;     P1.4----| GREEN | | GREEN | | GREEN | | GREEN |
;     P1.5----| YELLOW| | YELLOW| | YELLOW| | YELLOW|
;     P1.6----|  RED  | |  RED  | |  RED  | |  RED  |
;     P1.7----| BLUE  | | BLUE  | | BLUE  | | BLUE  |
;	    |_______| |_______| |_______| |_______|
;       ^         |         |         |         |
;    [0 = ON]     |         |         |         |
;	      P1.0      P1.1      P1.2      P1.3  <--[1 = ON]
;
;	Team 4: Lixhjideny Méndez, Yamira Rentas, David Velez
;	University of Puerto Rico, Mayagüez Campus
;	April 3rd, 2015
;	Built with IAR Embedded Workbench Version: 6.30.1
;******************************************************************************;
;-------------------------------------------------------------------------------
; Includes and Initialization
;-------------------------------------------------------------------------------
#include "msp430.h"

;Timer Setup Constants
TRESET	EQU 3	; Amount of time the Red lights reset state will last
TMMIN	EQU 5	; Minimum time green primary can be ON

TSMAX	EQU 5	; Max time green secondary can be ON
TSWAIT	EQU 2	; Max input wait time before moving on from green secondary

TMAXSOLO	EQU 5	; Max time any solo can be ON
TWAITSOLO	EQU 2	; Max input wait time before moving on from any solo

TYELLOW	EQU 2	; Amount of time yellow or delays will last

TINTERVAL	EQU 10	; Interval for each clock interrupt (1 = 0.1 seconds)
MPDELAY	EQU 500	; Delay for a completely off appeareance of the LEDs
		; of the same color caused by the time multiplexing

; Pin output values for the 
; Primary Road Lights
PBLUELED	 EQU 0x6C
PGREENLED	 EQU 0xDC
PYELLOWLED EQU 0xBC
PREDLED	 EQU 0x7C
; Pin output values for the 
; Secondary Road Lights
SBLUELED	 EQU 0x63
SGREENLED	 EQU 0xD3
SYELLOWLED EQU 0xB3
SREDLED	 EQU 0x73
; Bits meanings for the inputFlags 
; and Current State variables
PGREENBIT	EQU BIT0
PSOLOBIT	EQU BIT1
SGREENBIT	EQU BIT2
SSOLOBIT	EQU BIT3
YELLOWBIT	EQU BIT4

;-------------------------------------------------------------------------------
; Variables
;-------------------------------------------------------------------------------
	    ORG 0200h
; Stores the input from the vehicle sensors
; of each traffic light and the current state 
; of the traffic lights
inputFlags    db 0
currentState  db 0

; Stores the amount of times both the time
; and the button interrupts have been executed
; (Memory separated only for debugging purposes)
timeCount     dw 0
pushCount     dw 0

;-------------------------------------------------------------------------------
; Register Definitions
;-------------------------------------------------------------------------------

; Dedicated registers for storing the LED pin values
; of the current state the traffic light is in
#define PLED R11
#define SLED R12

; Dedicated registers for storing the current Timer values
#define TRESETCTR R4 ; Red lights (reset state)
#define TMMINCTR  R5 ; Current time green primary can be ON

#define TSMAXCTR  R6 ; Max time green secondary can be ON
#define TSWAITCTR R7 ; Max time we can wait for input before moving on from green secondary

#define TMAXSOLOCTR R8 ; Max time any solo can be ON
#define TWAITSOLOCTR R9 ; Max time we can wait for input before moving on from any solo

#define TYELLOWCTR  R10 ; Amount of time yellow or delays will last

;--------------------------------------------------------------------------
; Interrupt Vector Table Entries
;--------------------------------------------------------------------------
	ORG  0FFFEh
	DC16 RESET		; set reset vector 
	ORG  0FFF2h 		; MSP430 Timer_A0 Vector Address
	DW   TA0_ISR
	ORG  0FFE6h		; MSP430 Port2 Vector Address
	DW   PORT2_ISR
        
	RSEG CSTACK 		; pre-declaration of segment
	RSEG CODE 		; place program in 'CODE' segment
;--------------------------------------------------------------------------
; M A I N - main driver for the Traffic Light Controller program.
;	Executes the distinct setup subroutines, and prepares
;	the MSP430 for the reset state (all but red leds off).
;	Then, executes the time multiplexing that shows lights
;	on primary and secondary lights as if they were ON at
;	the same time (this is due to hardware limitations).
;--------------------------------------------------------------------------
RESET: 	MOV #SFE(CSTACK), SP	; set up stack
  
main: 	NOP			; main program
	mov #WDTPW+WDTHOLD, &WDTCTL	; stop watchdog timer
	
	call #subSetTimer		; set up the timer
	call #subSetInputs		; set up the P2 for input
	
	clr inputFlags		; clear the input flags and current
	clr currentState		; state variables from any values
	mov.b #PREDLED, PLED	; left from a previous run and set
	mov.b #SREDLED, SLED	; the LEDS to their reset state
	NOP			; Giving time for system to setup	

multiplex;{
	mov.b PLED,&P1OUT
	call #subDelay
	
	mov.b SLED, &P1OUT
	call #subDelay
	
	jmp multiplex
;}

;--------------------------------------------------------------------------
; Setup Subroutines
;--------------------------------------------------------------------------

; Set Timer A_0 at a frequency of 12KHz, enable interrupt generation,
; counting up with an up limit of 1200. With this setup the timer
; will generate an interrupt every 0.1 seconds.
subSetTimer:

	bis.b #LFXT1S_2, &BCSCTL3	    ;very low frequency mode: 10kHz to 50kHz
	mov.w #CCIE, &CCTL0
	mov.w #0110h, &TA0CTL	    ;set frequency to 12KHz and the way to
				    ;count is increasing to a certain limit
	mov.w #1200*TINTERVAL, &TA0CCR0  ;set the limit to get a precision of 1s
	nop
	ret

; Set the P2 port with its P2.0-P2.3 pins as input, leaving the rest of
; the pins as output. Also enables the pullup resistors, interrupts, and
; cleans any flags previously stored in these input pins. Finally enables
; the Global Interrupt in the Status Register.
subSetInputs:
	mov.b #0xF0, &P2DIR
	mov.b #0xFF, &P1DIR
	
	bic.b #0Fh, &P2SEL
	bis.b #0Fh, &P2REN	
	bis.b #0Fh, &P2OUT	 
	bis.b #0Fh, &P2IES
	bis.b #0Fh, &P2IE
	bic.b #0Fh, &P2IFG

	bis #GIE, SR
	ret

; Provides a delay for the time multiplexing method utilized in the program.
subDelay:
	push R15
	mov #MPDELAY, R15
d_wait	dec R15
	jnz d_wait
	pop R15
	ret
;-------------------------------------------------------------------------------
; State Subroutines : Follows the state diagrams by performing 
; the previously established validations in the state diagram:
; 1. Check if yellow/delay wait timer has passed, move from 
;    yellow state if true (jhs)
; 2. Check if the minimum timer has passed, stay in the current
;    state if not (jlo)
; 3. Check the inputs of the other states, move to the first 
;    state with a true bit (jnz), performing the necessary status
;    bis/bic and timer resets to achieve a succesful transition
; 4. If not in primary green state, check if the maximum timer 
;    has passed, move to the primary green state if true (jlo)
; 5. Check the bit of it's own side, stay w/o reseting any 
;    timers if true
; 6. If every previous check failed, move to the green primary 
;    state
;-------------------------------------------------------------------------------

; Primary Solo Light State
subPSolo:
	inc TYELLOWCTR
	cmp #TYELLOW,TYELLOWCTR
	jhs ps_cont
	bit #YELLOWBIT, currentState
	jnz ps_yellow

	mov.b #SREDLED, SLED
	mov.b #PREDLED, PLED
	jmp stay_ps

ps_yellow	mov.b #PREDLED, PLED
	mov.b #SYELLOWLED, SLED
	jmp stay_pg
	
ps_cont	mov.b #PBLUELED, PLED
	mov.b #SREDLED, SLED
	
	inc TMAXSOLOCTR
	inc TWAITSOLOCTR
	cmp #TWAITSOLO, TWAITSOLOCTR
	jlo stay_ps
	
	bit #PGREENBIT, inputFlags
	jnz ps_to_pg
	bit #SSOLOBIT, inputFlags
	jnz ps_to_ss
	bit #SGREENBIT, inputFlags
	jnz ps_to_sg

	cmp #TMAXSOLO, TMAXSOLOCTR
	jhs ps_to_pg
	bit #PSOLOBIT, inputFlags
	jnz ps_to_ps

ps_to_pg	mov.b #PGREENBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #PGREENBIT, inputFlags
	jmp end_ps
ps_to_ss	mov.b #SSOLOBIT, currentState
	bic   #YELLOWBIT, currentState
	bis.b #SSOLOBIT, inputFlags
	jmp end_ps
ps_to_sg	mov.b #SGREENBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #SGREENBIT, inputFlags
	jmp end_ps
ps_to_ps	mov.b #PSOLOBIT, currentState
	bic.b #PSOLOBIT, inputFlags
	jmp stay_ps
	
end_ps	clr TMAXSOLOCTR
	clr TWAITSOLOCTR
	clr TYELLOWCTR
stay_ps	ret

; Primary Green Light State
subPGreen:
	inc TYELLOWCTR
	cmp #TYELLOW,TYELLOWCTR
	jhs pg_cont
	bit #YELLOWBIT, currentState
	jnz pg_yellow

	mov.b #SREDLED, SLED
	mov.b #PREDLED, PLED
	jmp stay_pg

pg_yellow	mov.b #PREDLED, PLED
	mov.b #SYELLOWLED, SLED
	jmp stay_pg
	
pg_cont	mov.b #PGREENLED, PLED
	mov.b #SREDLED, SLED
	
	inc TMMINCTR
	cmp #TMMIN, TMMINCTR
	jlo stay_pg
	
	bit #SSOLOBIT, inputFlags
	jnz pg_to_ss
	bit #SGREENBIT, inputFlags
	jnz pg_to_sg
	bit #PSOLOBIT, inputFlags
	jnz pg_to_ps

pg_to_pg	bic.b #PGREENBIT, inputFlags
	jmp stay_pg
pg_to_ss	mov.b #SSOLOBIT, currentState
	bis   #YELLOWBIT, currentState
	bic.b #SSOLOBIT, inputFlags
	jmp end_pg
pg_to_sg	mov.b #SGREENBIT, currentState
	bis   #YELLOWBIT, currentState
	bic.b #SGREENBIT, inputFlags
	jmp end_pg
pg_to_ps	mov.b #PSOLOBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #PSOLOBIT, inputFlags
	jmp end_pg

end_pg	clr TMMINCTR
	clr TYELLOWCTR
stay_pg	ret

; Secondary Solo Light State
subSSolo:
	inc TYELLOWCTR
	cmp #TYELLOW,TYELLOWCTR
	jhs ss_cont
	bit #YELLOWBIT, currentState
	jnz ss_yellow

	mov.b #SREDLED, SLED
	mov.b #PREDLED, PLED
	jmp stay_ss

ss_yellow	mov.b #PYELLOWLED, PLED
	mov.b #SREDLED, SLED
	jmp stay_pg
	
ss_cont	mov.b #SBLUELED, PLED
	mov.b #PREDLED, SLED
	
	inc TMAXSOLOCTR
	inc TWAITSOLOCTR
	cmp #TWAITSOLO, TWAITSOLOCTR
	jlo stay_ss

	bit #SGREENBIT, inputFlags
	jnz ss_to_sg
	bit #PSOLOBIT, inputFlags
	jnz ss_to_ps
	bit #PGREENBIT, inputFlags
	jnz ss_to_pg

	cmp #TMAXSOLO, TMAXSOLOCTR
	jhs ss_to_pg
	bit #SSOLOBIT, inputFlags
	jnz ss_to_ss


ss_to_pg	mov.b #PGREENBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #PGREENBIT, inputFlags
	jmp end_ss
ss_to_ps	mov.b #PSOLOBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #PSOLOBIT, inputFlags
	jmp end_ss
ss_to_sg	mov.b #SGREENBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #SGREENBIT, inputFlags
	jmp end_ss
ss_to_ss	mov.b #SSOLOBIT, currentState
	bic.b #SSOLOBIT, inputFlags
	jmp stay_ss
	
end_ss	clr TMAXSOLOCTR
	clr TWAITSOLOCTR
	clr TYELLOWCTR
stay_ss	ret

; Secondary Green Light State
subSGreen:
	inc TYELLOWCTR
	cmp #TYELLOW,TYELLOWCTR
	jhs sg_cont
	bit #YELLOWBIT, currentState
	jnz sg_yellow

	mov.b #SREDLED, SLED
	mov.b #PREDLED, PLED
	jmp stay_sg

sg_yellow	mov.b #PYELLOWLED, PLED
	mov.b #SREDLED, SLED
	jmp stay_sg


sg_cont	mov.b #PREDLED, PLED
	mov.b #SGREENLED, SLED
	
	inc TSMAXCTR
	inc TSWAITCTR
	cmp #TSWAIT, TSWAITCTR
	jlo stay_sg

	bit #PSOLOBIT, inputFlags
	jnz sg_to_ps
	bit #PGREENBIT, inputFlags
	jnz sg_to_pg
	bit #SSOLOBIT, inputFlags
	jnz sg_to_ss

	cmp #TSMAX, TSMAXCTR
	jhs sg_to_pg
	bit #SGREENBIT, inputFlags
	jnz sg_to_sg

sg_to_pg	mov.b #PGREENBIT, currentState
	bis   #YELLOWBIT, currentState
	bic.b #PGREENBIT, inputFlags
	jmp end_sg
sg_to_ss	mov.b #SSOLOBIT, currentState
	bic   #YELLOWBIT, currentState
	bic.b #SSOLOBIT, inputFlags
	jmp end_sg
sg_to_ps	mov.b #PSOLOBIT, currentState
	bis   #YELLOWBIT, currentState
	bic.b #PSOLOBIT, inputFlags
	jmp end_sg
sg_to_sg	mov.b #SGREENBIT, currentState
	bic.b #SGREENBIT, inputFlags
	jmp stay_sg
	
end_sg	clr TSMAXCTR
	clr TSWAITCTR
	clr TYELLOWCTR
stay_sg	ret

; Reset State
subReset:	
	mov.b #PREDLED, PLED
	mov.b #SREDLED, SLED
	inc TRESETCTR
	cmp #TRESET, TRESETCTR
	jlo stay_rs

	bit #PSOLOBIT, inputFlags
	jnz rs_to_ps
	bit #PGREENBIT, inputFlags
	jnz rs_to_pg
	bit #SSOLOBIT, inputFlags
	jnz rs_to_ss
	bit #SGREENBIT, inputFlags
	jnz rs_to_sg

rs_to_pg	mov.b #PGREENBIT, currentState
	bic.b #PGREENBIT, inputFlags
	jmp end_rs
rs_to_ps	mov.b #PSOLOBIT, currentState
	bic.b #PSOLOBIT, inputFlags
	jmp end_rs
rs_to_ss	mov.b #SSOLOBIT, currentState
	bic.b #SSOLOBIT, inputFlags
	jmp end_rs
rs_to_sg	mov.b #SGREENBIT, currentState
	bic.b #SGREENBIT, inputFlags
	jmp end_rs

end_rs	clr TRESETCTR
stay_rs	ret
;-------------------------------------------------------------------------------
; The Timer ISR - Checks for the current state every interval previously 
; 		specified. Then proceeds to the respective subroutine.
;-------------------------------------------------------------------------------
TA0_ISR 	
	inc &timeCount
CheckStates
	bit.b #PSOLOBIT, currentState
	jnz PSoloState

	bit.b #PGREENBIT, currentState
	jnz PGreenState

	bit.b #SSOLOBIT, currentState
	jnz SSoloState

	bit.b #SGREENBIT, currentState
	jnz SGreenState
	jmp ResetState

PSoloState
	call #subPSolo
	jmp finish

PGreenState
	call #subPGreen
	jmp finish

SSoloState
	call #subSSolo
	jmp finish

SGreenState
	call #subSGreen
	jmp finish

ResetState
	call #subReset

finish	reti

;-------------------------------------------------------------------------------
; The Input ISR - Checks for an input in the P2.0 port, stores the received 
;		input in memory, and clears all input flags. 
;-------------------------------------------------------------------------------
PORT2_ISR
	bis.b &P2IFG, inputFlags
	bic.b #0xFF, &P2IFG
	inc pushCount
	reti
;-------------------------------------------------------------------------------
; This is the end.
;-------------------------------------------------------------------------------
	END