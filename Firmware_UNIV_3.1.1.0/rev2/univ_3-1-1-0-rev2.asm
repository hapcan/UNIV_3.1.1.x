;==============================================================================
;   HAPCAN - Home Automation Project Firmware (http://hapcan.com)
;   Copyright (C) 2014 hapcan.com
;
;   This program is free software: you can redistribute it and/or modify
;   it under the terms of the GNU General Public License as published by
;   the Free Software Foundation, either version 3 of the License, or
;   (at your option) any later version.
;
;   This program is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;   GNU General Public License for more details.
;
;   You should have received a copy of the GNU General Public License
;   along with this program.  If not, see <http://www.gnu.org/licenses/>.
;==============================================================================
;   Filename:              univ_3-1-1-0.asm
;   Associated diagram:    univ_3-1-1-x.sch
;   Author:                Jacek Siwilo                          
;   Note:                  13 channel button & temp module
;==============================================================================
;   Revision History
;   Rev:  Date:     Details:
;   0     08.2013   Original version
;   1     09.2013   ID number added in code to each transmitted message
;   2     05.2014   Temperature & thermostat messages are sent periodically
;==============================================================================
;===  FIRMWARE DEFINITIONS  =================================================== 
;==============================================================================
    #define    ATYPE    .1                            ;application type [0-255]
    #define    AVERS    .1                         ;application version [0-255]
    #define    FVERS    .0                            ;firmware version [0-255]

    #define    FREV     .2                         ;firmware revision [0-65536]
;==============================================================================
;===  NEEDED FILES  ===========================================================
;==============================================================================
    LIST P=18F26K80                              ;directive to define processor
    #include <P18F26K80.INC>           ;processor specific variable definitions
    #include "univ_3-1-1-0-rev2.inc"                         ;project variables
    #include "univ3-fake_bootloader-rev2.inc"    ;fake bootloader for debugging

INCLUDEDFILES   code    
    #include "univ3-routines-rev3.inc"                     ;UNIV 3 CPU routines
	#include "univ3-1-wire-rev0.inc"                          ;1-wire functions

;==============================================================================
;===  FIRMWARE CHECKSUM  ======================================================
;==============================================================================
FIRMCHKSM   code    0x001000
    DB      0x5B, 0xA9, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF			
;==============================================================================
;===  FIRMWARE ID  ============================================================
;==============================================================================
FIRMID      code    0x001010
    DB      0x30, 0x00, 0x03,ATYPE,AVERS,FVERS,FREV>>8,FREV
;            |     |     |     |     |     |     |_____|_____ firmware revision
;            |     |     |     |     |     |__________________ firmware version
;            |     |     |     |     |_____________________ application version
;            |     |     |     |______________________________ application type
;            |     |     |________________________________ hardware version '3'
;            |_____|______________________________________ hardware type 'UNIV'
;==============================================================================
;===  MOVED VECTORS  ==========================================================
;==============================================================================
;PROGRAM RESET VECTOR
FIRMRESET   code    0x1020
        goto    Main
;PROGRAM HIGH PRIORITY INTERRUPT VECTOR
FIRMHIGHINT code    0x1030
        call    HighInterrupt
        retfie
;PROGRAM LOW PRIORITY INTERRUPT VECTOR
FIRMLOWINT  code    0x1040
        call    LowInterrupt
        retfie

;==============================================================================
;===  FIRMWARE STARTS  ========================================================
;==============================================================================
FIRMSTART   code    0x001050
;------------------------------------------------------------------------------
;---  LOW PRIORITY INTERRUPT  -------------------------------------------------
;------------------------------------------------------------------------------
LowInterrupt
        movff   STATUS,STATUS_LOW           ;save STATUS register
        movff   WREG,WREG_LOW               ;save working register
        movff   BSR,BSR_LOW                 ;save BSR register
        movff   FSR0L,FSR0L_LOW             ;save other registers used in high int
        movff   FSR0H,FSR0H_LOW
        movff   FSR1L,FSR1L_LOW
        movff   FSR1H,FSR1H_LOW

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMREADY,0
        bra     ExitLowInterrupt            ;main firmware is not ready yet
    ;CAN buffer
        movlb   0x1
        btfsc   CANFULL,0                   ;check if CAN received anything
        call    CANInterrupt                ;proceed with CAN interrupt

ExitLowInterrupt
        movff   BSR_LOW,BSR                 ;restore BSR register
        movff   WREG_LOW,WREG               ;restore working register
        movff   STATUS_LOW,STATUS           ;restore STATUS register
        movff   FSR0L_LOW,FSR0L             ;restore other registers used in high int
        movff   FSR0H_LOW,FSR0H
        movff   FSR1L_LOW,FSR1L
        movff   FSR1H_LOW,FSR1H
    return

;------------------------------------------------------------------------------
;---  HIGH PRIORITY INTERRUPT  ------------------------------------------------
;------------------------------------------------------------------------------
HighInterrupt
        movff   STATUS,STATUS_HIGH          ;save STATUS register
        movff   WREG,WREG_HIGH              ;save working register
        movff   BSR,BSR_HIGH                ;save BSR register
        movff   FSR0L,FSR0L_HIGH            ;save other registers used in high int
        movff   FSR0H,FSR0H_HIGH
        movff   FSR1L,FSR1L_HIGH
        movff   FSR1H,FSR1H_HIGH

    ;main firmware ready flag
        movlb   0x1
        btfss   FIRMREADY,0
        bra     ExitHighInterrupt           ;main firmware is not ready yet
    ;Timer0
		btfsc	INTCON,TMR0IF			    ;Timer0 interrupt? (1000ms)
		rcall	Timer0Interrupt
	;Timer2	
        btfsc	PIR1,TMR2IF		    	    ;Timer2 interrupt? (20ms)
		rcall	Timer2Interrupt

ExitHighInterrupt
        movff   BSR_HIGH,BSR                ;restore BSR register
        movff   WREG_HIGH,WREG              ;restore working register
        movff   STATUS_HIGH,STATUS          ;restore STATUS register
        movff   FSR0L_HIGH,FSR0L            ;restore other registers used in high int
        movff   FSR0H_HIGH,FSR0H
        movff   FSR1L_HIGH,FSR1L
        movff   FSR1H_HIGH,FSR1H
    return

;------------------------------------------------------------------------------
; Routine:			CAN INTERRUPT
;------------------------------------------------------------------------------
; Overview:			Checks CAN message for response and RTR and saves to FIFO
;------------------------------------------------------------------------------
CANInterrupt
		banksel CANFRAME2
		btfsc	CANFRAME2,0				    ;response message?
	return                                  ;yes, so ignore it and exit
		btfsc	CANFRAME2,1                 ;RTR (Remote Transmit Request)?
	return                                  ;yes, so ignore it and exit
        call    Copy_RXB_RXFIFOIN           ;copies received message to CAN RX FIFO input buffer
        call    WriteToCanRxFIFO		    ;saves message to FIFO
    return

;------------------------------------------------------------------------------
; Routine:			TIMER 0 INTERRUPT
;------------------------------------------------------------------------------
; Overview:			1000ms periodical interrupt
;------------------------------------------------------------------------------
Timer0Interrupt:
        call    Timer0Initialization8MHz    ;restart 1000ms Timer   
        call    UpdateUpTime                ;counts time from restart
        call    UpdateTransmitTimer         ;increment transmit timer (seconds after last transmission)
        banksel TIMER0_1000ms
        setf    TIMER0_1000ms               ;timer 0 interrupt occurred flag
    return

;------------------------------------------------------------------------------
; Routine:			TIMER 2 INTERRUPT
;------------------------------------------------------------------------------
; Overview:			20ms periodical interrupt
;------------------------------------------------------------------------------
Timer2Interrupt
        rcall   Timer2Initialization        ;restart timer
        rcall   EnableInputs                ;set port pins as inputs
        rcall   ReadInputs                  ;read port
        rcall   DisableInputs               ;set port pins as outputs
		rcall	CnvrtTime			    	;counts temp conversion time
        banksel TIMER2_20ms
        setf    TIMER2_20ms                 ;timer 2 interrupt occurred flag
    return
;-------------------------------
Timer2Initialization
        movlb   0xF
        bcf     PMD1,TMR2MD                 ;enable timer 2
        movlw   0x3F          
		movwf	TMR2                        ;set 20ms (19.999500)
		movlw	b'01001111'				    ;start timer, prescaler=16, postscaler=10
		movwf	T2CON
        bsf     IPR1,TMR2IP                 ;high priority for interrupt
		bcf		PIR1,TMR2IF			        ;clear timer's flag
		bsf		PIE1,TMR2IE			        ;interrupt on
    return
;-------------------------------
EnableInputs
        ;it charges cap if diode was off
        setf    LATC                        
        movlw   b'00101100'                 ;set appropriate pins
        iorwf   LATA
        movlw   b'00010001'
        iorwf   LATB
        ;it sets pins as inputs
        setf    TRISC                       ;set appropriate pins
        movlw   b'00101100'
        iorwf   TRISA
        movlw   b'00010001'
        iorwf   TRISB
    return
;-------------------------------
ReadInputs
        ;move current states to Buttons
        movff   PORTC,ButtonsA              ;buttons 1-8
        setf    ButtonsB
        btfss   PORTA,2                     ;button 9
        bcf     ButtonsB,0
        btfss   PORTA,3                     ;button 10
        bcf     ButtonsB,1  
        btfss   PORTA,5                     ;button 11
        bcf     ButtonsB,2      
        btfss   PORTB,4                     ;button 12
        bcf     ButtonsB,3
        btfss   PORTB,0                     ;button 13
        bcf     ButtonsB,4
    return
;-------------------------------
DisableInputs
        ;move right led status
        movff   DiodesA,LATC  
        bcf     LATA,2                      ;button 9
        btfsc   DiodesB,0               
        bsf     LATA,2
        bcf     LATA,3                      ;button 10
        btfsc   DiodesB,1               
        bsf     LATA,3
        bcf     LATA,5                      ;button 11
        btfsc   DiodesB,2               
        bsf     LATA,5
        bcf     LATB,4                      ;button 12
        btfsc   DiodesB,3               
        bsf     LATB,4
        bcf     LATB,0                      ;button 13
        btfsc   DiodesB,4               
        bsf     LATB,0
        ;set pins as outputs
        clrf    TRISC
        movlw   b'11010011'                 ;clear appropriate pins
        andwf   TRISA
        movlw   b'11101110'
        andwf   TRISB
    return
;-------------------------------
CnvrtTime
		setf	WREG
		xorwf	CNVRTFLAG,W				    ;if flag = FF do not change
		bz		$ + 6
		tstfsz	CNVRTFLAG				    ;decrement conversion in progress flag
		decf	CNVRTFLAG
	return

;==============================================================================
;===  MAIN PROGRAM  ===========================================================
;==============================================================================
Main:
    ;disable global interrupts for startup
        call    DisAllInt                   ;disable all interrupts 
    ;firmware initialization
        rcall   PortInitialization          ;prepare processor ports
        call    GeneralInitialization       ;read eeprom config, clear other registers
        call    FIFOInitialization          ;prepare FIFO buffers
		call	Timer0Initialization8MHz    ;Timer 0 initialization for 1s periodical interrupt 
        call    Timer2Initialization        ;Timer 2 initialization for 20ms periodical interrupt
		call	ButtonPowerUpValues         ;button on power up values
		call	TempPowerUpValues           ;temperature on power up values
    ;firmware ready
        banksel FIRMREADY
        bsf     FIRMREADY,0                 ;set flag "firmware started and ready for interrupts"
    ;enable global interrupts
        call    EnAllInt                    ;enable all interrupts

;-------------------------------
Loop:                                       ;main loop
        clrwdt                              ;clear Watchdog timer
        call    ReceiveProcedure            ;check if any msg in RX FIFO and if so - process the msg
        call    TransmitProcedure           ;check if any msg in TX FIFO and if so - transmit it
        rcall   OnceA20ms                   ;do routines only after 20ms interrupt 
        rcall   OnceA1000ms                 ;do routines only after 1000ms interrupt
    bra     Loop

;-------------------------------
OnceA20ms                                   ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER2_20ms
        tstfsz  TIMER2_20ms                 ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
        call    RecognizeButtons            ;recognize what button is pressed
        banksel TIMER2_20ms
        clrf    TIMER2_20ms
    return
;-------------------------------
OnceA1000ms                                 ;procedures executed once per 1000ms (flag set in interrupt)
        banksel TIMER0_1000ms
        tstfsz  TIMER0_1000ms               ;flag set?
        bra     $ + 4
    return                                  ;no, so exit
		call	ReadTemperature			    ;read ds sensor
        call    CnvrtTemperature            ;do conversion in DS sensor
        call    SaveSateToEeprom            ;save thermostat value into eeprom memory when needed
        call    UpdateHealthRegs            ;saves health maximums to eeprom
        banksel TIMER0_1000ms
        clrf    TIMER0_1000ms
    return


;==============================================================================
;===  FIRMWARE ROUTINES  ======================================================
;==============================================================================
;------------------------------------------------------------------------------
; Routine:			PORT INITIALIZATION
;------------------------------------------------------------------------------
; Overview:			It sets processor pins. All unused pins should be set as
;                   outputs and driven low
;------------------------------------------------------------------------------
PortInitialization                          ;default all pins set as analog (portA,B) or digital (portB,C) inputs 
    ;PORT A
        banksel ANCON0                      ;select memory bank
        ;0-digital, 1-analog input
        movlw   b'11100011'                 ;(x,x,x,AN4,AN3,AN2,AN1-boot_mode,AN0-volt)
        movwf   ANCON0
        ;output level
        clrf    LATA                        ;all low
        ;0-output, 1-input
        movlw   b'11111111'                 ;all inputs
        movwf   TRISA       
    ;PORT B
        ;0-digital, 1-analog input
        movlw   b'11111000'                 ;(x,x,x,x,x,AN10,AN9,AN8)
        movwf   ANCON1
        ;output level
        clrf    LATB                        ;all low
        ;0-output, 1-input
        movlw   b'00011011'                 ;all output except CANRX & button inputs
        movwf   TRISB
    ;PORT C
        ;output level
        clrf    LATC                        ;all low
        ;0-output, 1-input
        movlw   b'11111111'                 ;all intput 
        movwf   TRISC
    return

;------------------------------------------------------------------------------
; Routine:			NODE STATUS
;------------------------------------------------------------------------------
; Overview:			It prepares status messages when status request was
;                   received
;------------------------------------------------------------------------------
NodeStatusRequest

;------buttons---------------
ButtonStatus: MACRO ButNr,ButReg,DiodeReg,RegBit ;macro sends status for chosen button
		movlw	ButNr                       ;button x
		movwf	TXFIFOIN6
		setf	WREG                        ;0xFF - pressed
		btfsc	ButReg,RegBit
		clrf	WREG                        ;0x00 - released
		movwf	TXFIFOIN7
		setf	WREG                        ;0xFF - diode on
		btfss	DiodeReg,RegBit
		clrf	WREG                        ;0x00 - diode off
		movwf	TXFIFOIN8
		rcall	SendButtonStatus
    ENDM
;------------
        banksel TXFIFOIN0
        ButtonStatus   .1,ButtonsA,DiodesA,0  ;button 1, call macro, /macro_arg: Button_No, Button_REGISTER, Diode_REGISTER, Register_PIN/
        ButtonStatus   .2,ButtonsA,DiodesA,1  ;button 2
        ButtonStatus   .3,ButtonsA,DiodesA,2  ;button 3
        ButtonStatus   .4,ButtonsA,DiodesA,3  ;button 4
        ButtonStatus   .5,ButtonsA,DiodesA,4  ;button 5
        ButtonStatus   .6,ButtonsA,DiodesA,5  ;button 6
        ButtonStatus   .7,ButtonsA,DiodesA,6  ;button 7
        ButtonStatus   .8,ButtonsA,DiodesA,7  ;button 8
        ButtonStatus   .9,ButtonsB,DiodesB,0  ;button 9
        ButtonStatus  .10,ButtonsB,DiodesB,1  ;button 10
        ButtonStatus  .11,ButtonsB,DiodesB,2  ;button 11
        ButtonStatus  .12,ButtonsB,DiodesB,3  ;button 12
        ButtonStatus  .13,ButtonsB,DiodesB,4  ;button 13
        bra     TempStatus
;------------
SendButtonStatus
        movlw   0x30			            ;set relay frame
		movwf   TXFIFOIN0
        movlw   0x10
		movwf   TXFIFOIN1
		bsf		TXFIFOIN1,0					;response bit
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
		setf	TXFIFOIN4				    ;unused
		setf	TXFIFOIN5				    ;unused
		setf	TXFIFOIN9				    ;unused
		setf	TXFIFOIN10				    ;unused
		setf	TXFIFOIN11				    ;unused
		call	WriteToCanTxFIFO
	return

;------temperature-----------
TempStatus
        banksel TXFIFOIN0
		tstfsz	ERRORREG			        ;CheckIfError
	bra		SendDSError
		movlw	0x11					    ;"sensor 1"
		movwf	TXFIFOIN6				
		movf	TEMPMSB,W                   ;current temperature
		movwf	TXFIFOIN7
		movf	TEMPLSB,W
		movwf	TXFIFOIN8
		movff	THERMMSB,TXFIFOIN9			;set thermostat temp
		movff	THERMLSB,TXFIFOIN10
		movff	HISTERE,TXFIFOIN11          ;hysteresis
		rcall	SendTempStatus
ThermostatStatus:						    ;sends status after instruction
		movlw	0x12					    ;"thermostat 1"
		movwf	TXFIFOIN6
		movff	STHERMOS,TXFIFOIN7          ;thermostat status
		setf	TXFIFOIN8			    	;unused
		setf	TXFIFOIN9			    	;unused
		setf	TXFIFOIN10			    	;unused
		setf	TXFIFOIN11			    	;unused
		rcall	SendTempStatus
	return
SendDSError:
		movlw	0xF0					    ;0xF0 - ERROR FRAME
		movwf	TXFIFOIN6
		movff	ERRORREG,TXFIFOIN7			;load error flag
		setf	TXFIFOIN8			    	;unused
		setf	TXFIFOIN9			    	;unused
		setf	TXFIFOIN10			    	;unused
		setf	TXFIFOIN11			    	;unused
		rcall	SendTempStatus
	return
SendTempStatus
        banksel TXFIFOIN0
		movlw	0x30					    ;set frame type
		movwf	TXFIFOIN0
		movlw	0x40
		movwf	TXFIFOIN1
		bsf		TXFIFOIN1,0			    	;set response bit
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
		setf	TXFIFOIN4
		setf	TXFIFOIN5
		;TXFIFOIN6 - TXFIFOIN11 are already changed
		call	WriteToCanTxFIFO
    return

;------------------------------------------------------------------------------
; Routine:			DO INSTRUCTION
;------------------------------------------------------------------------------
; Overview:			Executes instruction immediately or sets timer for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionRequest
        banksel INSTR1                      ;allow only known values
        movlw   0x06                        ;INSTR less than?
        cpfslt  INSTR1
    bra     ExitDoInstructionRequest        ;no, so exit
        movf    INSTR1,W                    ;recognize instruction
        call    ComputedGoto                ;modify PC according to WREG value
        bra     Instr00                     ;instruction 00
        bra     Instr01                     ;instruction 01
        bra     Instr02                     ;instruction 02
        bra     Instr03                     ;instruction 03
        bra     Instr04                     ;instruction 04
        bra     Instr05                     ;instruction 05
ExitDoInstructionRequest
    return

;-------------------------------
;Instruction execution
Instr00                                     ;turn off diode
        movf    INSTR2,W                    ;get mask of channels to change (buttons 1-8)
		comf	WREG                        ;modify mask
		andwf	DiodesA,F
        movf    INSTR3,W                    ;(buttons 9-13)
		comf	WREG
		andwf	DiodesB,F
		bra		ExitDoInstructionNow 
Instr01                                     ;turn on diode
        movf    INSTR2,W                    ;get mask of channels to change (buttons 1-8)
		iorwf	DiodesA,F
        movf    INSTR3,W                    ;(buttons 9-13)
		iorwf	DiodesB,F
		bra		ExitDoInstructionNow 
Instr02                                     ;toggle
        movf    INSTR2,W                    ;get mask of channels to change (buttons 1-8)
		xorwf	DiodesA,F
        movf    INSTR3,W                    ;(buttons 9-13)
		xorwf	DiodesB,F
		bra		ExitDoInstructionNow 
Instr03									    ;set THERMOSTAT to INSTR2,INSTR3
		movff	INSTR2,THERMMSB
        movff   INSTR3,THERMLSB
        rcall   EepromToSave
		call	TempStatus				    ;give quick response and send value of new thermos
		bra		ExitDoInstructionNow
Instr04									    ;decrement THERM INSTR2 times (each time = 0.0625deg)
		movlw	0xFC					    ;min 0xFC90 = -55deg
		xorwf	THERMMSB,W
		bnz		$ + .8                      ;not yet
		movlw	0x90
		xorwf	THERMLSB,W
		bz		$ + .12                     ;it's max
        decf    THERMLSB                    ;decrement thermostat LSB
        bc      $ + 4                       ;branch if not borrow b=!c     
        decf    THERMMSB                    ;and borrow if underflowed
		decfsz	INSTR2
		bra		Instr04
        rcall   EepromToSave
		call	TempStatus			    	;give quick response and send value of new thermos
		bra		ExitDoInstructionNow
Instr05									    ;increment THERM INSTR2 times (each time = 0.0625deg)
		movlw	0x07					    ;max 0x07D0 = 125deg
		xorwf	THERMMSB,W
		bnz		$ + .8                      ;not yet
		movlw	0xD0
		xorwf	THERMLSB,W
		bz		$ + .10                     ;it's max
		infsnz	THERMLSB                    ;increment termostat LSB
        incf    THERMMSB                    ;and MSB if LSB overflowed
		decfsz	INSTR2
		bra		Instr05
        rcall   EepromToSave
		call	TempStatus			    	;give quick response and send value of new thermos
		bra		ExitDoInstructionNow
ExitDoInstructionNow
		setf	INSTR1			    	    ;clear instruction
   ;    clrf    TIMER                       ;clear timer /timer not used/
   ;    call	DoInstructionLater          ;clear waiting instruction for channel indicated in INSTR2 /timer not used/
	return
;------------	        	
EepromToSave                                ;indicate that save to eeprom nedded
        banksel EEPROMTIMER
		movlw	0x06					    ;wait 6s before saving to eeprom
		movwf	EEPROMTIMER
    return

;------------------------------------------------------------------------------
; Routine:			DO INSTRUCTION LATER
;------------------------------------------------------------------------------
; Overview:			It saves instruction for particular channel for later
;                   execution
;------------------------------------------------------------------------------
DoInstructionLater
	return

;==============================================================================
;                   BUTTON PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:			BUTTON POWER UP VALUES
;------------------------------------------------------------------------------
; Overview:			Sets registers at power up 
;------------------------------------------------------------------------------
ButtonPowerUpValues
		setf	ButtonsA        		    ;buttons status as released
		setf	ButtonsB
        clrf    DiodesA                     ;LEDs off              
        clrf    DiodesB
        movlw   .13                         ;clear counters            
        lfsr    FSR0,BUT1Cnt
        clrf    POSTINC0
        decfsz  WREG
        bra     $ - 4
    return

;------------------------------------------------------------------------------
; Routine:			RECOGNIZE BUTTONS
;------------------------------------------------------------------------------
; Overview:			Recognizes which button is pressed and for how long.
;                   Routine also sends button message to the CAN bus. 
;------------------------------------------------------------------------------
RecognizeButtons
		call	Button1_ON
		call	Button1_OFF
		call	Button2_ON
		call	Button2_OFF
		call	Button3_ON
		call	Button3_OFF
		call	Button4_ON
		call	Button4_OFF
		call	Button5_ON
		call	Button5_OFF
		call	Button6_ON
		call	Button6_OFF
		call	Button7_ON
		call	Button7_OFF
		call	Button8_ON
		call	Button8_OFF
		call	Button9_ON
		call	Button9_OFF
		call	Button10_ON
		call	Button10_OFF
		call	Button11_ON
		call	Button11_OFF
		call	Button12_ON
		call	Button12_OFF
		call	Button13_ON
		call	Button13_OFF
    return

;----------------------------
Button_IncCnt:MACRO ButCnt                  ;increment but don't overflow button counter
		incfsz	ButCnt
		bra		$ + 4
		decf	ButCnt
    ENDM
;------------
Button_Pressed:MACRO ButNr,ButCnt,DiodeReg,DiodeBit  ;counter equal 2 (40ms-button pressed)
        banksel BUTCnfg                     ;turn on in config?
		btfss	BUTCnfg+ButNr-1,0
	bra $ + .30                             ;no - go to macro end
        movlw   .2                          ;counter = 2?
        cpfseq  ButCnt
	bra $ + .24                             ;no - go to macro end	
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6					
		movlw   0xFF    
        movwf   TXFIFOIN7                   ;button code 0xFF - pressed          
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
    ENDM
;------------
Button_400ms:MACRO ButNr,ButCnt,DiodeReg,DiodeBit   ;counter equal 20 (400ms-button pressed)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,1
	bra $ + .30                             ;no - go to macro end
		movlw   .20                         ;counter =20?
		cpfseq  ButCnt                      ;skip if so
	bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
		movlw   0xFE    
        movwf   TXFIFOIN7                   ;button code 0xFE - pressed for 400ms
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
		call    TransmitButton
	ENDM
;------------
Button_4s:MACRO ButNr,ButCnt,DiodeReg,DiodeBit      ;counter equal 200 (4s-button pressed)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,2
	bra $ + .30                             ;no - go to macro end
		movlw   .200                        ;counter =200?
		cpfseq  ButCnt                      ;skip if so
	bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
		movlw   0xFD    
        movwf   TXFIFOIN7                   ;button code 0xFD - pressed for 4s
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
		call    TransmitButton
	ENDM
;------------
Button_Released:MACRO ButNr,ButCnt,DiodeReg,DiodeBit    ;counter >2 (released after 20ms)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,3
	bra $ + .32                             ;no - go to macro end
        movlw   .2                          ;counter >=2?
        cpfslt  ButCnt                      ;to send msg counter must be at least 2 to make sure "pressed msg" was send
        bra     $ + 4                       ;if counter <2 means button was in the same state, so do not send msg
	bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6					
		movlw   0x00    
        movwf   TXFIFOIN7                   ;button code 0x00 - released          
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
        call    TransmitButton
	ENDM
;------------
Button_60_400ms:MACRO ButNr,ButCnt,DiodeReg,DiodeBit    ;2 < counter < 20 (release before 400ms)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,4
	bra $ + .38                             ;no - go to macro end
        movlw   .2                          ;counter >=2?
        cpfslt  ButCnt                      ;to send msg counter must be at least 2 to make sure "pressed msg" was send
        bra     $ + 4                       ;if counter <2 means button was in the same state, so do not send msg
	bra $ + .30                             ;no - go to macro end
        movlw   .20                         ;counter <20?
        cpfslt  ButCnt                      ;skip if so
	bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
		movlw   0xFC    
        movwf   TXFIFOIN7                   ;button code 0xFC - released within 400ms
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
		call    TransmitButton
	ENDM
;------------
Button_400_4s:MACRO ButNr,ButCnt,DiodeReg,DiodeBit  ;20 < counter < 200 (released between 400ms and 4s)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,5
	bra $ + .38                             ;no - go to macro end
        movlw   .20                         ;counter >=20?
        cpfslt  ButCnt                      ;to send msg counter must be at least 20
        bra     $ + 4                       ;if not do not send msg
	bra $ + .30                             ;no - go to macro end
        movlw   .200                        ;counter <200?
        cpfslt  ButCnt                      ;skip if so
	bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
		movlw   0xFB    
        movwf   TXFIFOIN7                   ;button code 0xFB - released within 4s
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
		call    TransmitButton
	ENDM
;------------
Button_4s_infin:MACRO ButNr,ButCnt,DiodeReg,DiodeBit ;200 < counter < infinity (released after 4s)
        banksel BUTCnfg                     ;turn on in config?
        btfss   BUTCnfg+ButNr-1,6
	bra $ + .32                             ;no - go to macro end
        movlw   .200                        ;counter >=200?
        cpfslt  ButCnt                      ;to send msg counter must be at least 200
        bra     $ + 4                       ;if not do not send msg
	bra $ + .24                             ;no - go to macro end
        banksel TXFIFOIN0
        movlw   ButNr                       ;set button number in msg to be sent
        movwf   TXFIFOIN6
		movlw   0xFA    
        movwf   TXFIFOIN7                   ;button code 0xFA - released after 4s
        setf    WREG                        ;diode state 0xFF - on
        btfss   DiodeReg,DiodeBit
        clrf    WREG	        	        ;0x00 - off
        movwf   TXFIFOIN8
		call    TransmitButton
	ENDM
;------------
TransmitButton
        banksel TXFIFOIN0
		movlw	0x30				    	;set frame type
		movwf	TXFIFOIN0
		movlw	0x10
		movwf	TXFIFOIN1
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
		setf	TXFIFOIN4				    ;unused
		setf	TXFIFOIN5			    	;unused
		;(TXFIFOIN6 -TXFIFOIN8) are already changed in macro
		setf	TXFIFOIN9				    ;unused
		setf	TXFIFOIN10			    	;unused
		setf	TXFIFOIN11				    ;unused
		call	WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
    return

;------------
ButtonON:MACRO ButNr,ButCnt,DiodeReg,DiodeBit       ;do all needed routines when button is pressed
        Button_IncCnt   ButCnt                                 ;increment button counter      /macro_arg: Button_COUNTER/
        Button_Pressed  ButNr,ButCnt,DiodeReg,DiodeBit         ;button pressed?               /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_400ms    ButNr,ButCnt,DiodeReg,DiodeBit         ;button held for 400ms?        /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_4s       ButNr,ButCnt,DiodeReg,DiodeBit         ;button held for 4s?           /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
	ENDM
;------------
ButtonOFF:MACRO ButNr,ButCnt,DiodeReg,DiodeBit       ;do all needed routines when button is released
        Button_Released ButNr,ButCnt,DiodeReg,DiodeBit         ;button released?              /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_60_400ms ButNr,ButCnt,DiodeReg,DiodeBit         ;button released within 400ms? /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        Button_400_4s   ButNr,ButCnt,DiodeReg,DiodeBit         ;button released within 4s?    /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
		Button_4s_infin ButNr,ButCnt,DiodeReg,DiodeBit         ;button released after 4s?     /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
        clrf	ButCnt                                         ;reset counter
	ENDM

;----------button 1----------
Button1_ON:
        btfsc   ButtonsA,0                          ;button on?
    return                                          ;no
        ButtonON   .1,BUT1Cnt,DiodesA,0              ;all routines for button ON /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
    return
;------------
Button1_OFF:
        btfss   ButtonsA,0                          ;button off?
    return                                          ;no
        ButtonOFF  .1,BUT1Cnt,DiodesA,0              ;all routines for button OFF /macro_arg: Button_No, Button_COUNTER, Diode_REGISTER, Diode_PIN/
    return
;----------button 2----------
Button2_ON:
        btfsc   ButtonsA,1                   
    return                                       
        ButtonON   .2,BUT2Cnt,DiodesA,1        
    return
;------------
Button2_OFF:
        btfss   ButtonsA,1                        
    return                                        
        ButtonOFF  .2,BUT2Cnt,DiodesA,1             
    return
;----------button 3----------
Button3_ON:
        btfsc   ButtonsA,2                     
    return                                     
        ButtonON   .3,BUT3Cnt,DiodesA,2            
    return
;------------
Button3_OFF:
        btfss   ButtonsA,2                         
    return                                       
        ButtonOFF  .3,BUT3Cnt,DiodesA,2        
    return
;----------button 4----------
Button4_ON:
        btfsc   ButtonsA,3    
    return
        ButtonON   .4,BUT4Cnt,DiodesA,3
    return
;------------
Button4_OFF:
        btfss   ButtonsA,3
    return
        ButtonOFF  .4,BUT4Cnt,DiodesA,3
    return
;----------button 5----------
Button5_ON:
        btfsc   ButtonsA,4    
    return
        ButtonON   .5,BUT5Cnt,DiodesA,4
    return
;------------
Button5_OFF:
        btfss   ButtonsA,4
    return
        ButtonOFF  .5,BUT5Cnt,DiodesA,4
    return
;----------button 6----------
Button6_ON:
        btfsc   ButtonsA,5    
    return
        ButtonON   .6,BUT6Cnt,DiodesA,5
    return
;------------
Button6_OFF:
        btfss   ButtonsA,5
    return
        ButtonOFF  .6,BUT6Cnt,DiodesA,5
    return
;----------button 7----------
Button7_ON:
        btfsc   ButtonsA,6    
    return
        ButtonON   .7,BUT7Cnt,DiodesA,6
    return
;------------
Button7_OFF:
        btfss   ButtonsA,6
    return
        ButtonOFF  .7,BUT7Cnt,DiodesA,6
    return
;----------button 8----------
Button8_ON:
        btfsc   ButtonsA,7    
    return
        ButtonON   .8,BUT8Cnt,DiodesA,7
    return
;------------
Button8_OFF:
        btfss   ButtonsA,7
    return
        ButtonOFF  .8,BUT8Cnt,DiodesA,7
    return
;----------button 9----------
Button9_ON:
        btfsc   ButtonsB,0    
    return
        ButtonON   .9,BUT9Cnt,DiodesB,0
    return
;------------
Button9_OFF:
        btfss   ButtonsB,0
    return
        ButtonOFF  .9,BUT9Cnt,DiodesB,0
    return
;----------button 10---------
Button10_ON:
        btfsc   ButtonsB,1    
    return
        ButtonON   .10,BUT10Cnt,DiodesB,1
    return
;------------
Button10_OFF:
        btfss   ButtonsB,1
    return
        ButtonOFF  .10,BUT10Cnt,DiodesB,1
    return
;----------button 11---------
Button11_ON:
        btfsc   ButtonsB,2    
    return
        ButtonON   .11,BUT11Cnt,DiodesB,2
    return
;------------
Button11_OFF:
        btfss   ButtonsB,2
    return
        ButtonOFF  .11,BUT11Cnt,DiodesB,2
    return
;----------button 12---------
Button12_ON:
        btfsc   ButtonsB,3    
    return
        ButtonON   .12,BUT12Cnt,DiodesB,3
    return
;------------
Button12_OFF:
        btfss   ButtonsB,3
    return
        ButtonOFF  .12,BUT12Cnt,DiodesB,3
    return
;----------button 13---------
Button13_ON:
        btfsc   ButtonsB,4    
    return
        ButtonON   .13,BUT13Cnt,DiodesB,4
    return
;------------
Button13_OFF:
        btfss   ButtonsB,4
    return
        ButtonOFF  .13,BUT13Cnt,DiodesB,4
    return

;==============================================================================
;                   TEMPERATURE PROCEDURES
;==============================================================================
;------------------------------------------------------------------------------
; Routine:			TEMPERATURE POWER UP VALUES
;------------------------------------------------------------------------------
; Overview:			Sets registers at power up 
;------------------------------------------------------------------------------
TempPowerUpValues                 
    ;config from eeprom
        ;thermostat value
        banksel THERMMSB
        movff   CONFIG13,THERMMSB           ;set switching temperature
        movff   CONFIG14,THERMLSB
        movlw   0x08                        ;if THERM = 0x0800 then take last saved
        xorwf   THERMMSB,W                  
		bnz     $ + .16
		movlw   0x00
		xorwf   THERMLSB,W                  
		bnz     $ + .10
		movff   CONFIG15,THERMMSB           ;last saved switching temperature
        movff   CONFIG16,THERMLSB
        ;histeresis
        banksel HISTEMSB
		movff	CONFIG17,HISTERE			;<7:2>.<1:0> (2 bits after decimal point)
		movff	CONFIG17,HISTELSB        
        clrf    HISTEMSB                  
        bcf     STATUS,C                    ;rotate left to match decimal point of DS temperature
        rlcf    HISTELSB 
        rlcf    HISTEMSB
        rlcf    HISTELSB
        rlcf    HISTEMSB                    ;<MSB><LSB> = <000000hh><hhhh.hh00> h-hysteresis from CONFIG, .- decimal point
        ;temperature offset
        movff   CONFIG18,OFFSETMSB          ;set temperature offset
        movff   CONFIG19,OFFSETLSB
        ;other values
        banksel TEMPLSB
        movlw   0x08                        ;previous temperature 0x0800
        movwf   STEMPLSB					
        clrf    STEMPMSB
        movlw   0x80                        ;thermostat status not set yet
        movwf   STHERMOS
        setf    CNVRTFLAG                   ;will force to do temp A/D convertion first
        clrf    COUNTER                     ;counter of 256s for periodical sending temp & therm msgs
    return
;------------------------------------------------------------------------------
; Routine:			TEMPERATURE A/D CONVERT 
;------------------------------------------------------------------------------
; Overview:			It does 1-wire initialization and if there is no error it
;                   sends "do temperature measurement" order to DS sensor
;------------------------------------------------------------------------------
CnvrtTemperature
        ;1-wire initialization
		call    DisAllInt				    ;disable interrupts
		call	InitiateDS				    ;initiate DS
		call    EnAllInt				    ;enable interrupts
		tstfsz	ERRORREG
	return                                  ;error on 1-wire bus indicated in ERRORREG
        ;convert request
		call    DisAllInt				    ;disable interrupts
		call	DSReset                     ;Send Reset Pulse and read for Presence Detect Pulse
		movlw   SKPROM					    ;Send Skip ROM Command (0xCC)
		call    DSTXByte                    
		movlw   0x44                 	    ;Send Convert T (0x44)
		call    DSTXByte
		call    EnAllInt				    ;enable interrupts
        ;strong  pull up on
        bsf     PORTDQ,DQ                   ;high level on DS
        bcf     TRISDQ,DQ                   ;pin as output
        ;flag
		movlw	.40					    	;wait 40x20ms=800ms to convert
		movwf	CNVRTFLAG				    ;conversion in progress flag
	return
;----------------------------
InitiateDS:
		;presence
		call	DSReset                     ;Send Reset Pulse and read for Presence Detect Pulse
		btfsc   PDBYTE,0                    ;1 = Presence Detect Detected
		bra		CheckDSROM
		call    NoDevice
	    bra		ExitInitiateDS
CheckDSROM                                  ;checks if there is only one device on network
		call	CheckROM
		tstfsz	WREG
		bra		$ + 4
		bra		ReadDSROM
		call	TooManyDevices
	    bra		ExitInitiateDS
ReadDSROM                                   ;check CRC
		call	DSReset
		call	ReadROM
		tstfsz	WREG					    ;returned 1 means wrong CRC
		bra		$ +4
		bra		CheckDS
		call	WrongCRC
	    bra		ExitInitiateDS		
CheckDS
		movlw	0x22					    ;DS1822
		xorwf	ROMCODE0,W
		bz		NoDSError
		movlw	0x28					    ;DS18B20
		xorwf	ROMCODE0,W
		bz		NoDSError
		call	WrongDevices
    	bra		ExitInitiateDS
NoDSError
		clrf	ERRORREG				    ;no error
ExitInitiateDS
	return

;------------------------------------------------------------------------------
; Routine:			READ TEMPERATURE
;------------------------------------------------------------------------------
; Overview:			It reads temperature value from DS
;------------------------------------------------------------------------------
ReadTemperature
        tstfsz  CNVRTFLAG                   ;conversion in progress if flag!=0
        bra     ExitReadTemperature
        ;request and read temperature
        call    DisAllInt                   ;disable interrupts
        rcall   GetTemp                     ;get temp
		call    EnAllInt                    ;enable interrupts
        tstfsz  ERRORREG
	return 
        setf    CNVRTFLAG				    ;make flag =FF to not repeat reading before next convertion
        rcall   AddTempOffset               ;add temperature offset
        rcall   CompareTherm                ;compare thermostat value to temperature and send msg if needed
        rcall   CompareTemp                 ;compare temperature to previous one and send msg if needed
        rcall   SendTempPeriodically        ;send temp & themostat every 256s
ExitReadTemperature
	return

;----------------------------
GetTemp
		;temp request
		call    DSReset              	    ;Send Reset Pulse and read for Presence Detect Pulse      
		movlw   SKPROM                      ;Send Skip ROM Command (0xCC)
		call    DSTXByte
		movlw   0xBE                        ;ReadScratchpap (0xBE)
		call    DSTXByte
		;read temp
		clrf	CRCREG					    ;init CRC
		call    DSRXByte           		    ;read LSB
		call	CalcCrc
		movf    IOBYTE,W
		movwf   TEMPLSB                 
		call    DSRXByte                    ;read MSB
		call	CalcCrc
		movf    IOBYTE,W
		movwf   TEMPMSB
		call    DSRXByte           		    ;read TH
		call	CalcCrc
		call    DSRXByte           		    ;read TL
		call	CalcCrc
		call    DSRXByte           	        ;read CFG
		call	CalcCrc
		call    DSRXByte           		    ;read RES
		call	CalcCrc
		call    DSRXByte           		    ;read RES
		call	CalcCrc
		call    DSRXByte           		    ;read RES
		call	CalcCrc
		call    DSRXByte           	    	;read CRC
		call	CalcCrc
		tstfsz	CRCREG					    ;test if CRC correct
		bra		GotTempIsBad
		clrf	ERRORREG				    ;no error
	return                                  ;reading ok
GotTempIsBad
		call	WrongCRC
	return                                  ;error when reading DS

;----------------------------
SendTempPeriodically 
        decfsz  COUNTER                     ;count 256s from last sending
    return
        rcall   TxTemp                      ;send temperature & thermostat
    return

;----------------------------
AddTempOffset                               ;add temperature offset to current temperature
        movf    OFFSETLSB,W
        addwf   TEMPLSB,F
        movf    OFFSETMSB,W
        addwfc  TEMPMSB,F
    return

;----------------------------
CompareTherm							    ;compare temp to thermostat and send msg if needed
        banksel TEMPLSB
        ;check difference between current temperature and thermostat
        movf    TEMPLSB,W                   ;DIFF(TTEMP) = THERM - TEMP (difference = thermostat - current)     
        subwf   THERMLSB,W
        movwf   TTEMPLSB
        movf    TEMPMSB,W
        subwfb  THERMMSB,W
        movwf   TTEMPMSB
        ;result negative?
        bnn     $ + .10                     
        comf    TTEMPLSB                    ;yes, so change to positive
        comf    TTEMPMSB
        infsnz  TTEMPLSB
        incf    TTEMPMSB
        ;compare the difference with histeresis
        movf    HISTELSB,W                  ;TTEMP = DIFF(TTEMP) - HISTE (temporary temperature = difference - histeresis) 
        subwf   TTEMPLSB,F 
        movf    HISTEMSB,W
        subwfb  TTEMPMSB,F
    bn      ExitCompareTherm                ;current temp inside histeresis, so exit
        ;temp above or below thermostat
        movf    TEMPLSB,W                   ;THERM - TEMP
        subwf   THERMLSB,W
        movf    TEMPMSB,W
        subwfb  THERMMSB,W
		bnn		TxTLTherm                   ;TEMP < THERM, so send TL
		bn		TxTHTherm                   ;TEMP > THERM, so send TH
TxTLTherm:
		movlw	0x00	    				;check if TL was sent
		xorwf	STHERMOS,W
		bz		ExitCompareTherm			;yes, so exit
		clrf	STHERMOS
		bra		TxTherm
TxTHTherm:
		movlw	0xFF					    ;check if TH was sent
		xorwf	STHERMOS,W
		bz		ExitCompareTherm			;yes, so exit
		setf	STHERMOS
		bra		TxTherm
TxTherm:
        banksel TXFIFOIN6
		movlw	0x12					    ;load data to transmit buffer
		movwf	TXFIFOIN6					;0x12 - "thermostat 1"
        movff   STHERMOS,TXFIFOIN7          ;thermostat state
		setf	TXFIFOIN8
		setf    TXFIFOIN9			    	;unused
		setf    TXFIFOIN10			    	;unused
		setf    TXFIFOIN11			    	;unused
		call	TransmitTemp
ExitCompareTherm
	return

;----------------------------
CompareTemp                                 ;compare temp to previous one and send msg if different
        banksel TEMPLSB
        ;TTEMP = STEMP - TEMP (temporary temperature = previous - current)
        movf    TEMPLSB,W      
        subwf   STEMPLSB,W
        movwf   TTEMPLSB
        movf    TEMPMSB,W
        subwfb  STEMPMSB,W
        movwf   TTEMPMSB
        ;result negative?
        bnn     $ + .10                     
        comf    TTEMPLSB                    ;yes, so change to positive
        comf    TTEMPMSB
        infsnz  TTEMPLSB
        incf    TTEMPMSB
        ;check if diference between current and previous temperature is greater or equal 0.5deg
        tstfsz  TTEMPMSB,W
    	bra		TxTemp                      ;temp difference greater than 16deg, so send msg        
        movlw   0x08                        ;difference greater or equal than 0.5deg?
        cpfslt  TTEMPLSB
	    bra		TxTemp                      ;yes, so send msg        
	bra		ExitCompareTemp

TxTemp
        banksel TXFIFOIN0
		movlw	0x11					    ;load data to transmit buffer
		movwf	TXFIFOIN6					;0x11 - "temperature 1"
        movff   TEMPMSB,TXFIFOIN7
        movff   TEMPLSB,TXFIFOIN8
        movff   THERMMSB,TXFIFOIN9          ;set thermostat temp
        movff   THERMLSB,TXFIFOIN10
        movff   HISTERE,TXFIFOIN11          ;hysteresis
		call	TransmitTemp
		movff	TEMPMSB,STEMPMSB		    ;save temperature MSB as previous value
		movff	TEMPLSB,STEMPLSB		    ;save temperature LSB as previous value
        rcall   TxTherm                     ;send thermostat frame as well
        clrf    COUNTER                     ;start counting 256s again
ExitCompareTemp
	return

;----------------------------
TransmitTemp
        banksel TXFIFOIN0
		movlw	0x30					    ;set frame type
		movwf	TXFIFOIN0
		movlw	0x40
		movwf	TXFIFOIN1
		movff	NODENR,TXFIFOIN2            ;node id
		movff	GROUPNR,TXFIFOIN3
		setf	TXFIFOIN4
		setf	TXFIFOIN5
		;(TXFIFOIN6 -TXFIFOIN11) are already changed parent procedure
		call	WriteToCanTxFIFO
    ;node can respond to its own message
        bcf     INTCON,GIEL                 ;disable low priority intr to make sure RXFIFO buffer is not overwritten
        call    Copy_TXFIFOIN_RXFIFOIN
        call    WriteToCanRxFIFO
        bsf     INTCON,GIEL                 ;enable back interrupt
	return

;------------------------------------------------------------------------------
; Routine:			1-wire ERRORS
;------------------------------------------------------------------------------
; Overview:			It sets error type
;------------------------------------------------------------------------------
NoDevice:                  	                ;no device on bus
 		movlw	0x01
		movwf	ERRORREG
	return
;------------
TooManyDevices:             	            ;too many devices on bus or wrong device (without 64bit rom)
 		movlw	0x02					
		movwf	ERRORREG
	return
;------------
WrongDevices:             	            	;wrong device
 		movlw	0x03
		movwf	ERRORREG
	return
;------------
WrongCRC:									;CRC problem
 		movlw	0x04
		movwf	ERRORREG
	return

;------------------------------------------------------------------------------
; Routine:			SAVE THERMOSTAT TO EEPROM
;------------------------------------------------------------------------------
; Overview:			It saves current thermostat value into EEPROM memory
;------------------------------------------------------------------------------
SaveSateToEeprom			
        banksel EEPROMTIMER
		;wait 6s before saving
		tstfsz	EEPROMTIMER
		bra		$ + 4
		bra		ExitSaveStateToEeprom
		decfsz	EEPROMTIMER
		bra		ExitSaveStateToEeprom
		;save to eeprom
		clrf    EEADRH				        ;point at high address
		movlw	CONFIG15				    ;point at low address	
		movwf	EEADR
		movf	THERMMSB,W  		        ;set data
        call    EepromSaveWREG
        incf    EEADR                       ;point at next address
		movf	THERMLSB,W  		        ;set data
        call    EepromSaveWREG
ExitSaveStateToEeprom
	return

;==============================================================================
;===  END  OF  PROGRAM  =======================================================
;==============================================================================
    END