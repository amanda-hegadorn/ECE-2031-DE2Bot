
ORG        &H000       
	JUMP   Init


;***************************************************************
;* Initialization
;***************************************************************
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	STORE  RVEL
	STORE  LVEL ;initialize to 0
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	;OUT    BEEP        ; Stop any beeping (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

WaitForSafety:
	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
	
WaitForUser:
	; This loop will wait for the user to press PB3, to ensure that
	; they have a chance to prepare for any movement in the main code.
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
;************************************************************************
Main: ; "Real" program starts here.
	OUT    RESETPOS    ; reset odometry in case wheels moved after programming	
	;TO DO: Add capability to get Nth x and y entries and set them as DesX and DesY
	;afterwards, increment N to get next one on next cycle

;This motion control algorithm is based off of tank drive controls using a joystick
;The desired coordinate acts as control input, and the wheel velocities adjust according to the maximum and minimum velocities, scaled to the maximum and minimum distance required
;In main, the robot goes through a loop where the dTheta, dX, and dY are measured and then velocities are adjusted according to this tank drive model:
;Max Dist: 10 ft x, 12 ft y
;Max coordinates, 8 ft x, 10 ft y
;2322 Robot Coordinates X, from +1161 to -1161
;2902 Robot Coordinates Y, from +1451 to -1451
; 
;V=R+L=(MaxDist-ABS(dX))*(dY/MaxDist)+dY  
;W=R-L=(MaxDist-ABS(dy))*(dX/MaxDist)+dX
;R=V+W/2
;L=V-W/2
;R and L are in robot units, scale to 200-456 for speeds

Process: 

	IN XPOS;read IN X, IN Y
	STORE currX
	OUT SSEG1

	IN YPOS
	STORE currY
	OUT SSEG2


	;calc dX
	
	LOAD  currX
	SUB   DesX
	STORE dX
	;OUT LCD
	
	;calc dY
	;LOAD DesY
	;SUB  currY
	LOAD currY
	SUB  DesY
	STORE dY
	;after storing dX and dY, check if dY error is smaller than deadband
	CALL  Abs
	SUB   DeadBand
	JNEG  CheckDeadBand

	CALL RPL
	CALL RML

	;calculate R and L
	LOAD V
	ADD  W
	SHIFT -1
	STORE R

	LOAD V
	SUB  W 
	SHIFT -1
	STORE L

	CALL Scale
	LOAD R
	OUT  RVELCMD
	LOAD L
	OUT  LCD
	OUT LVELCMD
	CALL WAIT1

	JUMP Process
	

CheckDeadBand:
	Load dY
	CALL Abs
	SUB  DeadBand
	;JNEG Main
	JNEG Die
	JUMP Process


	;set motor speeds
	;break when DesX and DesY are less than 15? (good deadband?)
	;increment n and go back to main

;*******************
;SIMPLE PROCEDURES
;*******************
RPL: ;R Plus L Calculation. Values must be stored in dX and dY. Result in V
	LOAD dX
	CALL Abs
	STORE temp
	LOAD MaxDistX
	SUB  temp
	STORE m16sA ;multA= (total-absd(x))

	LOAD  dY 
	STORE d16sN
	LOAD  MaxDistY
	STORE d16sD
	CALL  Div16s ;amount of X to add stored in dres16Q, ignoring remainder. 
	LOAD  dres16sQ
	STORE m16sB ;multB= (dY/MaxDistY)

	CALL  Mult16s
	LOAD  mres16sL ;only the low value matters beecause 1451*% will be less than 16 bits

	ADD   dY
	STORE V
	RETURN

RML: ; R Minus L Calculation.  Values must be stored in dX and dY. Result in W
	LOAD dY
	CALL Abs
	STORE temp
	LOAD MaxDistY
	SUB  temp
	STORE m16sA ;multA= (total-abs(dx))

	LOAD  dX
	XOR   &HFFFF
	ADDI  1
	STORE d16sN
	LOAD  MaxDistX
	STORE d16sD
	CALL  Div16s ;amount of X to add stored in dres16Q, ignoring remainder. 
	LOAD  dres16sQ
	STORE m16sB ;multB= (dY/MaxDistY)

	CALL  Mult16s
	LOAD  mres16sL ;only the low value matters beecause 1451*% will be less than 16 bits

	ADD   dX
	STORE W
	RETURN 

Scale: ;scales the distance values in RVEL and LVEL to a speed from 200 to 512
	LOAD R
	SHIFT -9 ;divide by approximately 512
	JPOS ADDR
	ADDI  -250
	STORE R
	JUMP  SCALEL
ADDR:
	ADDI 250
	STORE R

SCALEL:
	LOAD L
	SHIFT -9 ;divide by approximately 512
	JPOS ADDL
	ADDI  -250
	STORE R
	RETURN
ADDL:
	ADDI 250
	STORE L

	RETURN

	
Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	;LOAD   DEAD         ; An indication that we are dead
	IN     XPOS
	OUT    SSEG1
	IN     YPOS
	OUT    SSEG2
	;OUT    BEEP
	CALL   Wait1
	LOAD   ZERO
	OUT    BEEP
Forever:
	JUMP   Forever      ; Do this forever.
	DEAD:  DW &HDEAD    ; Example of a "local" variable



;*************************************************
;MOTION CONTROL VARIABLES
;*************************************************
currX: dw 0 ;initialize current pos to 0
currY: dw 0 ; initialize current pos to 0
dX: dw 0
dY: dw 0
;This will be stored in a table somewhere after this, with ability to increment
DesX: dw &H500
DesY: dw &HF005
;************
N: dw 0 ;This is the coordinate counter

V: dw 0 ;R+L
W: dw 0; R-L

R: dw 0
L: dw 0

DeadBand: dw 50
MaxDistX: dw 1161 
MaxDistY: dw 1451
MinVel: dw 250
MaxVel: dw 500

Temp: dw 0 ;for random math



    ;***************************************************************
;* Subroutines
;***************************************************************

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN

; Subroutine to wait the number of counts currently in AC
WaitAC:
	STORE  WaitTime
	OUT    Timer
WACLoop:
	IN     Timer
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	SUB    WaitTime
	JNEG   WACLoop
	RETURN
	WaitTime: DW 0     ; "local" variable.

; Converts an angle to [0,359]
Mod360:
	JNEG   M360N       ; loop exit condition
	ADDI   -360        ; start removing 360 at a time
	JUMP   Mod360      ; keep going until negative
M360N:
	ADDI   360         ; get back to positive
	JNEG   M360N       ; (keep adding 360 until non-negative)
	RETURN
	
; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError



;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOAD  Zero
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOAD  Sixteen       ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ; add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ; subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOAD  Zero
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOAD   Seventeen
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOAD   One
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOAD   Zero
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	XOR    NegOne
	ADDI   1
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10
Sixteen:  DW 16
seventeen: DW 17


; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
TwoFeet:  DW 586       ; ~2ft in 1.04mm units
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500
FMidSq:   DW 122500 ; the square of 350 for calculations
StopDist: DW 119    ; the approximate distance in robot units it takes to stop the robot at medium speed

MinBatt:  DW 70       ; 14.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
