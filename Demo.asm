
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
	STORE  Count
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	;OUT    BEEP        ; Stop any beeping (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display battery voltage (hex, tenths of volts)

	LOAD XHead ; load table
	STORE pX ; store in X table
	LOAD YHead ; load table
	STORE pY ; store in X table
	;LOAD Zero ; count starts at 0
	;STORE Count

	;IN XIO
	;AND Mask3
	;JPOS WaitForSafety
	JUMP CoordinateEntry

CoordinateEntry: 
   ; This loop will allow the user to input the 12 (x,y) coordinates
   ; to the DE2bot using the switches and PB1.
   ; Use switches to enter the coordinates in binary and display on the 7 seg
   ; PB2 will serve as an enter key 
   ; make a counter to track destinations 
    LOAD Count 
    ADD One ;increment count
    STORE Count
	OUT LCD ; output destination number to LCD
    LOAD Zero
    OUT SSEG1
    LOAD Zero
    OUT SSEG2
    JUMP WaitForX
     
    
    ;blink the LEDS above PB2 so the user knows to press it as an enter key 
    WaitForX: ;waits for user to press PB2
    	IN SWITCHES ; input the number into the switches for x
    	OUT SSEG1 ; output to 1st 7seg
   	 	ISTORE pX  
        IN TIMER 
    	AND Mask1 ; blink LEDs
    	SHIFT 4 ;  LED 5
    	OUT XLEDS ;send to LEDs
    	IN XIO ; contains KEYs
    	AND Mask1 ;key2 mask 
    	JPOS WaitForX ; not ready (KEYs are active-low, hence JPOS)
    LOAD pX 
    ADD One
    STORE pX ;increment pX 
    LOAD Zero 
   	OUT XLEDS ;stop when ready to continue 
    JUMP WaitForY

    WaitForY: 
    	IN SWITCHES ; input the number into the switches for x
    	OUT SSEG2 ; output to 2nd 7seg
   	 	ISTORE pY  
   	 	IN TIMER 
    	AND Mask1 ; blink LEDs
    	SHIFT 2 ;  LED 3
    	OUT XLEDS ;send to LEDs
    	IN XIO ; contains KEYs
    	AND Mask0 ;key1 mask 
    	JPOS WaitForY ; not ready (KEYs are active-low, hence JPOS)
    LOAD pY 
    ADD One
    STORE pY ;increment pY 
    LOAD Zero 
   	OUT XLEDS ;stop when ready to continue 
   	
    LOAD Count 
	ADDI -12
    ;SUB Twelve ;subtract 12 from count 
    JNEG CoordinateEntry ; jump back to top of loop if haven't entered 12 destinations

WaitForSafety:
	

	; This loop will wait for the user to toggle SW17.  Note that
	; SCOMP does not have direct access to SW17; it only has access
	; to the SAFETY signal contained in XIO.
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If readeltaY, jump to wait for PB3
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
	JPOS   WaitForUser ; not readeltaY (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once readeltaY to continue
;************************************************************************
Main: ; "Real" program starts here.
	OUT    RESETPOS    ; reset odometry in case wheels moved after programming

	LOADI &H0500
	STORE pX
	LOADI &H050C
	STORE pY

	LOAD  Zero
	STORE N

	;TO DO: Add capability to get Nth x and y entries and set them as DesX and DesY
Next:
	LOAD N 
	ADDI -12
	;ADDI -12 ;we are only doing four points right now to test motion, because I don't want to wait for them all.
	JZERO Die

	ILOAD pX
	STORE DesX
	
	ILOAD pY
	STORE DesY
	
	LOAD pX 
	ADD  One
	STORE pX

	LOAD pY 
	ADD  One
	STORE pY
	
	LOAD  N
	ADDI  1
	STORE N
	OUT LCD
	;afterwards, increment N to get next one on next cycle
	;-1161 to 1161 x dir
	;1451 y

Process: 

	IN XPOS;read IN X, IN Y
	STORE CurrX
	OUT SSEG1
	;CALL WAIT1

	IN YPOS
	STORE CurrY
	OUT SSEG2
	;CALL WAIT1

	;calc deltaX
	LOAD  DesX
	SUB   CurrX
	STORE deltaX
	STORE AtanX
	;OUT SSEG1
	;CALL WAIT1
	
	;calc deltaY

	LOAD DesY
	SUB  CurrY
	STORE deltaY
	STORE AtanY
	LOAD deltaY
	;after storing deltaX and deltaY, check if deltaY error is smaller than deadband
	CALL  Abs
	SUB   DeadBand
	JPOS  Cont ;if it is not 0 or negative, continue
	JUMP  CheckDeadBand ; if it is 0 or negative, jump to check x

	;Take the tangent and determine what quadrant the angle error is in
Cont:
	CALL ATan2 ;call on the current deltaX and deltaY stored in atanx and atany
	STORE DesTheta
	IN    THETA
	STORE CurrTheta
	LOAD  DesTheta
	SUB   CurrTheta
	CALL  Mod360
	STORE dTheta
	;OUT  LCD
	;CALL WAIT1

	;Set R or L depending on quadrant
Q1:	LOAD dTheta
	SUB Deg90
	JPOS Q2 ;jump to Quadrant 2 test
	;Q1
	;LOADI 1
	;OUT  LCD
	;CALL WAIT1
	LOAD FullSpeed
	STORE R ;R=250
	CALL  CALCL
	STORE L ;result from calcLR in AC and in ResLR
	JUMP Set

Q2:
	;check if Q2
	LOAD dTheta
	SUB  Deg180
	JPOS Q3
	;Q2
	;LOADI 2
	;OUT  LCD
	;CALL WAIT1
	LOAD ZERO
	SUB FullSpeed
	;LOAD FullSpeed
	STORE R
	CALL CALCL
	STORE L
	JUMP Set

Q3:
	;since we know it is either in Q3 or Q4, subtract from 180 to make negative
	LOAD Deg360
	SUB  dTheta
	STORE dTheta
	;check if Q3
	ADD  Deg90
	JPOS Q4

	;LOADI 3
	;OUT LCD
	;CALL WAIT1
	LOAD Zero
	SUB FullSpeed
	;XOR  Negone
	;ADDI 1
	STORE L
	CALL  CALCR
	STORE R ;result from calcLR in AC and in ResLR
	JUMP Set

Q4:
	;LOADI 4
	;OUT  LCD
	;CALL WAIT1
	LOAD FullSpeed
	STORE L
	CALL CALCR
	STORE R

	;set speeds
Set:	
	LOAD R
	OUT  RVELCMD
	;OUT  SSEG2
	LOAD L
	OUT LVELCMD
	;OUT SSEG1

	;LOADI 10
	;CALL WAITAC
	JUMP Process
	

CheckDeadBand:
	Load deltaX
	CALL Abs
	SUB  DeadBand
	JPOS Cont
	JUMP LIGHT

LIGHT: 
	;do lights
	;if we have N=3, light 3 lights. that means send &B..0111
	;LOAD  N
	;OUT XLEDS
	LOAD Four
	OUT  BEEP
	LOAD Two
	CALL WAITAC
	LOAD ZERO
	OUT  BEEP
	JUMP Next


;*******************
;SIMPLE PROCEDURES
;*******************
	
Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   ZERO         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD         ; An indication that we are dead
	OUT		LCD
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


;calculate the side of the car that is not full speed, store result in ResLR
CALCL:
	;(R+L)/2=(100-ABS(dtheta*100/90))-- in percent
	;(RL+FullSpeed)/2=(FullSpeed-ABS(dtheta*FullSpeed/90)) -- in speeds, because one (either R or L) will always be full speed
	;dtheta*FullSpeed
	LOAD FullSpeed 
	STORE m16sA
	LOAD  dTheta
	STORE m16sB
	CALL  Mult16s

	; divide by 90
	LOAD mres16sL ;only need low, bounds mean it will never be higher than max speed
	STORE d16sN
	LOAD Deg90
	STORE d16sD
	CALL  Div16s

	;ABS
	LOAD dres16sQ
	CALL ABS ; take ABS of AC
	STORE temp
	;subtract from full speed
	LOAD FullSpeed
	SUB  temp

	;multiply by 2
	SHIFT 1
	;OUT LCD
	;CALL WAIT1 ;should be 250
	;subtract full speed ;TO DO: I think this is redundant, and just another way to switch the sign. once operational, simplify algebra
	SUB FullSpeed
	STORE L
	;OUT LCD
	;CALL WAIT1 ;should be 250
	;OUT   SSEG2
	RETURN 

CALCR:
	;(R+L)/2=(100-ABS(dtheta*100/90))-- in percent
	;(RL+FullSpeed)/2=(FullSpeed-ABS(dtheta*FullSpeed/90)) -- in speeds, because one (either R or L) will always be full speed
	;dtheta*FullSpeed
	LOAD FullSpeed
	STORE m16sA
	LOAD  dTheta
	STORE m16sB
	CALL  Mult16s

	; divide by 90
	LOAD mres16sL ;only need low, bounds mean it will never be higher than max speed
	STORE d16sN
	LOAD Deg90
	STORE d16sD
	CALL  Div16s

	;ABS
	LOAD dres16sQ
	CALL ABS ; take ABS of AC
	STORE temp
	;subtract from full speed
	LOAD FullSpeed
	SUB  temp

	;multiply by 2
	SHIFT 1
	;subtract full speed ;TO DO: I think this is redundant, and just another way to switch the sign. once operational, simplify algebra
	SUB L
	STORE R
	;OUT   SSEG2
	RETURN 
;*************************************************
;MOTION CONTROL VARIABLES
;*************************************************
DesX:  dw 0
DesY:  dw 0

CurrX: dw 0 ;initialize current pos to 0
CurrY: dw 0 ; initialize current pos to 0
CurrTheta: dw 0
deltaX: dw 0
deltaY: dw 0
;This will be stored in a table somewhere after this, with ability to increment

dTheta: dw 0
DesTheta: DW 0
;************
N: dw 0 ;This is the coordinate counter
pX: DW 0
pY: DW 0
XHead: DW &H0500
YHead: DW &H050C
Count: DW 0 ;the variable for going through the coordinate entry

R: dw 0
L: dw 0

DeadBand: dw 30
MaxDistX: dw 1161 
MaxDistY: dw 1451
;MinVel: dw 450
FullSpeed: dw 375

Temp: dw 0 ;for random math
ResLR: dw 0

LEDMask: DW 0;


    ;***************************************************************
;* Subroutines
;***************************************************************



; Subroutine to wait (block) for 1 second
Wait1:
	STORE AC
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
	LOAD AC
	RETURN
	WaitTime: DW 0     ; "local" variable.
	AC: DW 0 ;stores current AC and restores

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
	OUT    I2C_RdeltaY     ; start the communication
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
	OUT    I2C_RdeltaY     ; start the communication
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
	IN     I2C_RdeltaY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store deltaX and deltaY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW -1                                                             ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	XOR    NegOne
	ADDI   1            ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOAD  Nine            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits



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
Twelve:   DW 12
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
I2C_RdeltaY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RdeltaY: EQU &H98  ; UART status
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


ORG		   &H0500
	X1: DW &H0489
	X2: DW &H0489
	X3: DW &HFB77
	X4: DW &HFB77
	X5: DW &H015c
	X6: DW &H0000
	X7: DW &HFF30
	X8: DW &HFC77
	X9: DW 0
	X10: DW &H015c
	X11: DW 0
	X12: DW &HFB77

ORG        &H050C
	Y1: DW &HFA55
	Y2: DW &H05AB
	Y3: DW &HFA55
	Y4: DW &H05AB
	Y5: DW 0
	Y6: DW &H015c
	Y7: DW 0
	Y8: DW &H015c
	Y9: DW 0
	Y10: DW &H015c
	Y11: DW 0
	Y12: DW &HFB77