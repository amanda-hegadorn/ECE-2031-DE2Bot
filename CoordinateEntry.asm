; Coordinate Entry Tests

ORG 0  ; Begin program at x000
LOAD DesX ; load table
STORE XPtr ; store in X table
LOAD DesY ; load table
STORE YPtr ; store in X table
LOAD Zero ; count starts at 0
STORE Count
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
   	 	ISTORE XPtr  
        IN TIMER 
    	AND Mask1 ; blink LEDs
    	SHIFT 4 ;  LED 5
    	OUT XLEDS ;send to LEDs
    	IN XIO ; contains KEYs
    	AND Mask1 ;key2 mask 
    	JPOS WaitForX ; not ready (KEYs are active-low, hence JPOS)
    LOAD XPtr 
    ADD One
    STORE XPtr ;increment xptr 
    LOAD Zero 
   	OUT XLEDS ;stop when ready to continue 
    JUMP WaitForY

    WaitForY: 
    	IN SWITCHES ; input the number into the switches for x
    	OUT SSEG2 ; output to 2nd 7seg
   	 	ISTORE YPtr  
   	 	IN TIMER 
    	AND Mask1 ; blink LEDs
    	SHIFT 2 ;  LED 3
    	OUT XLEDS ;send to LEDs
    	IN XIO ; contains KEYs
    	AND Mask0 ;key1 mask 
    	JPOS WaitForY ; not ready (KEYs are active-low, hence JPOS)
    LOAD YPtr 
    ADD One
    STORE YPtr ;increment Yptr 
    LOAD Zero 
   	OUT XLEDS ;stop when ready to continue 
   	
    LOAD Count 
    SUB Twelve ;subtract 12 from count 
    JNEG CoordinateEntry ; jump back to top of loop if haven't entered 12 destinations
	JUMP Check
    
Check:	
	LOAD Zero
	STORE Count
	OUT LCD
	JUMP Loop
	
Loop:
	LOAD Count
	OUT LCD
;load values from table and print
	LOAD DesX
	ADD  Count ;increment x pointer (head of table) with count
	STORE XPtr
	ILOAD XPtr
	OUT SSEG1
	
	LOAD DesY
	ADD  Count
	STORE YPtr
	ILOAD YPtr
	OUT SSEG2
;wait and increment pointers
	CALL WAIT1
	LOAD Count
	ADDI 1
	STORE Count
	SUB Twelve
	JNEG Loop
	JUMP InfLoop

InfLoop: ;inf loop to stop program
	LOADI &HD0BE
	OUT   LCD
	JUMP InfLoop
	
	
	




; Subroutine to wait (block) for 1 second
WAIT1:
	STORE AC
	OUT    TIMER
	JUMP Wloop
Wloop:
	IN     TIMER
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	LOAD   AC
	RETURN
	
	
AC: DW 0
;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 0 ; "Temp" is not a great name, but can be useful

; Coordinate Entry variables
DesX: DW &H0500
DesY: DW &H050C
XPtr: DW 0
YPtr: DW 12
Count: DW 0


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
Seventeen: DW 17

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
