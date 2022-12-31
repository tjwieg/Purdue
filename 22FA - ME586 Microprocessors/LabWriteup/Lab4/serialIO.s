; Harsh Savla & TJ Wiegman
; ME 58600
; 2022-09-19
; serialIO.s

RCC_APB2ENR EQU 0x40021018 ; enable APB2 clock for USART1
GPIOA_CRH   EQU 0x40010804 ; configure PA09 and PA10 for tx/rx
USART1_BRR  EQU 0x40013808 ; configure USART1 baud rate
USART1_CR1  EQU 0x4001380C ; enable USART1, set parity, mode
USART1_SR   EQU 0x40013800 ; USART1 satus register
USART1_DR   EQU 0x40013804 ; USART1 data register

;USART1en    EQU 2_0100000000000101
USART1baud  EQU 0x00D0 ; hex fraction for setting baud rate
;USART1ctrl  EQU 2_0010000000001100

; allocate some RAM for bindec
        AREA MyData, DATA, READWRITE
num3    SPACE 2
array3  SPACE 6

; program code
        AREA ARMex, CODE, READONLY
        ENTRY
initcom     PROC    ; initializes serial channel 1 for asynchronous communcations
        EXPORT initcom
        ; push LR to stack
        push {LR}

        ; turn on APB2 perhipheral clock
        ldr R3, =RCC_APB2ENR
        ldr R1, [R3] ; save current APB2 state
        orr R1, #0x4000
		orr R1, #0x0005
        str R1, [R3]

        ; configure port A - PA09 output, PA10 input
        ldr R3, =GPIOA_CRH
        ldr R1, =0x444444B4
        str R1, [R3]

        ; configure USART1 baud rate
        ldr R3, =USART1_BRR
        mov R1, #USART1baud ; as close as possible to 115,200 -- impossible to get exact at 24MHz
        str R1, [R3]

        ; enable USART1 for 8 data bits. disable parity and interrupts
        ldr R3, =USART1_CR1
        mov R1, #0x200C
        str R1, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP

checkcom    PROC    ; checks to see if a character is in the data receive register. writes 0xFF to R0 if availabe, 0x00 otherwise
        EXPORT checkcom 
        ; push LR to stack
        push {LR}

        ; check status register
        ldr R3, =USART1_SR
        ldr R1, [R3]
        and R1, #32 ; mask out unneeded flags
        cmp R1, #32 ; check if RXNE flag is set
        beq ready

        ; set R0 to 0x00 if not ready
        mov R0, #0x00
        b chEnd

        ; set R0 to 0xFF if ready
ready   mov R0, #0xFF

        ; End subroutine and go back to caller
chEnd   pop {LR}
        bx LR
        ENDP

getchar     PROC    ; fetches character from serial channel 1 and writes it as ASCII to R0
        EXPORT getchar
        ; push LR to stack
        push {LR}

        ldr R3, =USART1_DR
        ldrb R0, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP

showchar    PROC    ; checks that TXE is set, then outputs ASCII character in R0 to serial channel 1
        EXPORT showchar 
        ; push LR to stack
        push {LR}

        ; check status register
shWait	ldr R3, =USART1_SR
        ldr R1, [R3]
        and R1, #128 ; mask out unneeded flags
        cmp R1, #128 ; check if TXE flag is set
        beq write
        
        ; if DR is not ready yet
        b shWait
        
        ; if DR is ready
write   ldr R3, =USART1_DR
        strb R0, [R3]

        ; End subroutine and go back to caller
		pop {LR}
        bx LR
        ENDP

bindec      PROC ; converts a 16-bit signed binary number into five decimal characters (digits)
; preceded by either a space or a minus sign depending on whether the signed number is positive or negative
        EXPORT bindec
        ; push LR to stack
        push {LR}

        ; fill array with spaces
        mov R1, #1
        ldr R3, =array3
        mov R0, #0x20   ; ascii character for " "
clrloop strb R0, [R3], #1
        add R1, #1
        cmp R1, #7
        bne clrloop

        ; get input number from RAM, put into R0
        ldr R3, =num3
        ldrh R0, [R3]

        mov R1, R0
        lsr R1, #15 ; shift first digit down to LSB
        cmp R1, #1  ; is negative?
        beq neg1

        ; is positive
        mov R1, #0x20 ; ascii character for " "
        ldr R3, =array3
        strb R1, [R3], #5
        b binloop

        ; is negative
neg1    mov R1, #0x2D ; ascii character for "-"
        ldr R3, =array3
        strb R1, [R3], #5
        sxth R0
        sub R0, #1
        eor R0, #0xFFFFFFFF

        ; divide by 10
binloop	mov R4, #10
		udiv R1, R0, R4  ; R1 holds quotient
        mul R2, R1, R4
        sub R2, R0, R2      ; R2 holds remainder

        ; convert to ASCII and store
        add R2, #0x30
        strb R2, [R3], #-1
        mov R0, R1
        cmp R0, #0
        bne binloop

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP

shownum     PROC ; takes binary half-word from R0 and outputs decimal over serial
        EXPORT shownum
        ; push LR to stack
        push {LR}

        ; Load R0 half-word to RAM for bindec to use it
        ldr R3, =num3
        strh R0, [R3]
        bl bindec

        ; Loop through decimal characters until all printed
        mov R2, #0
sloop   ldr R3, =array3
        add R3, R2
        ldrb R0, [R3]
        bl showchar
        add R2, #1
        cmp R2, #6
        bne sloop
		
		; Write newline afterwards, forces buffer empty
		mov R0, #0x0A
		bl showchar
		mov R0, #0x0D
		bl showchar

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP
    END
