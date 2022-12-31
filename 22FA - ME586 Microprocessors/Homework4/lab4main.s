; Harsh Savla & TJ Wiegman
; ME 58600
; 2022-09-26
; lab4main.s

EXTI_PR         EQU 0x40010414

; allocate some RAM for clicks and outclicks
        AREA MyData, DATA, READWRITE
clicks      SPACE 2
outclicks   SPACE 2

; program code
        AREA ARMex, CODE, READONLY
        ENTRY
__main PROC
        EXPORT __main
        IMPORT initcom
        IMPORT initint
        IMPORT inittime
        IMPORT shownum
        IMPORT checkcom
        IMPORT getchar

        ; Initialize serial communications
        bl initcom

        ; Initialize clicks and outclicks to zero
        mov R0, #0
        ldr R3, =clicks
        strh R0, [R3]
        ldr R3, =outclicks
        strh R0, [R3]

        ; Set up external clock interrupt
        bl initint

        ; Set up 100ms timer
        bl inittime

        ; Main loop
chloop  ldr R3, =outclicks
        ldrh R1, [R3]
        cmp R1, #0x0000
        bne shclick
        
        ; Check if received serial comm
        bl checkcom
        cmp R0, #0xFF
        bne chloop

        ; Check if received character is ESC
        bl getchar
        cmp R0, #0x1B   ; ASCII value for ESCAPE
        beq done
        b chloop

shclick mov R0, R1 ; copy outclicks from R1 to R0
        mov R1, #0
        strh R1, [R3] ; reset outclicks to zero
        bl shownum ; R0 still holds old outclicks value
        b chloop
        
done    b done
        ENDP

EXTI1_IRQHandler    PROC
        EXPORT EXTI1_IRQHandler
        ; push LR to stack
        push {LR}
        
        ; Clear interrupt flag
        ldr R3, =EXTI_PR
        mov R1, #0x02
        str R1, [R3]

        ; Increase clicks value
        ldr R3, =clicks
        ldrh R1, [R3]
        add R1, #1
        strh R1, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP

SysTick_Handler     PROC
        EXPORT SysTick_Handler
        ; push LR to stack
        push {LR}

        ; Get clicks, then reset
        ldr R3, =clicks
        ldrh R1, [R3]
        mov R2, #0
        strh R2, [R3]

        ; Move clicks to outclicks
        ldr R3, =outclicks
        strh R1, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP
    END