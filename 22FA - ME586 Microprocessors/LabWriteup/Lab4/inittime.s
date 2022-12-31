; Harsh Savla & TJ Wiegman
; ME 58600
; 2022-09-26
; inittime.s

; a subroutine to set up the SysTick timer interrupt for every 100ms
STK_CTRL    EQU 0xE000E010
STK_LOAD    EQU 0xE000E014

; program code
        AREA ARMex, CODE, READONLY
        ENTRY
inittime PROC
        EXPORT inittime
        ; push LR to stack
        push {LR}

        ; Set timer to 100 ms
        ldr R3, =STK_LOAD
        ldr R1, =0x249EFF
        str R1, [R3]

        ; Enable timer, systick interrupt, and 24MHz counter
        ldr R3, =STK_CTRL
        mov R1, #0x07
        str R1, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP
    END