; Harsh Savla & TJ Wiegman
; ME 58600
; 2022-09-26
; initint.s

; a subroutine to enable the external IRQ interrupt pin
RCC_APB2ENR     EQU 0x40021018
GPIOA_CRL       EQU 0x40010800
EXTI_IMR        EQU 0x40010400
EXTI_FTSR       EQU 0x4001040C
NVIC_ISER0      EQU 0xE000E100

; program code
        AREA ARMex, CODE, READONLY
        ENTRY
initint PROC
        EXPORT initint
        ; push LR to stack
        push {LR}

        ; Enable GPIO A
        ldr R3, =RCC_APB2ENR
        ldrb R1, [R3]
        orr R1, #0x04  ; enable bit 2 (0100)
        strb R1, [R3]

        ; Configure PA01
        ldr R3, =GPIOA_CRL
        ldrb R1, [R3]
        and R1, #0x0F ; set PA01 to 0000
        orr R1, #0x40 ; set PA01 to 0100
        strb R1, [R3]

        ; Unmask External Interrupt 1
        ldr R3, =EXTI_IMR
        mov R1, #0x02
        str R1, [R3]

        ; Set trigger to falling edge
        ldr R3, =EXTI_FTSR
        mov R1, #0x02
        str R1, [R3]

        ; Enable in NVIC (?)
        ldr R3, =NVIC_ISER0
        mov R1, #0x80
        str R1, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        ENDP
    END