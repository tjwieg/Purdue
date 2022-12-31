; Equate statements
RCC_APB2ENR     EQU 0x40021018
IOPB            EQU 2_00001000 ; same as 0x08
IOPC            EQU 2_00010000 ; same as 0x10

GPIOB_CRL       EQU 0x40010C00
GPIOB_CRH       EQU 0x40010C04

GPIOC_CRL       EQU 0x40011000
GPIOC_CRH       EQU 0x40011004

        AREA ARMex, CODE, READONLY
initports   PROC
        EXPORT initports
        ; push LR to stack
        push {LR}
        
        ; Adjust APB2 state
        ldr R3, =RCC_APB2ENR
        ldr R1, [R3] ; save current APB2 state
        orr R1, #IOPB+IOPC
        str R1, [R3]

        ; Adjust GPIOB pin modes
        ldr R3, =GPIOB_CRL
        ldr R1, =0x44444444
        str R1, [R3]

        ldr R3, =GPIOB_CRH
        str R1, [R3]

        ; Adjust GPIOC pin modes
        ldr R3, =GPIOC_CRL
        ldr R1, =0x33333333
        str R1, [R3]

        ldr R3, =GPIOC_CRH
        str R1, [R3]

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        
        ENDP
    END