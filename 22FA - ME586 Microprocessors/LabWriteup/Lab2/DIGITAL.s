; Equate statements
GPIOB_IDR       EQU 0x40010C08
GPIOC_ODR       EQU 0x4001100C

        AREA ARMex, CODE, READONLY
        ENTRY
__main  PROC
        EXPORT __main
        IMPORT initports
        IMPORT logic

        bl initports

loop    ldr R3, =GPIOB_IDR
        ldrh R4, [R3]
        and R0, R4, #0x00E0
        ror R0, #5 ; store PB5-7 into R0
        and R1, R4, #0x0700
        ror R1, #8 ; store PB8-10 into R1

        bl logic

        ldr R3, =GPIOC_ODR
        lsl R2, #6
        str R2, [R3]

        b loop
        ENDP
    END