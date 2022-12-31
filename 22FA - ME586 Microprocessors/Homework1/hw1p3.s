var1    EQU 0x20000014

    AREA ARMex, CODE, READONLY
        ENTRY
__main  PROC
        EXPORT __main

        ldr R0, =var1
        ldrb R1, [R0]
        mov R2, R1
        add R2, #0x25
        strb R2, [R0]
        ror R2, #4
        
        ENDP
    END