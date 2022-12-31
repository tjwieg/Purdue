        AREA ARMex, CODE, READONLY
logic      PROC
        EXPORT logic
        push {LR}   ; push LR to stack

        ; Check X
        and R3, R0, R1
        and R2, R3, #0x01 ; mask to only 1st bit

        ; Check Y
        and R3, #0x04 ; mask to only 3rd bit
        lsr R3, #1
        orr R2, R3

        ; Check Z
        lsr R3, #1
        and R3, R2
        and R4, R0, R1
        and R4, #0x02 ; mask to only 2nd bit
        lsr R4, #1
        and R3, R4
        lsl R3, #2
        orr R2, R3

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        
        ENDP
    END