        AREA ARMex, CODE, READONLY
logic      PROC
        EXPORT logic
        ; push LR to stack
        push {LR}

        ; Check X
        and R3, R0, R1
        and R2, R3, #0x01 ; mask to only 1st bit

        ; Check Y
		eor R3, R0, R1
		mvn R3, R3
        and R3, #0x04 ; mask to only 3rd bit
        lsr R3, #1
        orr R2, R3

        ; Check Z
        eor R3, R0, R1
		mvn R3, R3
		and R3, #0x07
		cmp R3, #0x07
		beq allsame
nsame   mov R3, #0x00
		b save2
allsame mov R3, #0x04
save2   orr R2, R3

        ; End subroutine and go back to caller
        pop {LR}
        bx LR
        
        ENDP
    END