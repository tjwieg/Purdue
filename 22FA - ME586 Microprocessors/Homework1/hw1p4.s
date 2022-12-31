; Homework 1, Problem 4

avg_size EQU 10     ; how many numbers to average?

        AREA MyData, DATA, READWRITE  ; allocates RAM
array1 		SPACE 10
	
		AREA    ARMex, CODE, READONLY ; code goes here
			ENTRY
__main		PROC
			EXPORT  __main

			mov R4, #0x01       ; R4 is number of samples
			ldr R5, =array1
			ldrb R0, [R5], #1
			ldrb R1, [R5], #1

loop_add	add R0, R1          ; R0 is running total
			add R4, #1          ; one more sample in R0
			ldrb R1, [R5], #1
			cmp R4, #avg_size
			bne loop_add

b_div		udiv R3, R0, R4     ; R3 is quotient
			mul R1, R3, R4
			sub R2, R0, R1      ; R2 is remainder
done		b done
		ENDP
	END