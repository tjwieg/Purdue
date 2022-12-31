;Equates
;myEquate	EQU 0x12345678

           	AREA MyConst, DATA, READONLY	; allocates memory for initialized data

;const_label	Your DCB, , DCW, DCD, etc. statements go here.

			AREA MyData, DATA, READWRITE  ; allocates memory for uninitialized data

;data_label	Your SPACE, etc. statements go here.
	
			AREA    ARMex, CODE, READONLY
			ENTRY
__main		PROC
			EXPORT  __main
			;IMPORT my_subroutine
;------------------------------------------------------------------------------------
;your code goes here
			nop

done    	b done
		
			ENDP
			END