			AREA    ARMex, CODE, READONLY
			ENTRY
__main		PROC
			EXPORT  __main
			IMPORT bindec
			IMPORT initcom
			IMPORT shownum
;------------------------------------------------------------------------------------
;your code goes here

			bl initcom
			mov R0, #243
			bl shownum
			mov R0, #0xF0FA
			bl shownum
done		b done
		
			ENDP

	END