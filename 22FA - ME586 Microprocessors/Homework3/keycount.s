; Harsh Savla & TJ Wiegman
; ME 58600
; 2022-09-19
; keycount.s

; allocate some RAM for bindec
        AREA MyData, DATA, READWRITE
counter FILL 2, 0x00

; main program code
        AREA ARMex, CODE, READONLY
        ENTRY
__main  PROC
        EXPORT __main
        IMPORT initcom
        IMPORT checkcom
        IMPORT getchar
        IMPORT shownum

        ; Initialize serial communications
        bl initcom

        ; Check if received character from serial
chloop  bl checkcom
        cmp R0, #0xFF
        bne chloop

        ; If received, fetch character into R0
        bl getchar

        ; If escape character, end program
        cmp R0, #0x1B ; ASCII for ESC
        beq done

        ; If 's' goto incrementer
        cmp R0, #0x73 ; ASCII for 's'
        beq incr
        ; If 'd' goto decrementer
        cmp R0, #0x64 ; ASCII for 'd'
        beq decr
        ; Else check next character
        b chloop

        ; Increment counter
incr    ldr R3, =counter
        ldrh R0, [R3]
        add R0, #1
        strh R0, [R3]
        b out0

        ; Decrement counter
decr    ldr R3, =counter
        ldrh R0, [R3]
        sub R0, #1
        strh R0, [R3]

        ; Output decimal of R0 (counter) over serial
out0    bl shownum
        b chloop

done    b done
        ENDP
    END