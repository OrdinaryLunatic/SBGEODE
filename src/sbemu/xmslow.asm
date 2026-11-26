; Real-mode legacy XMS filter. The installer patches the header before publication.
bits 16
org 0
    jmp short multiplex
    dw 584ch
previous_int2f dd 0
manager dd 0
passive db 0                ; offset 12: failed TSR install
issued db 0                 ; offset 13: someone may have cached control
busy db 0
pending db 0                ; 0=none, 1=free, 2=unlock then free
handle dw 0

multiplex:
    pushf
    cmp ax,4310h
    jne .chain
    cmp byte [cs:passive],0
    jne .chain
    popf
    pushf
    call far [cs:previous_int2f]
    mov byte [cs:issued],1
    push cs
    pop es
    mov bx,control
    iret
.chain:
    popf
    jmp far [cs:previous_int2f]

control:
    pushf
    cmp byte [cs:passive],0
    jne pass
    cmp ah,08h
    je filtered
    cmp ah,09h
    jne pass
    test dx,dx
    jnz filtered
pass:
    popf
    jmp far [cs:manager]

filtered:
    cli
    cmp byte [cs:busy],0
    jne busy_error
    mov byte [cs:busy],1
    popf
    push ax
    push dx
    call cleanup
    pop dx
    pop ax
    jc failed
    cmp ah,08h
    je query
    call low_block
finish:
    mov byte [cs:busy],0
    retf
busy_error:
    popf
    cmp ah,08h
    jne .error
    xor dx,dx
.error:
    xor ax,ax
    mov bl,0a0h
    retf
failed:
    cmp ah,08h
    jne .error
    xor dx,dx
.error:
    xor ax,ax
    jmp finish

query:
    call far [cs:manager]
    test ax,ax
    jz .none
    push si
    push di
    xor si,si
    mov di,15360            ; extended memory begins at or above 1 MiB
    cmp di,ax
    jbe .next
    mov di,ax
.next:
    cmp si,di
    jae .done
    mov dx,di
    sub dx,si
    inc dx
    shr dx,1
    add dx,si
    push dx
    call low_block
    test ax,ax
    jz .smaller
    mov [cs:handle],dx
    mov byte [cs:pending],1
    call cleanup
    pop si
    jc .abort
    jmp .next
.smaller:
    pop di
    cmp byte [cs:pending],0
    jne .abort
    cmp bl,0a0h             ; only insufficient space narrows the search
    jne .abort
    dec di
    jmp .next
.done:
    xor bl,bl
    test si,si
    jnz .result
    mov bl,0a0h
    jmp .result
.abort:
    xor si,si
.result:
    mov ax,si
    pop di
    pop si
.none:
    mov dx,ax
    jmp finish

; Validate the exact handle allocated for this request. Keep the native
; allocation result on the stack while Lock returns the byte address in DX:BX.
low_block:
    push bp
    mov bp,sp
    push dx
    mov ah,09h
    call far [cs:manager]
    test ax,ax
    jz .done
    mov [cs:handle],dx
    mov byte [cs:pending],1
    push bx
    mov ah,0ch
    call far [cs:manager]
    test ax,ax
    jz .reject
    mov byte [cs:pending],2
    cmp dx,100h
    jae .outside
    shl dx,6
    mov ax,bx
    shr bx,10
    add dx,bx
    test ax,3ffh
    jz .rounded
    inc dx
.rounded:
    add dx,[bp-2]
    jc .outside
    cmp dx,16384
    ja .outside
    mov dx,[cs:handle]
    mov ah,0dh
    call far [cs:manager]
    test ax,ax
    jz .reject
    mov byte [cs:pending],0
    mov dx,[cs:handle]
    pop bx
    jmp .done
.outside:
    mov bl,0a0h
.reject:
    push bx
    call cleanup
    pop ax
    jc .cleanup_error
    mov bx,ax
.cleanup_error:
    xor ax,ax
    mov dx,[bp-2]
.done:
    mov sp,bp
    pop bp
    ret

; Only this wrapper owns the pending handle. A manager refusing cleanup
; leaves that operation pending. Retry before creating another allocation.
cleanup:
    cmp byte [cs:pending],0
    je .ok
    cmp byte [cs:pending],2
    jne .free
    mov dx,[cs:handle]
    mov ah,0dh
    call far [cs:manager]
    test ax,ax
    jnz .unlocked
    cmp bl,0aah             ; XMS reports that this block is already unlocked
    jne .error
.unlocked:
    mov byte [cs:pending],1
.free:
    mov dx,[cs:handle]
    mov ah,0ah
    call far [cs:manager]
    test ax,ax
    jz .error
    mov byte [cs:pending],0
.ok:
    clc
    ret
.error:
    stc
    ret
