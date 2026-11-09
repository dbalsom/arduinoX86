; Segment boundary observations for the 286 and 386EX.
; Built by `cargo run -p test_generator --bin segment_limits -- ...`.
; Each invocation starts in real mode and executes exactly one probe.

bits 16
org 0
%if CPU == 286
cpu 286
%else
cpu 386
%endif

%define CODE_BASE 0x10000
%define DATA_BASE 0x20000
%define CODE_SEL 8
%define RESULT_SEL 16
%define STACK_SEL 24
%define TEST_SEL 32

; The host reads these offsets, rather than depending on instruction lengths.
db 'SLIM', 1, 0
dw entry, probe, after_probe, result, stopped, end_program

entry:
%ifdef SMOKE
    SMOKE_SETUP
%endif
    cli
    cld
    mov ax, 0x1000
    mov ds, ax
    mov es, ax
    mov ax, 0x3000
    mov ss, ax
    mov sp, 0x8000
%if CPU == 386
    mov esp, 0x8000
%endif

%if PROTECTED
    lgdt [gdtr]
    lidt [idtr]
    smsw ax
    or ax, 1
    lmsw ax
    jmp CODE_SEL:protected_entry
protected_entry:
    mov ax, RESULT_SEL
    mov es, ax
    mov ax, STACK_SEL
    mov ss, ax
    mov ax, TEST_SEL
%if STACK
    mov ss, ax
%else
    mov ds, ax
%endif
%else
    xor ax, ax
    mov es, ax
%assign vector 0
%rep 32
    mov word [es:vector * 4], handler_%+vector
    mov word [es:vector * 4 + 2], 0x1000
%assign vector vector + 1
%endrep
    mov ax, 0x1000
    mov es, ax
    mov ax, 0x2000
%if STACK
    mov ss, ax
%else
    mov ds, ax
%endif
%endif

%if CPU == 386
    mov ebx, OFFSET
    mov eax, 0xa5c39e71
    mov [es:result + 16], esp
%else
    mov bx, OFFSET
    mov ax, 0x9e71
    mov [es:result + 16], sp
%endif

%if STACK
%define SEG ss
%else
%define SEG ds
%endif
%if ADDRESS32
%define ADDR ebx
%define ADDRSIZE a32
%else
%define ADDR bx
%define ADDRSIZE a16
%endif
%if DIRECT
%define ADDR OFFSET
%endif
%if WIDTH == 1
%define VALUE al
%elif WIDTH == 2
%define VALUE ax
%else
%define VALUE eax
%endif
%ifdef SMOKE_FAULT
%if SMOKE_FAULT == 12
    mov dx, 40
%endif
%endif
probe:
%ifdef SMOKE_FAULT
%if SMOKE_FAULT == 6
    db 0x0f, 0x0b
%elif SMOKE_FAULT == 12
    mov ss, dx               ; The local test supplies a nonpresent descriptor.
%elif SMOKE_FAULT == 13
    mov ds, ax               ; The sentinel is outside the GDT.
%endif
%else
%if WRITE
    ADDRSIZE mov [SEG:ADDR], VALUE
%else
    ADDRSIZE mov VALUE, [SEG:ADDR]
%endif
%endif
after_probe:
%if CPU == 386
    mov [es:result + 12], eax
    mov [es:result + 20], esp
%else
    mov [es:result + 12], ax
    mov [es:result + 20], sp
%endif
    smsw ax
    mov [es:result + 24], ax
    mov word [es:result], 0x534c
%ifdef SMOKE
    mov si, result
    mov cx, 28
.print:
    mov al, [es:si]
    out 0xe9, al
    inc si
    loop .print
    mov al, 0
    out 0xf4, al
%endif
    jmp stopped
    times 64 db 0x90
align 16, db 0x90
stopped:
    jmp stopped

; All exceptions have a common software frame: vector, error, IP, CS, FLAGS.
; Keep the original fault address and skip the probe only after recording it.
%assign vector 0
%rep 32
handler_%+vector:
%if !PROTECTED || (vector != 8 && vector != 10 && vector != 11 && vector != 12 && vector != 13 && vector != 14 && vector != 17)
    push word 0xffff
%endif
    push word vector
    jmp fault
%assign vector vector + 1
%endrep
fault:
%if CPU == 386
    mov [es:result + 12], eax
%else
    mov [es:result + 12], ax
%endif
    push bp
    mov bp, sp
    mov ax, [ss:bp + 2]
    mov [es:result + 2], ax
    mov ax, [ss:bp + 4]
    mov [es:result + 4], ax
    mov ax, [ss:bp + 6]
    mov [es:result + 6], ax
    mov ax, [ss:bp + 8]
    mov [es:result + 8], ax
    mov ax, [ss:bp + 10]
    mov [es:result + 10], ax
    mov word [ss:bp + 6], after_probe
%if PROTECTED
    mov word [ss:bp + 8], CODE_SEL
%else
    mov word [ss:bp + 8], 0x1000
%endif
%if CPU == 386
    mov eax, [es:result + 12]
%else
    mov ax, [es:result + 12]
%endif
    pop bp
    add sp, 4
    iret

align 8, db 0
gdt:
    dq 0
    dw 0xffff, CODE_BASE & 0xffff
    db CODE_BASE >> 16, 0x9b, 0, 0
    dw 0xffff, CODE_BASE & 0xffff
    db CODE_BASE >> 16, 0x93, 0, 0
    dw 0xffff, 0
    db 3, 0x93, 0, 0
%if LIMIT > 0xfffff
%if (LIMIT & 0xfff) != 0xfff
%error "Page granular limit must end in fff"
%endif
%define ENCODED_LIMIT (LIMIT >> 12)
%define GRANULARITY 0x80
%else
%define ENCODED_LIMIT LIMIT
%define GRANULARITY 0
%endif
    dw ENCODED_LIMIT & 0xffff, DATA_BASE & 0xffff
    db DATA_BASE >> 16, 0x93 | (DOWN << 2)
    db ((ENCODED_LIMIT >> 16) & 15) | GRANULARITY | (BIG << 6), 0
%ifdef SMOKE_FAULT
%if SMOKE_FAULT == 12
    dw 0xffff, 0
    db 2, 0x13, 0, 0
%endif
%endif
gdt_end:
gdtr:
    dw gdt_end - gdt - 1
    dd CODE_BASE + gdt
idt:
%assign vector 0
%rep 32
    dw handler_%+vector, CODE_SEL
    db 0, 0x86
    dw 0
%assign vector vector + 1
%endrep
idt_end:
idtr:
    dw idt_end - idt - 1
    dd CODE_BASE + idt

align 4, db 0
result:
    dw 0, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff
    dd 0, 0, 0
    dw 0, 0
end_program:
