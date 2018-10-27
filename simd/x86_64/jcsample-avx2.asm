;
; jcsample.asm - downsampling (64-bit AVX2)
;
; Copyright 2009 Pierre Ossman <ossman@cendio.se> for Cendio AB
; Copyright (C) 2009, 2016, D. R. Commander.
; Copyright (C) 2015, Intel Corporation.
;
; Based on the x86 SIMD extension for IJG JPEG library
; Copyright (C) 1999-2006, MIYASAKA Masaru.
; For conditions of distribution and use, see copyright notice in jsimdext.inc
;
; This file should be assembled with NASM (Netwide Assembler),
; can *not* be assembled with Microsoft's MASM or any compatible
; assembler (including Borland's Turbo Assembler).
; NASM is available from http://nasm.sourceforge.net/ or
; http://sourceforge.net/project/showfiles.php?group_id=6208
;
; [TAB8]

%include "jsimdext.inc"
%use smartalign
ALIGNMODE P6

; --------------------------------------------------------------------------
    SECTION     SEG_TEXT
    BITS        64
;
; Downsample pixel values of a single component.
; This version handles the common case of 2:1 horizontal and 1:1 vertical,
; without smoothing.
;
; GLOBAL(void)
; jsimd_h2v1_downsample_avx2(JDIMENSION image_width, int max_v_samp_factor,
;                            JDIMENSION v_samp_factor,
;                            JDIMENSION width_in_blocks, JSAMPARRAY input_data,
;                            JSAMPARRAY output_data);
;

; r10d = JDIMENSION image_width
; r11 = int max_v_samp_factor
; r12d = JDIMENSION v_samp_factor
; r13d = JDIMENSION width_in_blocks
; r14 = JSAMPARRAY input_data
; r15 = JSAMPARRAY output_data

    align       32
    GLOBAL_FUNCTION(jsimd_h2v1_downsample_avx2)

EXTN(jsimd_h2v1_downsample_avx2):
    push        rbp
    mov         rax, rsp
    mov         rbp, rsp
    collect_args 6

    shl         r13, 3                  ; imul r13,DCTSIZE (r13 = output_cols)
    jz          near .return


    ; -- expand_right_edge

    mov         rcx, r13
    shl         rcx, 1                  ; output_cols * 2
    sub         rcx, r10
    jle         short .expand_end

    test        r11, r11
    jle         short .expand_end

    cld
    mov         rsi, r14                ; input_data
.expandloop:
    mov         rdx, rcx

    mov         rdi, JSAMPROW [rsi]
    add         rdi, r10
    movzx       eax, JSAMPLE [rdi-1]

    rep stosb

    mov         rcx, rdx

    add         rsi, byte SIZEOF_JSAMPROW
    sub         r11, 1
    jg          short .expandloop

align 16
.expand_end:

    ; -- h2v1_downsample

    mov         eax, r12d               ; rowctr
    test        r12d, r12d
    jle         near .return

    mov          edx, 0x00010000        ; bias pattern
    vmovd        xmm7, edx
    vpbroadcastd ymm7, xmm7             ; xmm7={0, 1, 0, 1, 0, 1, 0, 1}
    vpcmpeqw     ymm6, ymm6, ymm6
    vpsrlw       ymm6, ymm6, BYTE_BIT   ; ymm6={0xFF 0x00 0xFF 0x00 ..}

align 16
.rowloop:
    mov         rcx, r13                ; output_cols
    mov         rsi, JSAMPROW [r14]     ; inptr
    mov         rdi, JSAMPROW [r15]     ; outptr

    cmp         rcx, byte SIZEOF_YMMWORD
    jb          .columnloop_r24
align 16
.columnloop:
    vmovdqu     ymm0, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     ymm1, YMMWORD [rsi+1*SIZEOF_YMMWORD]

.downsample:
    vpsrlw      ymm2, ymm0, BYTE_BIT
    vpand       ymm0, ymm0, ymm6
    vpsrlw      ymm3, ymm1, BYTE_BIT
    vpand       ymm1, ymm1, ymm6

    vpaddw      ymm0, ymm0, ymm2
    vpaddw      ymm1, ymm1, ymm3
    vpaddw      ymm0, ymm0, ymm7
    vpaddw      ymm1, ymm1, ymm7
    vpsrlw      ymm0, ymm0, 1
    vpsrlw      ymm1, ymm1, 1

    vpackuswb   ymm0, ymm0, ymm1
    vpermq      ymm0, ymm0, 0xd8

    vmovdqu     YMMWORD [rdi+0*SIZEOF_YMMWORD], ymm0

    sub         rcx, byte SIZEOF_YMMWORD    ; outcol
    add         rsi, byte 2*SIZEOF_YMMWORD  ; inptr
    add         rdi, byte 1*SIZEOF_YMMWORD  ; outptr
    cmp         rcx, byte SIZEOF_YMMWORD
    jae         .columnloop
    test        ecx, ecx
    jnz         .columnloop_r24


    add         r14, byte SIZEOF_JSAMPROW  ; input_data
    add         r15, byte SIZEOF_JSAMPROW  ; output_data
    sub         eax, 1                     ; rowctr
    jg          .rowloop

.return:
    vzeroupper
    uncollect_args 6
    pop         rbp
    ret

.columnloop_r24:
    ; rcx can possibly be 8, 16, 24
    cmp         ecx, 24
    jne         .columnloop_r16
    vmovdqu     ymm0, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     xmm1, XMMWORD [rsi+1*SIZEOF_YMMWORD]
    mov         ecx, SIZEOF_YMMWORD
    jmp         .downsample

.columnloop_r16:
    cmp         ecx, 16
    jne         .columnloop_r8
    vmovdqu     ymm0, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vpxor       xmm1, xmm1, xmm1
    mov         ecx, SIZEOF_YMMWORD
    jmp         .downsample

.columnloop_r8:
    vmovdqu     xmm0, XMMWORD[rsi+0*SIZEOF_YMMWORD]
    vpxor       xmm1, xmm1, xmm1
    mov         ecx, SIZEOF_YMMWORD
    jmp         .downsample


; --------------------------------------------------------------------------
;
; Downsample pixel values of a single component.
; This version handles the standard case of 2:1 horizontal and 2:1 vertical,
; without smoothing.
;
; GLOBAL(void)
; jsimd_h2v2_downsample_avx2(JDIMENSION image_width, int max_v_samp_factor,
;                            JDIMENSION v_samp_factor,
;                            JDIMENSION width_in_blocks, JSAMPARRAY input_data,
;                            JSAMPARRAY output_data);
;

; r10d = JDIMENSION image_width
; r11 = int max_v_samp_factor
; r12d = JDIMENSION v_samp_factor
; r13d = JDIMENSION width_in_blocks
; r14 = JSAMPARRAY input_data
; r15 = JSAMPARRAY output_data

    align       32
    GLOBAL_FUNCTION(jsimd_h2v2_downsample_avx2)

EXTN(jsimd_h2v2_downsample_avx2):
    push        rbp
    mov         rax, rsp
    mov         rbp, rsp
    collect_args 6

    shl         r13, 3                  ; imul r13,DCTSIZE (r13 = output_cols)
    jz          near .return


    ; -- expand_right_edge

    mov		rcx, r13
    shl         rcx, 1                  ; output_cols * 2
    sub         rcx, r10
    jle         short .expand_end

    test        r11, r11
    jle         short .expand_end

    cld
    mov         rsi, r14                ; input_data
.expandloop:
    mov         rdx, rcx
    mov         rdi, JSAMPROW [rsi]
    add         rdi, r10
    movzx       eax, JSAMPLE [rdi-1]

    rep stosb

    mov         rcx, rdx

    add         rsi, byte SIZEOF_JSAMPROW
    sub         r11, 1
    jg          short .expandloop

align 16
.expand_end:

    ; -- h2v2_downsample

    mov         eax, r12d               ; rowctr
    test        r12d,r12d
    jle         near .return

    mov          edx, 0x00020001         ; bias pattern
    vmovd        xmm7, edx
    vpcmpeqw     ymm6, ymm6, ymm6
    vpbroadcastd ymm7, xmm7             ; ymm7={1, 2, 1, 2, 1, 2, 1, 2}
    vpsrlw       ymm6, ymm6, BYTE_BIT   ; ymm6={0xFF 0x00 0xFF 0x00 ..}

align 16
.rowloop:
    mov         rcx, r13                	       ; output_cols
    mov         rdx, JSAMPROW [r14+0*SIZEOF_JSAMPROW]  ; inptr0
    mov         rsi, JSAMPROW [r14+1*SIZEOF_JSAMPROW]  ; inptr1
    mov         rdi, JSAMPROW [r15]                    ; outptr

    cmp         ecx, byte SIZEOF_YMMWORD
    jb          near .columnloop_r24

align 16
.columnloop:
    vmovdqu     ymm0, YMMWORD [rdx+0*SIZEOF_YMMWORD]
    vmovdqu     ymm1, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     ymm2, YMMWORD [rdx+1*SIZEOF_YMMWORD]
    vmovdqu     ymm3, YMMWORD [rsi+1*SIZEOF_YMMWORD]

.downsample:
    vpand       ymm4, ymm0, ymm6
    vpsrlw      ymm0, ymm0, BYTE_BIT
    vpand       ymm5, ymm1, ymm6
    vpsrlw      ymm1, ymm1, BYTE_BIT
    vpaddw      ymm0, ymm0, ymm4
    vpaddw      ymm1, ymm1, ymm5

    vpand       ymm4, ymm2, ymm6
    vpsrlw      ymm2, ymm2, BYTE_BIT
    vpand       ymm5, ymm3, ymm6
    vpsrlw      ymm3, ymm3, BYTE_BIT
    vpaddw      ymm2, ymm2, ymm4
    vpaddw      ymm3, ymm3, ymm5

    vpaddw      ymm0, ymm0, ymm1
    vpaddw      ymm2, ymm2, ymm3
    vpaddw      ymm0, ymm0, ymm7
    vpaddw      ymm2, ymm2, ymm7
    vpsrlw      ymm0, ymm0, 2
    vpsrlw      ymm2, ymm2, 2

    vpackuswb   ymm0, ymm0, ymm2
    vpermq      ymm0, ymm0, 0xd8

    vmovdqu     YMMWORD [rdi+0*SIZEOF_YMMWORD], ymm0

    sub         rcx, byte SIZEOF_YMMWORD    ; outcol
    add         rdx, byte 2*SIZEOF_YMMWORD  ; inptr0
    add         rsi, byte 2*SIZEOF_YMMWORD  ; inptr1
    add         rdi, byte 1*SIZEOF_YMMWORD  ; outptr
    cmp         rcx, byte SIZEOF_YMMWORD
    jae         near .columnloop
    test        ecx, ecx
    jnz         short .columnloop_r24


    add         r14, byte 2*SIZEOF_JSAMPROW  ; input_data
    add         r15, byte 1*SIZEOF_JSAMPROW  ; output_data
    sub         eax, 1                       ; rowctr
    jg          near .rowloop

.return:
    vzeroupper
    uncollect_args 6
    pop         rbp
    ret

align 16
.columnloop_r24:
    cmp         ecx, 24
    jne         .columnloop_r16
    vmovdqu     ymm0, YMMWORD [rdx+0*SIZEOF_YMMWORD]
    vmovdqu     ymm1, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     xmm2, XMMWORD [rdx+1*SIZEOF_YMMWORD]
    vmovdqu     xmm3, XMMWORD [rsi+1*SIZEOF_YMMWORD]
    mov         ecx, SIZEOF_YMMWORD
    jmp         near .downsample

.columnloop_r16:
    cmp         ecx, 16
    jne         .columnloop_r8
    vmovdqu     ymm0, YMMWORD [rdx+0*SIZEOF_YMMWORD]
    vmovdqu     ymm1, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vpxor       xmm2, xmm2, xmm2
    vpxor       xmm3, xmm3, xmm3
    mov         ecx, SIZEOF_YMMWORD
    jmp         near .downsample

.columnloop_r8:
    vmovdqu     xmm0, XMMWORD [rdx+0*SIZEOF_XMMWORD]
    vmovdqu     xmm1, XMMWORD [rsi+0*SIZEOF_XMMWORD]
    vpxor       xmm2, xmm2, xmm2
    vpxor       xmm3, xmm3, xmm3
    mov         ecx, SIZEOF_YMMWORD
    jmp         near .downsample


; For some reason, the OS X linker does not honor the request to align the
; segment unless we do this.
    align       32
