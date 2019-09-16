;
; jsimdcpu.asm - SIMD instruction support check
;
; Copyright 2009 Pierre Ossman <ossman@cendio.se> for Cendio AB
; Copyright (C) 2016, D. R. Commander.
;
; Based on
; x86 SIMD extension for IJG JPEG library
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

; --------------------------------------------------------------------------
    SECTION     SEG_TEXT
    BITS        64
;
; Check if the CPU supports SIMD instructions
;
; GLOBAL(unsigned int)
; jpeg_simd_cpu_support(void)
;

    align       32
    GLOBAL_FUNCTION(jpeg_simd_cpu_support)

EXTN(jpeg_simd_cpu_support):
    push        rbx
    push        rdi


    ; Assume that all x86-64 processors support SSE & SSE2 instructions
    or          rdi, JSIMD_SSE2
    or          rdi, JSIMD_SSE

    ; Check whether CPUID leaf 07H is supported
    ; (leaf 07H is used to check for AVX2 instruction support)
    mov         rax, 0
    cpuid
    cmp         rax, 7
    jl          short .return           ; Maximum leaf < 07H

    ; Check for AVX2 instruction support
    xor         ecx, ecx
    lea         edi, [rcx+(JSIMD_SSE2|JSIMD_SSE)]
    lea         eax, [rcx+7]
    cpuid

    test        bl, 1<<5               ; bit5:AVX2
    jz          short .return

    ; Check for AVX2 O/S support
    xor         ecx, ecx
    lea         eax, [rcx+1]
    cpuid
    xor		ebx, ebx
    shr		ecx, 27
    and		ecx, 3
    cmp		 cl, 3
    sete         bl                      ; O/S does support XSAVE+AVX2
    
    xor         ecx, ecx
    xgetbv
    xor		edx, edx
    mov		ecx, edi
    or		ecx, JSIMD_AVX2
    and         eax, 6
    cmp          al, 6                  ; O/S manage XMM/YMM state with XSAVE
    sete	 dl
    test	 bl, dl
    cmovnz	edi, ecx 
.return:
    mov         eax, edi

    pop         rdi
    pop         rbx
    ret

; For some reason, the OS X linker does not honor the request to align the
; segment unless we do this.
    align       32
