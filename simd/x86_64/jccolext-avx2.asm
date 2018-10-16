;
; jccolext.asm - colorspace conversion (64-bit AVX2)
;
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

%include "jcolsamp.inc"
%use smartalign
ALIGNMODE P6

; --------------------------------------------------------------------------
;
; Convert some rows of samples to the output colorspace.
;
; GLOBAL(void)
; jsimd_rgb_ycc_convert_avx2(JDIMENSION img_width, JSAMPARRAY input_buf,
;                            JSAMPIMAGE output_buf, JDIMENSION output_row,
;                            int num_rows);
;

; r10d = JDIMENSION img_width
; r11 = JSAMPARRAY input_buf
; r12 = JSAMPIMAGE output_buf
; r13d = JDIMENSION output_row
; r14d = int num_rows

%define wk(i)   rbp - (WK_NUM - (i)) * SIZEOF_YMMWORD  ; ymmword wk[WK_NUM]
%define WK_NUM  6

    align       32
    GLOBAL_FUNCTION(jsimd_rgb_ycc_convert_avx2)

EXTN(jsimd_rgb_ycc_convert_avx2):
    push        rbp
    mov         rax, rsp                     ; rax = original rbp
    sub         rsp, byte 4
    and         rsp, byte (-SIZEOF_YMMWORD)  ; align to 256 bits
    mov         [rsp], rax
    mov         rbp, rsp                     ; rbp = aligned rbp
    lea         rsp, [wk(0)]
    collect_args 5
    push        rbx

    test        r10d, r10d
    jz          near .return


    shl		r13, 3 ; r13*SIZEOF_JSAMPROW	
    mov		rdi, r13
    mov		rbx, r13
    mov		rdx, r13
    add         rdi, JSAMPARRAY [r12+0*SIZEOF_JSAMPARRAY]
    add         rbx, JSAMPARRAY [r12+1*SIZEOF_JSAMPARRAY]
    add         rdx, JSAMPARRAY [r12+2*SIZEOF_JSAMPARRAY]

;	load constants
   lea	 	rax, [rel PW_F0299_F0337]
   vpbroadcastd ymm15, [rax]   ;PW_F0299_F0337
   vpbroadcastd ymm14, [rax+4] ;PW_F0114_F0250 
   vpcmpeqd	ymm0, ymm0, ymm0
   vpxor	xmm1, xmm1, xmm1
   vpbroadcastd ymm13, [rax+8] ;PW_MF016_MF033
   vpbroadcastd ymm12, [rax+12];PW_MF008_MF041
   vpsubd	ymm0, ymm1, ymm0 ; PD_ONE
   vpbroadcastd ymm11, [rax+16];PD_ONEHALFM1_CJ
   vpslld	ymm10, ymm0, (SCALEBITS -1) ; PD_ONEHALF


    mov         rsi, r11
    mov         eax, r14d
    test        r14, r14
    jle         near .return
align 16
.rowloop:
    push        rdx
    push        rbx
    push        rdi
    mov         ecx, r10d               ; col

    mov         rsi, JSAMPROW [r11]     ; inptr
    mov         rdi, JSAMPROW [rdi]     ; outptr0
    mov         rbx, JSAMPROW [rbx]     ; outptr1
    mov         rdx, JSAMPROW [rdx]     ; outptr2

    cmp         ecx, byte SIZEOF_YMMWORD
    jae         near .columnloop

%if RGB_PIXELSIZE == 3  ; ---------------

.column_ld1:
    push        rax
    push        rdx
    lea         ecx, [rcx+rcx*2]        ; imul ecx,RGB_PIXELSIZE
    test        cl, SIZEOF_BYTE
    jz          short .column_ld2
    sub         ecx, byte SIZEOF_BYTE
    movzx       eax, BYTE [rsi+rcx]
.column_ld2:
    test        cl, SIZEOF_WORD
    jz          short .column_ld4
    sub         ecx, byte SIZEOF_WORD
    movzx       edx, WORD [rsi+rcx]
    shl         eax, WORD_BIT
    or          eax, edx
.column_ld4:
    vmovd       xmmA, eax
    pop         rdx
    pop         rax
    test        cl, SIZEOF_DWORD
    jz          short .column_ld8
    sub         ecx, byte SIZEOF_DWORD
    vmovd       xmmF, XMM_DWORD [rsi+rcx]
    vpslldq     xmmA, xmmA, SIZEOF_DWORD
    vpor        xmmA, xmmA, xmmF
.column_ld8:
    test        cl, SIZEOF_MMWORD
    jz          short .column_ld16
    sub         ecx, byte SIZEOF_MMWORD
    vmovq       xmmB, XMM_MMWORD [rsi+rcx]
    vpslldq     xmmA, xmmA, SIZEOF_MMWORD
    vpor        xmmA, xmmA, xmmB
.column_ld16:
    test        cl, SIZEOF_XMMWORD
    jz          short .column_ld32
    sub         ecx, byte SIZEOF_XMMWORD
    vmovdqu     xmmB, XMM_MMWORD [rsi+rcx]
    vinserti128 ymmA, ymmB, xmmA, 1
.column_ld32:
    test        cl, SIZEOF_YMMWORD
    jz          short .column_ld64
    sub         ecx, byte SIZEOF_YMMWORD
    vmovdqa     ymmF, ymmA
    vmovdqu     ymmA, YMMWORD [rsi+0*SIZEOF_YMMWORD]
.column_ld64:
    test        cl, 2*SIZEOF_YMMWORD
    mov         ecx, SIZEOF_YMMWORD
    jz          short .rgb_ycc_cnv
    vmovdqa     ymmB, ymmA
    vmovdqu     ymmA, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     ymmF, YMMWORD [rsi+1*SIZEOF_YMMWORD]
    jmp         short .rgb_ycc_cnv

align 16
.columnloop:
    vmovdqu     ymmA, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     ymmF, YMMWORD [rsi+1*SIZEOF_YMMWORD]
    vmovdqu     ymmB, YMMWORD [rsi+2*SIZEOF_YMMWORD]

align 16
.rgb_ycc_cnv:
    ; ymmA=(00 10 20 01 11 21 02 12 22 03 13 23 04 14 24 05
    ;       15 25 06 16 26 07 17 27 08 18 28 09 19 29 0A 1A)
    ; ymmF=(2A 0B 1B 2B 0C 1C 2C 0D 1D 2D 0E 1E 2E 0F 1F 2F
    ;       0G 1G 2G 0H 1H 2H 0I 1I 2I 0J 1J 2J 0K 1K 2K 0L)
    ; ymmB=(1L 2L 0M 1M 2M 0N 1N 2N 0O 1O 2O 0P 1P 2P 0Q 1Q
    ;       2Q 0R 1R 2R 0S 1S 2S 0T 1T 2T 0U 1U 2U 0V 1V 2V)

    vpblendd  ymmC, ymmA, ymmB, 0x0F ; ymmC=(1L 2L 0M 1M 2M 0N 1N 2N 0O 1O 2O 0P 1P 2P 0Q 1Q
                                     ;       15 25 06 16 26 07 17 27 08 18 28 09 19 29 0A 1A)
    vpblendd  ymmA, ymmF, ymmA, 0x0F ; ymmA=(00 10 20 01 11 21 02 12 22 03 13 23 04 14 24 05
                                     ;       0G 1G 2G 0H 1H 2H 0I 1I 2I 0J 1J 2J 0K 1K 2K 0L)
    vpblendd  ymmB, ymmB, ymmF, 0x0F ; ymmB=(2A 0B 1B 2B 0C 1C 2C 0D 1D 2D 0E 1E 2E 0F 1F 2F
                                     ;       2Q 0R 1R 2R 0S 1S 2S 0T 1T 2T 0U 1U 2U 0V 1V 2V)
    vperm2i128  ymmF, ymmC, ymmC, 1  ; ymmF=(15 25 06 16 26 07 17 27 08 18 28 09 19 29 0A 1A
                                     ;       1L 2L 0M 1M 2M 0N 1N 2N 0O 1O 2O 0P 1P 2P 0Q 1Q)

    vpsrldq     ymmG, ymmA, 8     ; ymmG=(22 03 13 23 04 14 24 05 0G 1G 2G 0H 1H 2H 0I 1I
                                  ;       2I 0J 1J 2J 0K 1K 2K 0L -- -- -- -- -- -- -- --)
    vpslldq     ymmA, ymmA, 8     ; ymmA=(-- -- -- -- -- -- -- -- 00 10 20 01 11 21 02 12
                                  ;       22 03 13 23 04 14 24 05 0G 1G 2G 0H 1H 2H 0I 1I)

    vpunpckhbw  ymmA, ymmA, ymmF  ; ymmA=(00 08 10 18 20 28 01 09 11 19 21 29 02 0A 12 1A
                                  ;       0G 0O 1G 1O 2G 2O 0H 0P 1H 1P 2H 2P 0I 0Q 1I 1Q)
    vpslldq     ymmF, ymmF, 8     ; ymmF=(-- -- -- -- -- -- -- -- 15 25 06 16 26 07 17 27
                                  ;       08 18 28 09 19 29 0A 1A 1L 2L 0M 1M 2M 0N 1N 2N)

    vpunpcklbw  ymmG, ymmG, ymmB  ; ymmG=(22 2A 03 0B 13 1B 23 2B 04 0C 14 1C 24 2C 05 0D
                                  ;       2I 2Q 0J 0R 1J 1R 2J 2R 0K 0S 1K 1S 2K 2S 0L 0T)
    vpunpckhbw  ymmF, ymmF, ymmB  ; ymmF=(15 1D 25 2D 06 0E 16 1E 26 2E 07 0F 17 1F 27 2F
                                  ;       1L 1T 2L 2T 0M 0U 1M 1U 2M 2U 0N 0V 1N 1V 2N 2V)

    vpsrldq     ymmD, ymmA, 8     ; ymmD=(11 19 21 29 02 0A 12 1A 0G 0O 1G 1O 2G 2O 0H 0P
                                  ;       1H 1P 2H 2P 0I 0Q 1I 1Q -- -- -- -- -- -- -- --)
    vpslldq     ymmA, ymmA, 8     ; ymmA=(-- -- -- -- -- -- -- -- 00 08 10 18 20 28 01 09
                                  ;       11 19 21 29 02 0A 12 1A 0G 0O 1G 1O 2G 2O 0H 0P)

    vpunpckhbw  ymmA, ymmA, ymmG  ; ymmA=(00 04 08 0C 10 14 18 1C 20 24 28 2C 01 05 09 0D
                                  ;       0G 0K 0O 0S 1G 1K 1O 1S 2G 2K 2O 2S 0H 0L 0P 0T)
    vpslldq     ymmG, ymmG, 8     ; ymmG=(-- -- -- -- -- -- -- -- 22 2A 03 0B 13 1B 23 2B
                                  ;       04 0C 14 1C 24 2C 05 0D 2I 2Q 0J 0R 1J 1R 2J 2R)

    vpunpcklbw  ymmD, ymmD, ymmF  ; ymmD=(11 15 19 1D 21 25 29 2D 02 06 0A 0E 12 16 1A 1E
                                  ;       1H 1L 1P 1T 2H 2L 2P 2T 0I 0M 0Q 0U 1I 1M 1Q 1U)
    vpunpckhbw  ymmG, ymmG, ymmF  ; ymmG=(22 26 2A 2E 03 07 0B 0F 13 17 1B 1F 23 27 2B 2F
                                  ;       2I 2M 2Q 2U 0J 0N 0R 0V 1J 1N 1R 1V 2J 2N 2R 2V)

    vpsrldq     ymmE, ymmA, 8     ; ymmE=(20 24 28 2C 01 05 09 0D 0G 0K 0O 0S 1G 1K 1O 1S
                                  ;       2G 2K 2O 2S 0H 0L 0P 0T -- -- -- -- -- -- -- --)
    vpslldq     ymmA, ymmA, 8     ; ymmA=(-- -- -- -- -- -- -- -- 00 04 08 0C 10 14 18 1C
                                  ;       20 24 28 2C 01 05 09 0D 0G 0K 0O 0S 1G 1K 1O 1S)

    vpunpckhbw  ymmA, ymmA, ymmD  ; ymmA=(00 02 04 06 08 0A 0C 0E 10 12 14 16 18 1A 1C 1E
                                  ;       0G 0I 0K 0M 0O 0Q 0S 0U 1G 1I 1K 1M 1O 1Q 1S 1U)
    vpslldq     ymmD, ymmD, 8     ; ymmD=(-- -- -- -- -- -- -- -- 11 15 19 1D 21 25 29 2D
                                  ;       02 06 0A 0E 12 16 1A 1E 1H 1L 1P 1T 2H 2L 2P 2T)

    vpunpcklbw  ymmE, ymmE, ymmG  ; ymmE=(20 22 24 26 28 2A 2C 2E 01 03 05 07 09 0B 0D 0F
                                  ;       2G 2I 2K 2M 2O 2Q 2S 2U 0H 0J 0L 0N 0P 0R 0T 0V)
    vpunpckhbw  ymmD, ymmD, ymmG  ; ymmD=(11 13 15 17 19 1B 1D 1F 21 23 25 27 29 2B 2D 2F
                                  ;       1H 1J 1L 1N 1P 1R 1T 1V 2H 2J 2L 2N 2P 2R 2T 2V)

    vpxor       xmmH, xmmH, xmmH

    vpunpckhbw  ymmC, ymmA, ymmH  ; ymmC=(10 12 14 16 18 1A 1C 1E 1G 1I 1K 1M 1O 1Q 1S 1U)
    vpunpcklbw  ymmA, ymmA, ymmH  ; ymmA=(00 02 04 06 08 0A 0C 0E 0G 0I 0K 0M 0O 0Q 0S 0U)

    vpunpckhbw  ymmB, ymmE, ymmH  ; ymmB=(01 03 05 07 09 0B 0D 0F 0H 0J 0L 0N 0P 0R 0T 0V)
    vpunpcklbw  ymmE, ymmE, ymmH  ; ymmE=(20 22 24 26 28 2A 2C 2E 2G 2I 2K 2M 2O 2Q 2S 2U)

    vpunpckhbw  ymmF, ymmD, ymmH  ; ymmF=(21 23 25 27 29 2B 2D 2F 2H 2J 2L 2N 2P 2R 2T 2V)
    vpunpcklbw  ymmD, ymmD, ymmH  ; ymmD=(11 13 15 17 19 1B 1D 1F 1H 1J 1L 1N 1P 1R 1T 1V)

%else  ; RGB_PIXELSIZE == 4 ; -----------

.column_ld1:
    test        cl, SIZEOF_XMMWORD/16
    jz          short .column_ld2
    sub         ecx, byte SIZEOF_XMMWORD/16
    vmovd       xmmA, XMM_DWORD [rsi+rcx*RGB_PIXELSIZE]
.column_ld2:
    test        cl, SIZEOF_XMMWORD/8
    jz          short .column_ld4
    sub         ecx, byte SIZEOF_XMMWORD/8
    vmovq       xmmF, XMM_MMWORD [rsi+rcx*RGB_PIXELSIZE]
    vpslldq     xmmA, xmmA, SIZEOF_MMWORD
    vpor        xmmA, xmmA, xmmF
.column_ld4:
    test        cl, SIZEOF_XMMWORD/4
    jz          short .column_ld8
    sub         ecx, byte SIZEOF_XMMWORD/4
    vmovdqu     xmmF, XMMWORD [rsi+rcx*RGB_PIXELSIZE]
    vinserti128 ymmA, ymmF, xmmA, 0x1
.column_ld8:
    test        cl, SIZEOF_XMMWORD/2
    jz          short .column_ld16
    sub         ecx, byte SIZEOF_XMMWORD/2
    vmovdqa     ymmF, ymmA
    vmovdqu     ymmA, YMMWORD [rsi+rcx*RGB_PIXELSIZE]
.column_ld16:
    test        cl, SIZEOF_XMMWORD
    mov         ecx, SIZEOF_YMMWORD
    jz          short .rgb_ycc_cnv
    vmovdqa     ymmE, ymmA
    vmovdqa     ymmH, ymmF
    vmovdqu     ymmA, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     ymmF, YMMWORD [rsi+1*SIZEOF_YMMWORD]
    jmp         short .rgb_ycc_cnv

align 16
.columnloop:
    vmovdqu     ymmA, YMMWORD [rsi+0*SIZEOF_YMMWORD]
    vmovdqu     ymmF, YMMWORD [rsi+1*SIZEOF_YMMWORD]
    vmovdqu     ymmE, YMMWORD [rsi+2*SIZEOF_YMMWORD]
    vmovdqu     ymmH, YMMWORD [rsi+3*SIZEOF_YMMWORD]

align 16
.rgb_ycc_cnv:
    ; ymmA=(00 10 20 30 01 11 21 31 02 12 22 32 03 13 23 33
    ;       04 14 24 34 05 15 25 35 06 16 26 36 07 17 27 37)
    ; ymmF=(08 18 28 38 09 19 29 39 0A 1A 2A 3A 0B 1B 2B 3B
    ;       0C 1C 2C 3C 0D 1D 2D 3D 0E 1E 2E 3E 0F 1F 2F 3F)
    ; ymmE=(0G 1G 2G 3G 0H 1H 2H 3H 0I 1I 2I 3I 0J 1J 2J 3J
    ;       0K 1K 2K 3K 0L 1L 2L 3L 0M 1M 2M 3M 0N 1N 2N 3N)
    ; ymmH=(0O 1O 2O 3O 0P 1P 2P 3P 0Q 1Q 2Q 3Q 0R 1R 2R 3R
    ;       0S 1S 2S 3S 0T 1T 2T 3T 0U 1U 2U 3U 0V 1V 2V 3V)

    vperm2i128  ymmE, ymmA, ymmE, 0x31  ; ymmE=(04 14 24 34 05 15 25 35 06 16 26 36 07 17 27 37
                                        ;       0K 1K 2K 3K 0L 1L 2L 3L 0M 1M 2M 3M 0N 1N 2N 3N)
    vinserti128 ymmA, ymmA, xmmE, 1     ; ymmA=(00 10 20 30 01 11 21 31 02 12 22 32 03 13 23 33
                                        ;       0G 1G 2G 3G 0H 1H 2H 3H 0I 1I 2I 3I 0J 1J 2J 3J)

    vperm2i128  ymmH, ymmF, ymmH, 0x31  ; ymmH=(0C 1C 2C 3C 0D 1D 2D 3D 0E 1E 2E 3E 0F 1F 2F 3F
                                        ;       0S 1S 2S 3S 0T 1T 2T 3T 0U 1U 2U 3U 0V 1V 2V 3V)
    vinserti128 ymmF, ymmF, xmmH, 1     ; ymmF=(08 18 28 38 09 19 29 39 0A 1A 2A 3A 0B 1B 2B 3B
                                        ;       0O 1O 2O 3O 0P 1P 2P 3P 0Q 1Q 2Q 3Q 0R 1R 2R 3R)

    vpunpckhbw  ymmD, ymmA, ymmE      ; ymmD=(02 06 12 16 22 26 32 36 03 07 13 17 23 27 33 37
                                      ;       0I 0M 1I 1M 2I 2M 3I 3M 0J 0N 1J 1N 2J 2N 3J 3N)
    vpunpcklbw  ymmA, ymmA, ymmE      ; ymmA=(00 04 10 14 20 24 30 34 01 05 11 15 21 25 31 35
                                      ;       0G 0K 1G 1K 2G 2K 3G 3K 0H 0L 1H 1L 2H 2L 3H 3L)

    vpunpckhbw  ymmC, ymmF, ymmH      ; ymmC=(0A 0E 1A 1E 2A 2E 3A 3E 0B 0F 1B 1F 2B 2F 3B 3F
                                      ;       0Q 0U 1Q 1U 2Q 2U 3Q 3U 0R 0V 1R 1V 2R 2V 3R 3V)
    vpunpcklbw  ymmF, ymmF, ymmH      ; ymmF=(08 0C 18 1C 28 2C 38 3C 09 0D 19 1D 29 2D 39 3D
                                      ;       0O 0S 1O 1S 2O 2S 3O 3S 0P 0T 1P 1T 2P 2T 3P 3T)

    vpunpckhwd  ymmB, ymmA, ymmF      ; ymmB=(01 05 09 0D 11 15 19 1D 21 25 29 2D 31 35 39 3D
                                      ;       0H 0L 0P 0T 1H 1L 1P 1T 2H 2L 2P 2T 3H 3L 3P 3T)
    vpunpcklwd  ymmA, ymmA, ymmF      ; ymmA=(00 04 08 0C 10 14 18 1C 20 24 28 2C 30 34 38 3C
                                      ;       0G 0K 0O 0S 1G 1K 1O 1S 2G 2K 2O 2S 3G 3K 3O 3S)

    vpunpckhwd  ymmG, ymmD, ymmC      ; ymmG=(03 07 0B 0F 13 17 1B 1F 23 27 2B 2F 33 37 3B 3F
                                      ;       0J 0N 0R 0V 1J 1N 1R 1V 2J 2N 2R 2V 3J 3N 3R 3V)
    vpunpcklwd  ymmD, ymmD, ymmC      ; ymmD=(02 06 0A 0E 12 16 1A 1E 22 26 2A 2E 32 36 3A 3E
                                      ;       0I 0M 0Q 0U 1I 1M 1Q 1U 2I 2M 2Q 2U 3I 3M 3Q 3U)

    vpunpckhbw  ymmE, ymmA, ymmD      ; ymmE=(20 22 24 26 28 2A 2C 2E 30 32 34 36 38 3A 3C 3E
                                      ;       2G 2I 2K 2M 2O 2Q 2S 2U 3G 3I 3K 3M 3O 3Q 3S 3U)
    vpunpcklbw  ymmA, ymmA, ymmD      ; ymmA=(00 02 04 06 08 0A 0C 0E 10 12 14 16 18 1A 1C 1E
                                      ;       0G 0I 0K 0M 0O 0Q 0S 0U 1G 1I 1K 1M 1O 1Q 1S 1U)

    vpunpckhbw  ymmH, ymmB, ymmG      ; ymmH=(21 23 25 27 29 2B 2D 2F 31 33 35 37 39 3B 3D 3F
                                      ;       2H 2J 2L 2N 2P 2R 2T 2V 3H 3J 3L 3N 3P 3R 3T 3V)
    vpunpcklbw  ymmB, ymmB, ymmG      ; ymmB=(01 03 05 07 09 0B 0D 0F 11 13 15 17 19 1B 1D 1F
                                      ;       0H 0J 0L 0N 0P 0R 0T 0V 1H 1J 1L 1N 1P 1R 1T 1V)

    vpxor       xmmF, xmmF, xmmF

    vpunpckhbw  ymmC, ymmA, ymmF      ; ymmC=(10 12 14 16 18 1A 1C 1E 1G 1I 1K 1M 1O 1Q 1S 1U)
    vpunpcklbw  ymmA, ymmA, ymmF      ; ymmA=(00 02 04 06 08 0A 0C 0E 0G 0I 0K 0M 0O 0Q 0S 0U)

    vpunpckhbw  ymmD, ymmB, ymmF      ; ymmD=(11 13 15 17 19 1B 1D 1F 1H 1J 1L 1N 1P 1R 1T 1V)
    vpunpcklbw  ymmB, ymmB, ymmF      ; ymmB=(01 03 05 07 09 0B 0D 0F 0H 0J 0L 0N 0P 0R 0T 0V)

    vpunpckhbw  ymmG, ymmE, ymmF      ; ymmG=(30 32 34 36 38 3A 3C 3E 3G 3I 3K 3M 3O 3Q 3S 3U)
    vpunpcklbw  ymmE, ymmE, ymmF      ; ymmE=(20 22 24 26 28 2A 2C 2E 2G 2I 2K 2M 2O 2Q 2S 2U)

    vpunpcklbw  ymmF, ymmF, ymmH
    vpunpckhbw  ymmH, ymmH, ymmH
    vpsrlw      ymmF, ymmF, BYTE_BIT  ; ymmF=(21 23 25 27 29 2B 2D 2F 2H 2J 2L 2N 2P 2R 2T 2V)
    vpsrlw      ymmH, ymmH, BYTE_BIT  ; ymmH=(31 33 35 37 39 3B 3D 3F 3H 3J 3L 3N 3P 3R 3T 3V)

%endif  ; RGB_PIXELSIZE ; ---------------

    ; ymm0=R(02468ACEGIKMOQSU)=RE, ymm2=G(02468ACEGIKMOQSU)=GE, ymm4=B(02468ACEGIKMOQSU)=BE
    ; ymm1=R(13579BDFHJLNPRTV)=RO, ymm3=G(13579BDFHJLNPRTV)=GO, ymm5=B(13579BDFHJLNPRTV)=BO

    ; (Original)
    ; Y  =  0.29900 * R + 0.58700 * G + 0.11400 * B
    ; Cb = -0.16874 * R - 0.33126 * G + 0.50000 * B + CENTERJSAMPLE
    ; Cr =  0.50000 * R - 0.41869 * G - 0.08131 * B + CENTERJSAMPLE
    ;
    ; (This implementation)
    ; Y  =  0.29900 * R + 0.33700 * G + 0.11400 * B + 0.25000 * G
    ; Cb = -0.16874 * R - 0.33126 * G + 0.50000 * B + CENTERJSAMPLE
    ; Cr =  0.50000 * R - 0.41869 * G - 0.08131 * B + CENTERJSAMPLE

    vmovdqa     YMMWORD [wk(0)], ymm0   ; wk(0)=RE
    vmovdqa     YMMWORD [wk(1)], ymm1   ; wk(1)=RO
    vmovdqa     YMMWORD [wk(2)], ymm4   ; wk(2)=BE
    vmovdqa     YMMWORD [wk(3)], ymm5   ; wk(3)=BO

    vpunpckhwd  ymm6, ymm1, ymm3
    vpunpcklwd  ymm8, ymm1, ymm3
    vpmaddwd    ymm7, ymm8, ymm13  ; ymm7=ROL*-FIX(0.168)+GOL*-FIX(0.331)
    vpmaddwd    ymm4, ymm6, ymm13  ; ymm4=ROH*-FIX(0.168)+GOH*-FIX(0.331)
    vpmaddwd    ymm8, ymm8, ymm15  ; ymm8=ROL*FIX(0.299)+GOL*FIX(0.337)
    vpmaddwd    ymm9, ymm6, ymm15  ; ymm9=ROH*FIX(0.299)+GOH*FIX(0.337)


    vpxor       xmm6, xmm6, xmm6
    vpunpcklwd  ymm1, ymm6, ymm5        ; ymm1=BOL
    vpunpckhwd  ymm6, ymm6, ymm5        ; ymm6=BOH
    vpsrld      ymm1, ymm1, 1           ; ymm1=BOL*FIX(0.500)
    vpsrld      ymm6, ymm6, 1           ; ymm6=BOH*FIX(0.500)


    vpaddd      ymm7, ymm7, ymm1
    vpaddd      ymm4, ymm4, ymm6
    vpaddd      ymm7, ymm7, ymm11	;ymm11=PD_ONEHALFM1_CJ
    vpaddd      ymm4, ymm4, ymm11
    vpsrld      ymm7, ymm7, SCALEBITS   ; ymm7=CbOL
    vpsrld      ymm4, ymm4, SCALEBITS   ; ymm4=CbOH
    vpackssdw   ymm7, ymm7, ymm4        ; ymm7=CbO

    vmovdqa     ymm1, YMMWORD [wk(2)]   ; ymm1=BE

    vpunpckhwd  ymm6, ymm0, ymm2
    vpunpcklwd  ymm0, ymm0, ymm2
    vpmaddwd    ymm5, ymm0, ymm13  ; ymm5=REL*-FIX(0.168)+GEL*-FIX(0.331)
    vpmaddwd    ymm4, ymm6, ymm13  ; ymm4=REH*-FIX(0.168)+GEH*-FIX(0.331)
    vpmaddwd    ymm0, ymm0, ymm15  ; ymm0=REL*FIX(0.299)+GEL*FIX(0.337)
    vpmaddwd    ymm6, ymm6, ymm15  ; ymm6=REH*FIX(0.299)+GEH*FIX(0.337)

    vmovdqa     YMMWORD [wk(4)], ymm0   ; wk(6)=REL*FIX(0.299)+GEL*FIX(0.337)
    vmovdqa     YMMWORD [wk(5)], ymm6   ; wk(7)=REH*FIX(0.299)+GEH*FIX(0.337)

    vpxor       ymm6, ymm6, ymm6
    vpunpcklwd  ymm0, ymm6, ymm1        ; ymm0=BEL
    vpunpckhwd  ymm6, ymm6, ymm1        ; ymm6=BEH
    vpsrld      ymm0, ymm0, 1           ; ymm0=BEL*FIX(0.500)
    vpsrld      ymm6, ymm6, 1           ; ymm6=BEH*FIX(0.500)


    vpaddd      ymm5, ymm5, ymm0
    vpaddd      ymm4, ymm4, ymm6
    vpaddd      ymm5, ymm5, ymm11	; ymm11=PD_ONEHALFM1_CJ
    vpaddd      ymm4, ymm4, ymm11
    vpsrld      ymm5, ymm5, SCALEBITS   ; ymm5=CbEL
    vpsrld      ymm4, ymm4, SCALEBITS   ; ymm4=CbEH
    vpackssdw   ymm5, ymm5, ymm4        ; ymm5=CbE

    vpsllw      ymm7, ymm7, BYTE_BIT
    vpor        ymm5, ymm5, ymm7        ; ymm5=Cb
    vmovdqu     YMMWORD [rbx], ymm5     ; Save Cb

    vmovdqa     ymm0, YMMWORD [wk(3)]   ; ymm0=BO
    vmovdqa     ymm6, YMMWORD [wk(2)]   ; ymm6=BE
    vmovdqa     ymm1, YMMWORD [wk(1)]   ; ymm1=RO

    vpunpckhwd  ymm4, ymm0, ymm3
    vpunpcklwd  ymm0, ymm0, ymm3
    vpmaddwd    ymm7, ymm0, ymm12  ; ymm7=BOL*-FIX(0.081)+GOL*-FIX(0.418)
    vpmaddwd    ymm5, ymm4, ymm12  ; ymm5=BOH*-FIX(0.081)+GOH*-FIX(0.418)
    vpmaddwd    ymm0, ymm0, ymm14  ; ymm0=BOL*FIX(0.114)+GOL*FIX(0.250)
    vpmaddwd    ymm4, ymm4, ymm14  ; ymm4=BOH*FIX(0.114)+GOH*FIX(0.250)


    vpaddd      ymm0, ymm0, ymm8
    vpaddd      ymm4, ymm4, ymm9
    vpaddd      ymm0, ymm0, ymm10	; ymm10=PD_ONEHALF
    vpaddd      ymm4, ymm4, ymm10
    vpsrld      ymm0, ymm0, SCALEBITS   ; ymm0=YOL
    vpsrld      ymm4, ymm4, SCALEBITS   ; ymm4=YOH
    vpackssdw   ymm0, ymm0, ymm4        ; ymm0=YO

    vpxor       xmm4, xmm4, xmm4
    vpunpcklwd  ymm3, ymm4, ymm1        ; ymm3=ROL
    vpunpckhwd  ymm4, ymm4, ymm1        ; ymm4=ROH
    vpsrld      ymm3, ymm3, 1           ; ymm3=ROL*FIX(0.500)
    vpsrld      ymm4, ymm4, 1           ; ymm4=ROH*FIX(0.500)


    vpaddd      ymm7, ymm7, ymm3
    vpaddd      ymm5, ymm5, ymm4
    vpaddd      ymm7, ymm7, ymm11	; ymm11=PD_ONEHALFM1_CJ
    vpaddd      ymm5, ymm5, ymm11
    vpsrld      ymm7, ymm7, SCALEBITS   ; ymm7=CrOL
    vpsrld      ymm5, ymm5, SCALEBITS   ; ymm5=CrOH
    vpackssdw   ymm7, ymm7, ymm5        ; ymm7=CrO

    vmovdqa     ymm3, YMMWORD [wk(0)]   ; ymm3=RE

    vpunpckhwd  ymm4, ymm6, ymm2
    vpunpcklwd  ymm6, ymm6, ymm2
    vpmaddwd    ymm1, ymm6, ymm12  ; ymm1=BEL*-FIX(0.081)+GEL*-FIX(0.418)
    vpmaddwd    ymm5, ymm4, ymm12  ; ymm5=BEH*-FIX(0.081)+GEH*-FIX(0.418)
    vpmaddwd    ymm6, ymm6, ymm14  ; ymm6=BEL*FIX(0.114)+GEL*FIX(0.250)
    vpmaddwd    ymm4, ymm4, ymm14  ; ymm4=BEH*FIX(0.114)+GEH*FIX(0.250)


    vpaddd      ymm6, ymm6, YMMWORD [wk(4)]
    vpaddd      ymm4, ymm4, YMMWORD [wk(5)]
    vpaddd      ymm6, ymm6, ymm10
    vpaddd      ymm4, ymm4, ymm10
    vpsrld      ymm6, ymm6, SCALEBITS   ; ymm6=YEL
    vpsrld      ymm4, ymm4, SCALEBITS   ; ymm4=YEH
    vpackssdw   ymm6, ymm6, ymm4        ; ymm6=YE

    vpsllw      ymm0, ymm0, BYTE_BIT
    vpor        ymm6, ymm6, ymm0        ; ymm6=Y
    vmovdqu     YMMWORD [rdi], ymm6     ; Save Y

    vpxor       xmm4, xmm4, xmm4
    vpunpcklwd  ymm2, ymm4, ymm3        ; ymm2=REL
    vpunpckhwd  ymm4, ymm4, ymm3        ; ymm4=REH
    vpsrld      ymm2, ymm2, 1           ; ymm2=REL*FIX(0.500)
    vpsrld      ymm4, ymm4, 1           ; ymm4=REH*FIX(0.500)


    vpaddd      ymm1, ymm1, ymm2
    vpaddd      ymm5, ymm5, ymm4
    vpaddd      ymm1, ymm1, ymm11	; ymm11=PD_ONEHALFM1_CJ
    vpaddd      ymm5, ymm5, ymm11
    vpsrld      ymm1, ymm1, SCALEBITS   ; ymm1=CrEL
    vpsrld      ymm5, ymm5, SCALEBITS   ; ymm5=CrEH
    vpackssdw   ymm1, ymm1, ymm5        ; ymm1=CrE

    vpsllw      ymm7, ymm7, BYTE_BIT
    vpor        ymm1, ymm1, ymm7        ; ymm1=Cr
    vmovdqu     YMMWORD [rdx], ymm1     ; Save Cr

    sub         ecx, byte SIZEOF_YMMWORD
    add         rsi, RGB_PIXELSIZE*SIZEOF_YMMWORD  ; inptr
    add         rdi, byte SIZEOF_YMMWORD           ; outptr0
    add         rbx, byte SIZEOF_YMMWORD           ; outptr1
    add         rdx, byte SIZEOF_YMMWORD           ; outptr2
    cmp         ecx, byte SIZEOF_YMMWORD
    jae         near .columnloop
    test        ecx, ecx
    jnz         near .column_ld1

    pop         rdi
    pop         rbx
    pop         rdx

    add         r11, byte SIZEOF_JSAMPROW  ; input_buf
    add         rdi, byte SIZEOF_JSAMPROW
    add         rbx, byte SIZEOF_JSAMPROW
    add         rdx, byte SIZEOF_JSAMPROW
    sub         eax, 1                     ; num_rows
    jg          near .rowloop

.return:
    pop         rbx
    vzeroupper
    uncollect_args 5
    mov         rsp, rbp                ; rsp <- aligned rbp
    pop         rsp                     ; rsp <- original rbp
    pop         rbp
    ret

; For some reason, the OS X linker does not honor the request to align the
; segment unless we do this.
    align       32
