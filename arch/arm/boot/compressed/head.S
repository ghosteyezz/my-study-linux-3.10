/*
 *  linux/arch/arm/boot/compressed/head.S
 *
 *  Copyright (C) 1996-2002 Russell King
 *  Copyright (C) 2004 Hyok S. Choi (MPU support)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*SH START 130716-1*/
#include <linux/linkage.h>
#include <asm/assembler.h>

	.arch	armv7-a
	/*SH 타깃 CPU가 armv7-a임을 나타냄*/
/*
 * Debugging stuff
 *
 * Note that these macros must not contain any code which is not
 * 100% relocatable.  Any attempt to do so will result in a crash.
 * Please select one of the following when turning on debugging.
 */
#ifdef DEBUG

#if defined(CONFIG_DEBUG_ICEDCC)

#if defined(CONFIG_CPU_V6) || defined(CONFIG_CPU_V6K) || defined(CONFIG_CPU_V7)
		.macro	loadsp, rb, tmp
		/*SH .macro : 매크로함수 시작*/
		/*SH loadsp : 이름, rb,tmp : 매크로 인자*/
		.endm
		/*SH .endm : 매크로함수 끝*/
		.macro	writeb, ch, rb
		mcr	p14, 0, \ch, c0, c5, 0
		/*SH \ch : 인자 'ch'의 값을 사용*/
		.endm
#elif defined(CONFIG_CPU_XSCALE)
		.macro	loadsp, rb, tmp
		.endm
		.macro	writeb, ch, rb
		mcr	p14, 0, \ch, c8, c0, 0
		.endm
#else
		.macro	loadsp, rb, tmp
		.endm
		.macro	writeb, ch, rb
		mcr	p14, 0, \ch, c1, c0, 0
		.endm
#endif

#else

#include CONFIG_DEBUG_LL_INCLUDE

		.macro	writeb,	ch, rb
		senduart \ch, \rb
		.endm

#if defined(CONFIG_ARCH_SA1100)
		.macro	loadsp, rb, tmp
		mov	\rb, #0x80000000	@ physical base address
#ifdef CONFIG_DEBUG_LL_SER3
		add	\rb, \rb, #0x00050000	@ Ser3
#else
		add	\rb, \rb, #0x00010000	@ Ser1
#endif
		.endm
#elif defined(CONFIG_ARCH_S3C24XX)
		.macro loadsp, rb, tmp
		mov	\rb, #0x50000000
		add	\rb, \rb, #0x4000 * CONFIG_S3C_LOWLEVEL_UART_PORT
		.endm
#else
		.macro	loadsp,	rb, tmp
		addruart \rb, \tmp
		.endm
#endif
#endif
#endif

		.macro	kputc,val
		mov	r0, \val
		bl	putc
		.endm

		.macro	kphex,val,len
		mov	r0, \val
		mov	r1, #\len
		bl	phex
		.endm

		.macro	debug_reloc_start
#ifdef DEBUG
		kputc	#'\n'
		kphex	r6, 8		/* processor id */
		kputc	#':'
		kphex	r7, 8		/* architecture id */
#ifdef CONFIG_CPU_CP15
		kputc	#':'
		mrc	p15, 0, r0, c1, c0
		kphex	r0, 8		/* control reg */
#endif
		kputc	#'\n'
		kphex	r5, 8		/* decompressed kernel start */
		kputc	#'-'
		kphex	r9, 8		/* decompressed kernel end  */
		kputc	#'>'
		kphex	r4, 8		/* kernel execution address */
		kputc	#'\n'
#endif
		.endm

		.macro	debug_reloc_end
#ifdef DEBUG
		kphex	r5, 8		/* end of kernel */
		kputc	#'\n'
		mov	r0, r4
		bl	memdump		/* dump 256 bytes at start of kernel */
#endif
		.endm

/*SH 커널시작*/
/*SH 부트로더에게 초기 넘어오는 레지스터 값들*/
/*SH r0 = Must be 0*/
/*SH r1 = ARM Linux machine type*/
/*SH r2 = The physical address of the parameter list*/
/*SH 참조 : Vincent Sanders, Booting ARM Linux*/
/*SH zImage 관련 정보 - 이건 zImage를 이렇게 만들어야 된다는 우리들만의 약속~*/
/*SH zImage start + 0x24 = Magic number(0x016f2818)*/
/*SH zImage start + 0x28 = zImage start address(start)*/
/*SH zImage start + 0x2c = zImage end address(_edata)*/
		.section ".start", #alloc, #execinstr
		/*SH .section : ld에서 링크스크립터가 사용하는 Address section 정의*/
		/*SH ".start" : section 이름, #alloc : 할당가능, #execinstr : 실행가능*/
/*
 * sort out different calling conventions
 */
		.align
		/*SH .align : 다음에 올 코드의 정렬 간격을 나타냄. 기본값 2*/
	    /*SH ex>.align 3 : arm에서는 2^3 = 8 byte 간격으로 정렬*/
		/*SH               i386에서는 걍 3 byte 간격으로 정렬*/
		.arm				@ Always enter in ARM state
		/*SH .arm : arm(32bit) 명령어임을 나타냄으로 시작 */
start:
/*SH 라벨정의, start : 라벨이름*/
		.type	start,#function
		/*SH .type : 심볼(start:)의 타입을 정의*/
		/*SH start : 심볼이름, #function : 함수임을 나타냄*/
		.rept	7
		/*SH .rept # : .endr을 만나기 전까지의 명령어를 #번 반복하여 어셈블함*/
		mov	r0, r0
		/*SH r0=r0*/
		.endr
		/*SH .rept의 끝을 나타냄*/
   ARM(		mov	r0, r0		)
/*SH ARM()/THUMB() : arch/arm/include/asm/unified.h에 정의 */
		/*SH r0=r0*/
   ARM(		b	1f		)
			/*SH PC=라벨'1'의 주소값 : 라벨'1'의 주소값으로 무조건 분기*/
			/*SH 라벨명 뒤의 'f'의 의미 'frontword' 앞의 주소값의 라벨'1'을 찾아 분기*/
 THUMB(		adr	r12, BSYM(1f)	)
			/*SH FIXME r12=라벨'1'의 주소값 + 1 : +1을 해주는 이유는?*/
			/*SH BSYM() : arch/arm/include/asm/unified.h에 정의*/
 THUMB(		bx	r12		)
			/*SH FIXME PC=(라벨'1'의 주소값 + 1) : x의 의미를 정확히 모르겠음*/
			/*SH 동작설명으로는 (라벨'1'의 주소값 + 1)[0]의 값이 1이면 타겟은 Thumb, 0이면 타겟은 ARM*/

		.word	0x016f2818		@ Magic numbers to help the loader
		/*SH .word : 현재 Section(Address)에서 .align되 있는 만큼 크기로 alloc하여 값을 저장 한다*/
		.word	start			@ absolute load/run zImage address
		/*SH start : start 라벨을 뜻하며, 실제 0x00000000임*/
		.word	_edata			@ zImage end address
		/*SH _edata : arch/arm/kernel/vmlinux.lds.S에 정의되어있는 zImage의 끝 주소*/
 THUMB(		.thumb			)
		/*SH .thumb : thumb 명령어임을 나타냄으로 시작*/
/*SH END 130716-1*/
/*SH START 130718-1*/
1:
/*SH 라벨정의, 1 : 라벨이름*/
		mrs	r9, cpsr
		/*SH r9 = cpsr*/

#ifdef CONFIG_ARM_VIRT_EXT
		bl	__hyp_stub_install	@ get into SVC mode, reversibly
		/*SH LR = 'mov r7, r1'명령어의 주소값, PC=__hyp_stub_install의 주소값*/
		/*SH 즉, 돌아올 주소를 저장한 후, __hyp_stub_install로 분기한다*/
#endif
		mov	r7, r1			@ save architecture ID
		/*SH r7 = r1 : r1은 부트로더에서 할당 후 커널로 넘어옴*/
		mov	r8, r2			@ save atags pointer
		/*SH r8 = r2 : r2는 부트로더에서 할당 후 커널로 넘어옴*/

#ifndef __ARM_ARCH_2__
		/*
		 * Booting from Angel - need to enter SVC mode and disable
		 * FIQs/IRQs (numeric definitions from angel arm.h source).
		 * We only do this if we were in user mode on entry.
		 */
		mrs	r2, cpsr		@ get current mode
		/*SH r2 = cpsr*/
		tst	r2, #3			@ not user?
		/*SH r2 & 3 의 결과를 cpsr의 N,Z,C 플래그 셋*/
		bne	not_angel
		/*SH bne : b : 분기, ne : 같지 않음*/
		/*SH 같지 않다는 의미는 cpsr z flag를 체크하여 1이면 같음, 0이면 같지 않음으로 판정*/
		/*SH 현재 모드가 User모드가 아니면 not_angel로 점프*/
		mov	r0, #0x17		@ angel_SWIreason_EnterSVC
		/*SH r0 = 0x17*/
 ARM(		swi	0x123456	)	@ angel_SWI_ARM
			/*SH FIXME 관리자 호출 예외 : User 모드를 벗어나기 위한 방법?*/
 THUMB(		svc	0xab		)	@ angel_SWI_THUMB
			/*SH FIXME 관리자 호출 예외 : User 모드를 벗어나기 위한 방법?*/
not_angel:
		safe_svcmode_maskall r0
		msr	spsr_cxsf, r9		@ Save the CPU boot mode in
						@ SPSR
		/*SH 부팅당시의 cpsr의 값을 spsr에 저장한다*/
		/*SH spsr_cxsf = r9*/
		/*SH c : 제어 필드 마스크 바이트 PSR[7:0]*/
		/*SH x : 확장 필드 마스크 바이트 PSR[15:8]*/
		/*SH s : 상태 필드 마스크 바이트 PSR[23:16]*/
		/*SH f : 플래그 필드 마스크 바이트 PSR[31:24]*/
#else
		teqp	pc, #0x0c000003		@ turn off interrupts
#endif
/*SH END 130719-1*/
/*SH START 130723-1*/
		/*
		 * Note that some cache flushing and other stuff may
		 * be needed here - is there an Angel SWI call for this?
		 */

		/*
		 * some architecture specific code can be inserted
		 * by the linker here, but it should preserve r7, r8, and r9.
		 */

		.text

#ifdef CONFIG_AUTO_ZRELADDR
		@ determine final kernel image address
		mov	r4, pc
		/*SH r4 = pc*/
		and	r4, r4, #0xf8000000
		/*SH r4 &= 0xf8000000*/
		add	r4, r4, #TEXT_OFFSET
		/*SH r4 += 0x00008000*/
#else
		ldr	r4, =zreladdr
#endif
		/*SH 실재 zImage의 text영역의 시작주소값을 r4에 저장*/

		bl	cache_on
		/*SH bl : lr = 다음명령어의 주소 저장 후 분기*/

/*SH START 130725-2*/
restart:	adr	r0, LC0
			/*SH r0 = 라벨 'LC0'의 주소값*/
		ldmia	r0, {r1, r2, r3, r6, r10, r11, r12}
		/*SH FIXME r1 = object 'LC0'의 주소값??*/
		/*SH r2 = __bss_start*/
		/*SH r3 = _end*/
		/*SH r6 = _edata*/
		/*SH r10 = input_data_end - 4*/
		/*SH r11 = _got_start*/
		/*SH r12(ip) = _got_end*/
		ldr	sp, [r0, #28]
		/*SH sp(r13) = _user_stack_end*/
/*SH END 130725-2*/
/*SH START NEXT*/

		/*
		 * We might be running at a different address.  We need
		 * to fix up various pointers.
		 */
		sub	r0, r0, r1		@ calculate the delta offset
		add	r6, r6, r0		@ _edata
		add	r10, r10, r0		@ inflated kernel size location

		/*
		 * The kernel build system appends the size of the
		 * decompressed kernel at the end of the compressed data
		 * in little-endian form.
		 */
		ldrb	r9, [r10, #0]
		ldrb	lr, [r10, #1]
		orr	r9, r9, lr, lsl #8
		ldrb	lr, [r10, #2]
		ldrb	r10, [r10, #3]
		orr	r9, r9, lr, lsl #16
		orr	r9, r9, r10, lsl #24

#ifndef CONFIG_ZBOOT_ROM
		/* malloc space is above the relocated stack (64k max) */
		add	sp, sp, r0
		add	r10, sp, #0x10000
#else
		/*
		 * With ZBOOT_ROM the bss/stack is non relocatable,
		 * but someone could still run this code from RAM,
		 * in which case our reference is _edata.
		 */
		mov	r10, r6
#endif

		mov	r5, #0			@ init dtb size to 0
#ifdef CONFIG_ARM_APPENDED_DTB
/*
 *   r0  = delta
 *   r2  = BSS start
 *   r3  = BSS end
 *   r4  = final kernel address
 *   r5  = appended dtb size (still unknown)
 *   r6  = _edata
 *   r7  = architecture ID
 *   r8  = atags/device tree pointer
 *   r9  = size of decompressed image
 *   r10 = end of this image, including  bss/stack/malloc space if non XIP
 *   r11 = GOT start
 *   r12 = GOT end
 *   sp  = stack pointer
 *
 * if there are device trees (dtb) appended to zImage, advance r10 so that the
 * dtb data will get relocated along with the kernel if necessary.
 */

		ldr	lr, [r6, #0]
#ifndef __ARMEB__
		ldr	r1, =0xedfe0dd0		@ sig is 0xd00dfeed big endian
#else
		ldr	r1, =0xd00dfeed
#endif
		cmp	lr, r1
		bne	dtb_check_done		@ not found

#ifdef CONFIG_ARM_ATAG_DTB_COMPAT
		/*
		 * OK... Let's do some funky business here.
		 * If we do have a DTB appended to zImage, and we do have
		 * an ATAG list around, we want the later to be translated
		 * and folded into the former here.  To be on the safe side,
		 * let's temporarily move  the stack away into the malloc
		 * area.  No GOT fixup has occurred yet, but none of the
		 * code we're about to call uses any global variable.
		*/
		add	sp, sp, #0x10000
		stmfd	sp!, {r0-r3, ip, lr}
		mov	r0, r8
		mov	r1, r6
		sub	r2, sp, r6
		bl	atags_to_fdt

		/*
		 * If returned value is 1, there is no ATAG at the location
		 * pointed by r8.  Try the typical 0x100 offset from start
		 * of RAM and hope for the best.
		 */
		cmp	r0, #1
		sub	r0, r4, #TEXT_OFFSET
		add	r0, r0, #0x100
		mov	r1, r6
		sub	r2, sp, r6
		bleq	atags_to_fdt

		ldmfd	sp!, {r0-r3, ip, lr}
		sub	sp, sp, #0x10000
#endif

		mov	r8, r6			@ use the appended device tree

		/*
		 * Make sure that the DTB doesn't end up in the final
		 * kernel's .bss area. To do so, we adjust the decompressed
		 * kernel size to compensate if that .bss size is larger
		 * than the relocated code.
		 */
		ldr	r5, =_kernel_bss_size
		adr	r1, wont_overwrite
		sub	r1, r6, r1
		subs	r1, r5, r1
		addhi	r9, r9, r1

		/* Get the dtb's size */
		ldr	r5, [r6, #4]
#ifndef __ARMEB__
		/* convert r5 (dtb size) to little endian */
		eor	r1, r5, r5, ror #16
		bic	r1, r1, #0x00ff0000
		mov	r5, r5, ror #8
		eor	r5, r5, r1, lsr #8
#endif

		/* preserve 64-bit alignment */
		add	r5, r5, #7
		bic	r5, r5, #7

		/* relocate some pointers past the appended dtb */
		add	r6, r6, r5
		add	r10, r10, r5
		add	sp, sp, r5
dtb_check_done:
#endif

/*
 * Check to see if we will overwrite ourselves.
 *   r4  = final kernel address
 *   r9  = size of decompressed image
 *   r10 = end of this image, including  bss/stack/malloc space if non XIP
 * We basically want:
 *   r4 - 16k page directory >= r10 -> OK
 *   r4 + image length <= address of wont_overwrite -> OK
 */
		add	r10, r10, #16384
		cmp	r4, r10
		bhs	wont_overwrite
		add	r10, r4, r9
		adr	r9, wont_overwrite
		cmp	r10, r9
		bls	wont_overwrite

/*
 * Relocate ourselves past the end of the decompressed kernel.
 *   r6  = _edata
 *   r10 = end of the decompressed kernel
 * Because we always copy ahead, we need to do it from the end and go
 * backward in case the source and destination overlap.
 */
		/*
		 * Bump to the next 256-byte boundary with the size of
		 * the relocation code added. This avoids overwriting
		 * ourself when the offset is small.
		 */
		add	r10, r10, #((reloc_code_end - restart + 256) & ~255)
		bic	r10, r10, #255

		/* Get start of code we want to copy and align it down. */
		adr	r5, restart
		bic	r5, r5, #31

/* Relocate the hyp vector base if necessary */
#ifdef CONFIG_ARM_VIRT_EXT
		mrs	r0, spsr
		and	r0, r0, #MODE_MASK
		cmp	r0, #HYP_MODE
		bne	1f

		bl	__hyp_get_vectors
		sub	r0, r0, r5
		add	r0, r0, r10
		bl	__hyp_set_vectors
1:
#endif

		sub	r9, r6, r5		@ size to copy
		add	r9, r9, #31		@ rounded up to a multiple
		bic	r9, r9, #31		@ ... of 32 bytes
		add	r6, r9, r5
		add	r9, r9, r10

1:		ldmdb	r6!, {r0 - r3, r10 - r12, lr}
		cmp	r6, r5
		stmdb	r9!, {r0 - r3, r10 - r12, lr}
		bhi	1b

		/* Preserve offset to relocated code. */
		sub	r6, r9, r6

#ifndef CONFIG_ZBOOT_ROM
		/* cache_clean_flush may use the stack, so relocate it */
		add	sp, sp, r6
#endif

		bl	cache_clean_flush

		adr	r0, BSYM(restart)
		add	r0, r0, r6
		mov	pc, r0

wont_overwrite:
/*
 * If delta is zero, we are running at the address we were linked at.
 *   r0  = delta
 *   r2  = BSS start
 *   r3  = BSS end
 *   r4  = kernel execution address
 *   r5  = appended dtb size (0 if not present)
 *   r7  = architecture ID
 *   r8  = atags pointer
 *   r11 = GOT start
 *   r12 = GOT end
 *   sp  = stack pointer
 */
		orrs	r1, r0, r5
		beq	not_relocated

		add	r11, r11, r0
		add	r12, r12, r0

#ifndef CONFIG_ZBOOT_ROM
		/*
		 * If we're running fully PIC === CONFIG_ZBOOT_ROM = n,
		 * we need to fix up pointers into the BSS region.
		 * Note that the stack pointer has already been fixed up.
		 */
		add	r2, r2, r0
		add	r3, r3, r0

		/*
		 * Relocate all entries in the GOT table.
		 * Bump bss entries to _edata + dtb size
		 */
1:		ldr	r1, [r11, #0]		@ relocate entries in the GOT
		add	r1, r1, r0		@ This fixes up C references
		cmp	r1, r2			@ if entry >= bss_start &&
		cmphs	r3, r1			@       bss_end > entry
		addhi	r1, r1, r5		@    entry += dtb size
		str	r1, [r11], #4		@ next entry
		cmp	r11, r12
		blo	1b

		/* bump our bss pointers too */
		add	r2, r2, r5
		add	r3, r3, r5

#else

		/*
		 * Relocate entries in the GOT table.  We only relocate
		 * the entries that are outside the (relocated) BSS region.
		 */
1:		ldr	r1, [r11, #0]		@ relocate entries in the GOT
		cmp	r1, r2			@ entry < bss_start ||
		cmphs	r3, r1			@ _end < entry
		addlo	r1, r1, r0		@ table.  This fixes up the
		str	r1, [r11], #4		@ C references.
		cmp	r11, r12
		blo	1b
#endif

not_relocated:	mov	r0, #0
1:		str	r0, [r2], #4		@ clear bss
		str	r0, [r2], #4
		str	r0, [r2], #4
		str	r0, [r2], #4
		cmp	r2, r3
		blo	1b

/*
 * The C runtime environment should now be setup sufficiently.
 * Set up some pointers, and start decompressing.
 *   r4  = kernel execution address
 *   r7  = architecture ID
 *   r8  = atags pointer
 */
		mov	r0, r4
		mov	r1, sp			@ malloc space above stack
		add	r2, sp, #0x10000	@ 64k max
		mov	r3, r7
		bl	decompress_kernel
		bl	cache_clean_flush
		bl	cache_off
		mov	r1, r7			@ restore architecture number
		mov	r2, r8			@ restore atags pointer

#ifdef CONFIG_ARM_VIRT_EXT
		mrs	r0, spsr		@ Get saved CPU boot mode
		and	r0, r0, #MODE_MASK
		cmp	r0, #HYP_MODE		@ if not booted in HYP mode...
		bne	__enter_kernel		@ boot kernel directly

		adr	r12, .L__hyp_reentry_vectors_offset
		ldr	r0, [r12]
		add	r0, r0, r12

		bl	__hyp_set_vectors
		__HVC(0)			@ otherwise bounce to hyp mode

		b	.			@ should never be reached

		.align	2
.L__hyp_reentry_vectors_offset:	.long	__hyp_reentry_vectors - .
#else
		b	__enter_kernel
#endif

		.align	2
		.type	LC0, #object
LC0:		.word	LC0			@ r1
			/*SH FIXME object 'LC0'의 주소값?? */
		.word	__bss_start		@ r2
		/*SH FIXME vmlinux.lds.in의 __bss_start로 추정됨*/
		.word	_end			@ r3
		/*SH FIXME vmlinux.lds.in의 _end로 추정됨*/
		.word	_edata			@ r6
		/*SH FIXME vmlinux.lds.in의 _edata로 추정됨*/
		.word	input_data_end - 4	@ r10 (inflated size location)
		/*SH FIXME 이건 몰겠음*/
		/*SH 우짯든 여기에 압축을 해제한 zImage 코드 사이즈가 저장됨, 이건 약속~*/
		.word	_got_start		@ r11
		/*SH FIXME vmlinux.lds.in의 _got_start로 추정됨*/
		.word	_got_end		@ ip
		/*SH FIXME vmlinux.lds.in의 _got_end로 추정됨*/
		.word	.L_user_stack_end	@ sp
		/*SH 이 파일 맨 마지막에 정의되어 있음*/
		/*SH		.align*/
		/*SH		.section ".stack", "aw", %nobits*/
		/*SH .L_user_stack:	.space	4096*/
		/*SH .L_user_stack_end:*/
		.size	LC0, . - LC0

#ifdef CONFIG_ARCH_RPC
		.globl	params
params:		ldr	r0, =0x10000100		@ params_phys for RPC
		mov	pc, lr
		.ltorg
		.align
#endif

/*
 * Turn on the cache.  We need to setup some page tables so that we
 * can have both the I and D caches on.
 *
 * We place the page tables 16k down from the kernel execution address,
 * and we hope that nothing else is using it.  If we're using it, we
 * will go pop!
 *
 * On entry,
 *  r4 = kernel execution address
 *  r7 = architecture number
 *  r8 = atags pointer
 * On exit,
 *  r0, r1, r2, r3, r9, r10, r12 corrupted
 * This routine must preserve:
 *  r4, r7, r8
 */
		.align	5
		/*SH 2^5=32byte로 정렬*/
cache_on:	mov	r3, #8			@ cache_on function
			/*SH r3 = 8*/
		b	call_cache_fn

/*
 * Initialize the highest priority protection region, PR7
 * to cover all 32bit address and cacheable and bufferable.
 */
__armv4_mpu_cache_on:
		mov	r0, #0x3f		@ 4G, the whole
		mcr	p15, 0, r0, c6, c7, 0	@ PR7 Area Setting
		mcr 	p15, 0, r0, c6, c7, 1

		mov	r0, #0x80		@ PR7
		mcr	p15, 0, r0, c2, c0, 0	@ D-cache on
		mcr	p15, 0, r0, c2, c0, 1	@ I-cache on
		mcr	p15, 0, r0, c3, c0, 0	@ write-buffer on

		mov	r0, #0xc000
		mcr	p15, 0, r0, c5, c0, 1	@ I-access permission
		mcr	p15, 0, r0, c5, c0, 0	@ D-access permission

		mov	r0, #0
		mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
		mcr	p15, 0, r0, c7, c5, 0	@ flush(inval) I-Cache
		mcr	p15, 0, r0, c7, c6, 0	@ flush(inval) D-Cache
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
						@ ...I .... ..D. WC.M
		orr	r0, r0, #0x002d		@ .... .... ..1. 11.1
		orr	r0, r0, #0x1000		@ ...1 .... .... ....

		mcr	p15, 0, r0, c1, c0, 0	@ write control reg

		mov	r0, #0
		mcr	p15, 0, r0, c7, c5, 0	@ flush(inval) I-Cache
		mcr	p15, 0, r0, c7, c6, 0	@ flush(inval) D-Cache
		mov	pc, lr

__armv3_mpu_cache_on:
		mov	r0, #0x3f		@ 4G, the whole
		mcr	p15, 0, r0, c6, c7, 0	@ PR7 Area Setting

		mov	r0, #0x80		@ PR7
		mcr	p15, 0, r0, c2, c0, 0	@ cache on
		mcr	p15, 0, r0, c3, c0, 0	@ write-buffer on

		mov	r0, #0xc000
		mcr	p15, 0, r0, c5, c0, 0	@ access permission

		mov	r0, #0
		mcr	p15, 0, r0, c7, c0, 0	@ invalidate whole cache v3
		/*
		 * ?? ARMv3 MMU does not allow reading the control register,
		 * does this really work on ARMv3 MPU?
		 */
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
						@ .... .... .... WC.M
		orr	r0, r0, #0x000d		@ .... .... .... 11.1
		/* ?? this overwrites the value constructed above? */
		mov	r0, #0
		mcr	p15, 0, r0, c1, c0, 0	@ write control reg

		/* ?? invalidate for the second time? */
		mcr	p15, 0, r0, c7, c0, 0	@ invalidate whole cache v3
		mov	pc, lr

#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
#define CB_BITS 0x08
#else
#define CB_BITS 0x0c
/*SH exynos에서 정의 되어 있지 않음으로 0x0c*/
#endif

__setup_mmu:	sub	r3, r4, #16384		@ Page directory size
				/*SH r3 = zImage 시작주소값 - 0x4000(16384)*/
				/*SH Page directory 공간확보*/
		bic	r3, r3, #0xff		@ Align the pointer
		bic	r3, r3, #0x3f00
		/*SH 0x4000이하의 주소값 클리어*/
/*
 * Initialise the page tables, turning on the cacheable and bufferable
 * bits for the RAM area only.
 */
/*SH RAM영역만 cacheable, bufferable하게 설정*/
		mov	r0, r3
		/*SH r0 = Page directory 시작주소*/
		mov	r9, r0, lsr #18
		/*SH r9 = (Page directory 시작주소 >> 18)*/
		mov	r9, r9, lsl #18		@ start of RAM
		/*SH r9 = ((Page directory 시작주소 >> 18) << 18)*/
		/*SH r9 = Page directory 시작주소 에서 하위 18bit를 클리어*/
		/*SH r9 = 주석에 의하면 이것이 RAM영역의 시작주소*/
		/*SH FIXME 왜 이것이 RAM영역의 시작주소?*/
		add	r10, r9, #0x10000000	@ a reasonable RAM size
		/*SH r10 = RAM영역의 시작주소 + 0x10000000(256MB)*/
/*SH END 130724-1*/
/*SH START 130724-2*/
		/*SH translation table first-level descriptor의 포멧을 정해줌*/
		mov	r1, #0x12		@ XN|U + section mapping
		/*SH [4] = 1 : XN(Execute Never) = , [1:0] = 2(0b10):Section*/
		/*SH r1 = 0x00000012*/
		orr	r1, r1, #3 << 10	@ AP=11
		/*SH [15:14] = 3(0b11) : AP(Access Permission) = Read only, at any privileage level*//
		/*SH  A.R.M B3.5.1(1325 page)참고*/
		/*SH r1 = 0x00000012 | 0x00000c00 = 0x00000c12*/
		add	r2, r3, #16384
		/*SH r2 = page directory 시작주소 + 0x4000(16384)*/
		/*SH 다음 루프의 종료조건*/
1:		cmp	r1, r9				@ if virt > start of RAM
		cmphs	r10, r1			@   && end of RAM > virt
		bic	r1, r1, #0x1c		@ clear XN|U + C + B
		orrlo	r1, r1, #0x10	@ Set XN|U for non-RAM
		orrhs	r1, r1, r6		@ set RAM section settings
		str	r1, [r0], #4		@ 1:1 mapping
		add	r1, r1, #1048576
		teq	r0, r2
		bne	1b
/*SH c 스타일 루프로 표시하겠음*/
/*SH RAM영역 시작주소를 0x40000000으로 가정함*/
/*SH 그러므로 RAM영역 끝주소가 0x50000000으로 가정됨*/
/*SH for( r0 = 0x40004000, r1 = 0x00000c12, r2 = 0x40008000; r0 < r2; r0 += 0x4, r1 += 0x100000 ){*/
/*SH	r1 = r1 & ~(0x1c);										//clear XN|U + C + B*/
/*SH	if( r1 > 0x40000000 && r1 < 0x50000000 ) r1 = r1 | 0xE;	//set RAM section settings*/
/*SH	else r1 = r1 | 0x10;									//Set XN|U for non-RAM*/
/*SH	*r0 = r1;*/
/*SH }*/
/*SH 위 루프의 r1 값은,*/
/*SH Short-descriptor translation table first-level descriptor formats 중 Section format으로 셋팅*/
/*SH Section format*/
/*SH |31:30:29:28|27:26:25:24|23:22:21:20|19:18:17:16|15:14:13:12|11:10:09:08|07:06:05:04|03:02:01:00|*/
/*SH |                                   |  |  |  |  |  |        |     |  |           |  |  |  |  |  |*/
/*SH | Section base address              |NS| 0|nG| S|AP|   TEX  |  AP |xx|   Domain  |XN| C| B| 1| 0|*/
/*SH |                                   |  |  |  |  |  |        |     |  |           |  |  |  |  |  |*/
/*SH | 8: 4: 2: 0| 8: 4: 2: 0| 8: 4: 2: 0| 8: 4: 2: 0| 8: 4: 2: 0| 8: 4: 2: 0| 8: 4: 2: 0| 8: 4: 2: 0|*/
/*SH  A.R.M B3.5.1(1325 page)참고*/
/*SH 0xE의 0xC(0b1100) 부분 : 0b1000 가 C(cacheable), 0b100 가 B(bufferable)의 의미, RAM영역(0x40000000~0x50000000은 이렇게 셋팅*/
/*SH 0x10(0b10000) 부분 : 0b10000 가 EN(Execute Never) 의미, RAM 이외 영역은 이렇게 셋팅됨*/
/*SH END 130724-2*/
/*SH START 130725-1*/
/*
 * If ever we are running from Flash, then we surely want the cache
 * to be enabled also for our execution instance...  We map 2MB of it
 * so there is no map overlap problem for up to 1 MB compressed kernel.
 * If the execution is in RAM then we would only be duplicating the above.
 */
/*SH 만약 플래시에서 실행된다면, 우리는 우리의 실행인스턴스를 위한 캐시를 활성화 시키기를 원한다.*/
/*SH 우리는 1MB이상의 압축된 커널 덮어쓰기 문제가 없는 2MB의 Map이 있다.???(해석FIXME)*/
/*SH 만약 RAM에서 실행된다면 위 코드로 Write된 Section discriptor를 덮어 쓸것이다.*/
		orr	r1, r6, #0x04		@ ensure B is set for this
		orr	r1, r1, #3 << 10
		/*SH 위 Section discriptor format참고하여 C[3] = 0b1, AP[11:10] = 0b11을 셋한다*/
		mov	r2, pc
		mov	r2, r2, lsr #20
		orr	r1, r1, r2, lsl #20
		add	r0, r3, r2, lsl #2
		/*SH Page section을 저장할 주소를 구한다.*/
		/*SH FIXME 왜 이 주소가 되는지?*/
		str	r1, [r0], #4
		add	r1, r1, #1048576
		str	r1, [r0]
		/*SH Section discriptor를 Write한다(2page=2MB).*/
		mov	pc, lr
		/*SH return*/
ENDPROC(__setup_mmu)

@ Enable unaligned access on v6, to allow better code generation
@ for the decompressor C code:
__armv6_mmu_cache_on:
		mrc	p15, 0, r0, c1, c0, 0	@ read SCTLR
		bic	r0, r0, #2		@ A (no unaligned access fault)
		orr	r0, r0, #1 << 22	@ U (v6 unaligned access model)
		mcr	p15, 0, r0, c1, c0, 0	@ write SCTLR
		b	__armv4_mmu_cache_on

__arm926ejs_mmu_cache_on:
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
		mov	r0, #4			@ put dcache in WT mode
		mcr	p15, 7, r0, c15, c0, 0
#endif

__armv4_mmu_cache_on:
		mov	r12, lr
#ifdef CONFIG_MMU
		mov	r6, #CB_BITS | 0x12	@ U
		bl	__setup_mmu
		mov	r0, #0
		mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
		mcr	p15, 0, r0, c8, c7, 0	@ flush I,D TLBs
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
		orr	r0, r0, #0x5000		@ I-cache enable, RR cache replacement
		orr	r0, r0, #0x0030
#ifdef CONFIG_CPU_ENDIAN_BE8
		orr	r0, r0, #1 << 25	@ big-endian page tables
#endif
		bl	__common_mmu_cache_on
		mov	r0, #0
		mcr	p15, 0, r0, c8, c7, 0	@ flush I,D TLBs
#endif
		mov	pc, r12

__armv7_mmu_cache_on:
		mov	r12, lr
		/*SH r12 = lr :돌아갈 복귀 주소를 r12에 저장*/
#ifdef CONFIG_MMU
		mrc	p15, 0, r11, c0, c1, 4	@ read ID_MMFR0
		/*SH r11 = ID_MMFR0( Memory Model Feature Register 0, VMSA*/
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.89(1620 page)*/
		tst	r11, #0xf		@ VMSA
		/*SH ID_MMFR0[3:0] : VMSA support 를 검사*/
		movne	r6, #CB_BITS | 0x02	@ !XN
		/*SH ne : 같지않음 : Z bit가 1이 아님*/
		/*SH ID_MMFR0[3:0] : VMSA support 가 0b0000 = Not supported 가 아니라면, r6 = 0x0c | 0x02*/
/*SH END 130723-1*/
/*SH START 130724-1*/
		blne	__setup_mmu
		/*SH 같지 않다면, lr = mov r0, #0 명령어 주소값 저장후 __setup_mmu로 분기 */
		mov	r0, #0
		/*SH r0 = 0*/
		mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.2.5(1749 page) Data and intruction barrier operations, VMSM에서*/
		/*SH CP15DSB, Data Synchronization Barrier operation이다.*/
		/*SH FIXME 이 명령의 의도는?*/
		tst	r11, #0xf		@ VMSA
		/*SH ID_MMFR0[3:0] : VMSA support 를 검사*/
		mcrne	p15, 0, r0, c8, c7, 0	@ flush I,D TLBs
		/*SH ID_MMFR0[3:0] : VMSA support 가 0b0000 = Not supported 가 아니라면 실행*/
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.135(1716 page) TLBIALL, TLB Invalidate All, VMSM only*/
		/*SH TLB를 모두 무효화 시킨다.*/
#endif
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.130(1705 page) SCTLR, System Control Register*/
		/*SH r0 = SCTLR*/
		bic	r0, r0, #1 << 28	@ clear SCTLR.TRE
		orr	r0, r0, #0x5000		@ I-cache enable, RR cache replacement
		orr	r0, r0, #0x003c		@ write buffer
		bic	r0, r0, #2		@ A (no unaligned access fault)
		orr	r0, r0, #1 << 22	@ U (v6 unaligned access model)
						@ (needed for ARM1176)
		/*SH r0 = 0b|xxx0|xxxx|x1xx|xxxx|x1x1|xxxx|xx11|110x|*/
#ifdef CONFIG_MMU
#ifdef CONFIG_CPU_ENDIAN_BE8
		orr	r0, r0, #1 << 25	@ big-endian page tables
#endif
		mrcne   p15, 0, r6, c2, c0, 2   @ read ttb control reg
		/*SH ID_MMFR0[3:0] : VMSA support 가 0b0000 = Not supported 가 아니라면 실행*/
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.153(1721 page) TTBCR, Translation Table Base Control Register, VMSA*/
		/*SH r6 = TTBCR*/
		orrne	r0, r0, #1		@ MMU enabled
		/*SH r0 = 0b|xxx0|xxxx|x1xx|xxxx|x1x1|xxxx|xx11|1101|*/
		movne	r1, #0xfffffffd		@ domain 0 = client
		/*SH r1 = 0xfffffffd*/
		bic     r6, r6, #1 << 31        @ 32-bit translation system
		bic     r6, r6, #3 << 0         @ use only ttbr0
		/*SH r6 = 0b|0xxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xx00|*/
		mcrne	p15, 0, r3, c2, c0, 0	@ load page table pointer
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.154(1726 page) TTBR0, Translation Table Base Register 0, VMSA*/
		/*SH TTBR0 = page directory의 시작주소(r3,가정대로는 0x40004000임)*/
		mcrne	p15, 0, r1, c3, c0, 0	@ load domain access control
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.43(1558 page) DACR, Domain Access Control Register, VMSA*/
		/*SH DACR = 0xfffffffd*/
		/*SH D15~D1 = Manager, D0 = Client*/
		mcrne   p15, 0, r6, c2, c0, 2   @ load ttb control
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.153(1721 page) TTBCR, Translation Table Base Control Register, VMSA*/
		/*SH TTBCR = 원래 TTBCR에 0b|0xxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xx00|*/
		/*SH Short-descriptor, TTBR0만 사용하겠다는 뜻*/
#endif
		mcr	p15, 0, r0, c7, c5, 4	@ ISB
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.39(1550 page) CP15ISB, CP15 Instruction Synchronization Barrier operation,VMSA*/
		/*SH r0값은 무의미, FIXME Instruction Synchronization Barrier operation 이 수행된다?*/
		mcr	p15, 0, r0, c1, c0, 0	@ load control register
		/*SH A.R.M - B3.17.2(1481 page) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.130(1705 page) SCTLR, System Control Register*/
		/*SH SCTLR = 원래 SCTLR에 0b|xxx0|xxxx|x1xx|xxxx|x1x1|xxxx|xx11|1101|*/
		/*SH TRE = 0, I-cache = 1, RR cache replacement, write buffer, A = 0, U = 1, MMU = 1*/
		mrc	p15, 0, r0, c1, c0, 0	@ and read it back
		/*SH A.R.M - 1711 page -Note- 참고 : Write값을 업뎃 하려면 Read를 수행해주어랏을 강하게 주장*/
		mov	r0, #0
		/*SH r0 = 0*/
		mcr	p15, 0, r0, c7, c5, 4	@ ISB
		/*SH FIXME 888줄에서 하는 것을 왜 또하지?*/
		mov	pc, r12
		/*SH pc = 'restart : adr r0, LC0' 명령어의 주소값*/
		/*SH 드뎌 cache_on 함수에서 return!!!!!!*/
/*SH END 130725-1*/

__fa526_cache_on:
		mov	r12, lr
		mov	r6, #CB_BITS | 0x12	@ U
		bl	__setup_mmu
		mov	r0, #0
		mcr	p15, 0, r0, c7, c7, 0	@ Invalidate whole cache
		mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
		mcr	p15, 0, r0, c8, c7, 0	@ flush UTLB
		mrc	p15, 0, r0, c1, c0, 0	@ read control reg
		orr	r0, r0, #0x1000		@ I-cache enable
		bl	__common_mmu_cache_on
		mov	r0, #0
		mcr	p15, 0, r0, c8, c7, 0	@ flush UTLB
		mov	pc, r12

__common_mmu_cache_on:
#ifndef CONFIG_THUMB2_KERNEL
#ifndef DEBUG
		orr	r0, r0, #0x000d		@ Write buffer, mmu
#endif
		mov	r1, #-1
		mcr	p15, 0, r3, c2, c0, 0	@ load page table pointer
		mcr	p15, 0, r1, c3, c0, 0	@ load domain access control
		b	1f
		.align	5			@ cache line aligned
1:		mcr	p15, 0, r0, c1, c0, 0	@ load control register
		mrc	p15, 0, r0, c1, c0, 0	@ and read it back to
		sub	pc, lr, r0, lsr #32	@ properly flush pipeline
#endif

#define PROC_ENTRY_SIZE (4*5)

/*
 * Here follow the relocatable cache support functions for the
 * various processors.  This is a generic hook for locating an
 * entry and jumping to an instruction at the specified offset
 * from the start of the block.  Please note this is all position
 * independent code.
 *
 *  r1  = corrupted
 *  r2  = corrupted
 *  r3  = block offset
 *  r9  = corrupted
 *  r12 = corrupted
 */

/*SH 이 루틴의 r3로 넘어오는 값은 의 의미*/
/*SH r3 = 8 : mmu_cache on 호출*/
/*SH r3 = 12 : mmu_cache off 호출*/
/*SH r3 = 16 : mmu_cache flush 호출*/
call_cache_fn:	adr	r12, proc_types
				/*SH r12 = *(proc_types) */
#ifdef CONFIG_CPU_CP15
		mrc	p15, 0, r9, c0, c0	@ get processor ID
		/*SH r9 = MIDR, Main ID Register, VMSA*/
		/*SH A.R.M - B3.17.2(page 1481) 에서 찾아볼수 있다.*/
		/*SH A.R.M - B4.1.105(page 1648)*/
#else
		ldr	r9, =CONFIG_PROCESSOR_ID
#endif
1:		ldr	r1, [r12, #0]		@ get value
		ldr	r2, [r12, #4]		@ get mask
		eor	r1, r1, r9		@ (real ^ match)
		tst	r1, r2			@       & mask
		/*SH proc_types의 구조체중 현재 processor를 찾아 그에 맞는 mmu_cache 관련 루틴을 호출한다*/
		/*SH 결국 여기(exynos)에서는
		 *	.word	0x000f0000		@ new CPU Id
		 *	.word	0x000f0000
		 *	W(b)	__armv7_mmu_cache_on
		 *	W(b)	__armv7_mmu_cache_off
		 *	W(b)	__armv7_mmu_cache_flush
		 */
 ARM(		addeq	pc, r12, r3		) @ call cache function
			/*SH pc = mmu_cache 관련 함수(r3값으로 결정)*/
 THUMB(		addeq	r12, r3			)
 THUMB(		moveq	pc, r12			) @ call cache function
		add	r12, r12, #PROC_ENTRY_SIZE
		b	1b
		/*SH 라벨명 다음'b' = 'backward'즉 이전 주소의 라벨'1'을 찾아 분기한다*/

/*
 * Table for cache operations.  This is basically:
 *   - CPU ID match
 *   - CPU ID mask
 *   - 'cache on' method instruction
 *   - 'cache off' method instruction
 *   - 'cache flush' method instruction
 *
 * We match an entry using: ((real_id ^ match) & mask) == 0
 *
 * Writethrough caches generally only need 'on' and 'off'
 * methods.  Writeback caches _must_ have the flush method
 * defined.
 */
		.align	2
		.type	proc_types,#object
proc_types:
		.word	0x41000000		@ old ARM ID
		.word	0xff00f000
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)

		.word	0x41007000		@ ARM7/710
		.word	0xfff8fe00
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)

		.word	0x41807200		@ ARM720T (writethrough)
		.word	0xffffff00
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		mov	pc, lr
 THUMB(		nop				)

		.word	0x41007400		@ ARM74x
		.word	0xff00ff00
		W(b)	__armv3_mpu_cache_on
		W(b)	__armv3_mpu_cache_off
		W(b)	__armv3_mpu_cache_flush
		
		.word	0x41009400		@ ARM94x
		.word	0xff00ff00
		W(b)	__armv4_mpu_cache_on
		W(b)	__armv4_mpu_cache_off
		W(b)	__armv4_mpu_cache_flush

		.word	0x41069260		@ ARM926EJ-S (v5TEJ)
		.word	0xff0ffff0
		W(b)	__arm926ejs_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv5tej_mmu_cache_flush

		.word	0x00007000		@ ARM7 IDs
		.word	0x0000f000
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)

		@ Everything from here on will be the new ID system.

		.word	0x4401a100		@ sa110 / sa1100
		.word	0xffffffe0
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv4_mmu_cache_flush

		.word	0x6901b110		@ sa1110
		.word	0xfffffff0
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv4_mmu_cache_flush

		.word	0x56056900
		.word	0xffffff00		@ PXA9xx
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv4_mmu_cache_flush

		.word	0x56158000		@ PXA168
		.word	0xfffff000
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv5tej_mmu_cache_flush

		.word	0x56050000		@ Feroceon
		.word	0xff0f0000
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv5tej_mmu_cache_flush

#ifdef CONFIG_CPU_FEROCEON_OLD_ID
		/* this conflicts with the standard ARMv5TE entry */
		.long	0x41009260		@ Old Feroceon
		.long	0xff00fff0
		b	__armv4_mmu_cache_on
		b	__armv4_mmu_cache_off
		b	__armv5tej_mmu_cache_flush
#endif

		.word	0x66015261		@ FA526
		.word	0xff01fff1
		W(b)	__fa526_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__fa526_cache_flush

		@ These match on the architecture ID

		.word	0x00020000		@ ARMv4T
		.word	0x000f0000
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv4_mmu_cache_flush

		.word	0x00050000		@ ARMv5TE
		.word	0x000f0000
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv4_mmu_cache_flush

		.word	0x00060000		@ ARMv5TEJ
		.word	0x000f0000
		W(b)	__armv4_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv5tej_mmu_cache_flush

		.word	0x0007b000		@ ARMv6
		.word	0x000ff000
		W(b)	__armv6_mmu_cache_on
		W(b)	__armv4_mmu_cache_off
		W(b)	__armv6_mmu_cache_flush

		.word	0x000f0000		@ new CPU Id
		.word	0x000f0000
		W(b)	__armv7_mmu_cache_on
		W(b)	__armv7_mmu_cache_off
		W(b)	__armv7_mmu_cache_flush

		.word	0			@ unrecognised type
		.word	0
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)
		mov	pc, lr
 THUMB(		nop				)

		.size	proc_types, . - proc_types

		/*
		 * If you get a "non-constant expression in ".if" statement"
		 * error from the assembler on this line, check that you have
		 * not accidentally written a "b" instruction where you should
		 * have written W(b).
		 */
		.if (. - proc_types) % PROC_ENTRY_SIZE != 0
		.error "The size of one or more proc_types entries is wrong."
		.endif

/*
 * Turn off the Cache and MMU.  ARMv3 does not support
 * reading the control register, but ARMv4 does.
 *
 * On exit,
 *  r0, r1, r2, r3, r9, r12 corrupted
 * This routine must preserve:
 *  r4, r7, r8
 */
		.align	5
cache_off:	mov	r3, #12			@ cache_off function
		b	call_cache_fn

__armv4_mpu_cache_off:
		mrc	p15, 0, r0, c1, c0
		bic	r0, r0, #0x000d
		mcr	p15, 0, r0, c1, c0	@ turn MPU and cache off
		mov	r0, #0
		mcr	p15, 0, r0, c7, c10, 4	@ drain write buffer
		mcr	p15, 0, r0, c7, c6, 0	@ flush D-Cache
		mcr	p15, 0, r0, c7, c5, 0	@ flush I-Cache
		mov	pc, lr

__armv3_mpu_cache_off:
		mrc	p15, 0, r0, c1, c0
		bic	r0, r0, #0x000d
		mcr	p15, 0, r0, c1, c0, 0	@ turn MPU and cache off
		mov	r0, #0
		mcr	p15, 0, r0, c7, c0, 0	@ invalidate whole cache v3
		mov	pc, lr

__armv4_mmu_cache_off:
#ifdef CONFIG_MMU
		mrc	p15, 0, r0, c1, c0
		bic	r0, r0, #0x000d
		mcr	p15, 0, r0, c1, c0	@ turn MMU and cache off
		mov	r0, #0
		mcr	p15, 0, r0, c7, c7	@ invalidate whole cache v4
		mcr	p15, 0, r0, c8, c7	@ invalidate whole TLB v4
#endif
		mov	pc, lr

__armv7_mmu_cache_off:
		mrc	p15, 0, r0, c1, c0
#ifdef CONFIG_MMU
		bic	r0, r0, #0x000d
#else
		bic	r0, r0, #0x000c
#endif
		mcr	p15, 0, r0, c1, c0	@ turn MMU and cache off
		mov	r12, lr
		bl	__armv7_mmu_cache_flush
		mov	r0, #0
#ifdef CONFIG_MMU
		mcr	p15, 0, r0, c8, c7, 0	@ invalidate whole TLB
#endif
		mcr	p15, 0, r0, c7, c5, 6	@ invalidate BTC
		mcr	p15, 0, r0, c7, c10, 4	@ DSB
		mcr	p15, 0, r0, c7, c5, 4	@ ISB
		mov	pc, r12

/*
 * Clean and flush the cache to maintain consistency.
 *
 * On exit,
 *  r1, r2, r3, r9, r10, r11, r12 corrupted
 * This routine must preserve:
 *  r4, r6, r7, r8
 */
		.align	5
cache_clean_flush:
		mov	r3, #16
		b	call_cache_fn

__armv4_mpu_cache_flush:
		mov	r2, #1
		mov	r3, #0
		mcr	p15, 0, ip, c7, c6, 0	@ invalidate D cache
		mov	r1, #7 << 5		@ 8 segments
1:		orr	r3, r1, #63 << 26	@ 64 entries
2:		mcr	p15, 0, r3, c7, c14, 2	@ clean & invalidate D index
		subs	r3, r3, #1 << 26
		bcs	2b			@ entries 63 to 0
		subs 	r1, r1, #1 << 5
		bcs	1b			@ segments 7 to 0

		teq	r2, #0
		mcrne	p15, 0, ip, c7, c5, 0	@ invalidate I cache
		mcr	p15, 0, ip, c7, c10, 4	@ drain WB
		mov	pc, lr
		
__fa526_cache_flush:
		mov	r1, #0
		mcr	p15, 0, r1, c7, c14, 0	@ clean and invalidate D cache
		mcr	p15, 0, r1, c7, c5, 0	@ flush I cache
		mcr	p15, 0, r1, c7, c10, 4	@ drain WB
		mov	pc, lr

__armv6_mmu_cache_flush:
		mov	r1, #0
		mcr	p15, 0, r1, c7, c14, 0	@ clean+invalidate D
		mcr	p15, 0, r1, c7, c5, 0	@ invalidate I+BTB
		mcr	p15, 0, r1, c7, c15, 0	@ clean+invalidate unified
		mcr	p15, 0, r1, c7, c10, 4	@ drain WB
		mov	pc, lr

__armv7_mmu_cache_flush:
		mrc	p15, 0, r10, c0, c1, 5	@ read ID_MMFR1
		tst	r10, #0xf << 16		@ hierarchical cache (ARMv7)
		mov	r10, #0
		beq	hierarchical
		mcr	p15, 0, r10, c7, c14, 0	@ clean+invalidate D
		b	iflush
hierarchical:
		mcr	p15, 0, r10, c7, c10, 5	@ DMB
		stmfd	sp!, {r0-r7, r9-r11}
		mrc	p15, 1, r0, c0, c0, 1	@ read clidr
		ands	r3, r0, #0x7000000	@ extract loc from clidr
		mov	r3, r3, lsr #23		@ left align loc bit field
		beq	finished		@ if loc is 0, then no need to clean
		mov	r10, #0			@ start clean at cache level 0
loop1:
		add	r2, r10, r10, lsr #1	@ work out 3x current cache level
		mov	r1, r0, lsr r2		@ extract cache type bits from clidr
		and	r1, r1, #7		@ mask of the bits for current cache only
		cmp	r1, #2			@ see what cache we have at this level
		blt	skip			@ skip if no cache, or just i-cache
		mcr	p15, 2, r10, c0, c0, 0	@ select current cache level in cssr
		mcr	p15, 0, r10, c7, c5, 4	@ isb to sych the new cssr&csidr
		mrc	p15, 1, r1, c0, c0, 0	@ read the new csidr
		and	r2, r1, #7		@ extract the length of the cache lines
		add	r2, r2, #4		@ add 4 (line length offset)
		ldr	r4, =0x3ff
		ands	r4, r4, r1, lsr #3	@ find maximum number on the way size
		clz	r5, r4			@ find bit position of way size increment
		ldr	r7, =0x7fff
		ands	r7, r7, r1, lsr #13	@ extract max number of the index size
loop2:
		mov	r9, r4			@ create working copy of max way size
loop3:
 ARM(		orr	r11, r10, r9, lsl r5	) @ factor way and cache number into r11
 ARM(		orr	r11, r11, r7, lsl r2	) @ factor index number into r11
 THUMB(		lsl	r6, r9, r5		)
 THUMB(		orr	r11, r10, r6		) @ factor way and cache number into r11
 THUMB(		lsl	r6, r7, r2		)
 THUMB(		orr	r11, r11, r6		) @ factor index number into r11
		mcr	p15, 0, r11, c7, c14, 2	@ clean & invalidate by set/way
		subs	r9, r9, #1		@ decrement the way
		bge	loop3
		subs	r7, r7, #1		@ decrement the index
		bge	loop2
skip:
		add	r10, r10, #2		@ increment cache number
		cmp	r3, r10
		bgt	loop1
finished:
		ldmfd	sp!, {r0-r7, r9-r11}
		mov	r10, #0			@ swith back to cache level 0
		mcr	p15, 2, r10, c0, c0, 0	@ select current cache level in cssr
iflush:
		mcr	p15, 0, r10, c7, c10, 4	@ DSB
		mcr	p15, 0, r10, c7, c5, 0	@ invalidate I+BTB
		mcr	p15, 0, r10, c7, c10, 4	@ DSB
		mcr	p15, 0, r10, c7, c5, 4	@ ISB
		mov	pc, lr

__armv5tej_mmu_cache_flush:
1:		mrc	p15, 0, r15, c7, c14, 3	@ test,clean,invalidate D cache
		bne	1b
		mcr	p15, 0, r0, c7, c5, 0	@ flush I cache
		mcr	p15, 0, r0, c7, c10, 4	@ drain WB
		mov	pc, lr

__armv4_mmu_cache_flush:
		mov	r2, #64*1024		@ default: 32K dcache size (*2)
		mov	r11, #32		@ default: 32 byte line size
		mrc	p15, 0, r3, c0, c0, 1	@ read cache type
		teq	r3, r9			@ cache ID register present?
		beq	no_cache_id
		mov	r1, r3, lsr #18
		and	r1, r1, #7
		mov	r2, #1024
		mov	r2, r2, lsl r1		@ base dcache size *2
		tst	r3, #1 << 14		@ test M bit
		addne	r2, r2, r2, lsr #1	@ +1/2 size if M == 1
		mov	r3, r3, lsr #12
		and	r3, r3, #3
		mov	r11, #8
		mov	r11, r11, lsl r3	@ cache line size in bytes
no_cache_id:
		mov	r1, pc
		bic	r1, r1, #63		@ align to longest cache line
		add	r2, r1, r2
1:
 ARM(		ldr	r3, [r1], r11		) @ s/w flush D cache
 THUMB(		ldr     r3, [r1]		) @ s/w flush D cache
 THUMB(		add     r1, r1, r11		)
		teq	r1, r2
		bne	1b

		mcr	p15, 0, r1, c7, c5, 0	@ flush I cache
		mcr	p15, 0, r1, c7, c6, 0	@ flush D cache
		mcr	p15, 0, r1, c7, c10, 4	@ drain WB
		mov	pc, lr

__armv3_mmu_cache_flush:
__armv3_mpu_cache_flush:
		mov	r1, #0
		mcr	p15, 0, r1, c7, c0, 0	@ invalidate whole cache v3
		mov	pc, lr

/*
 * Various debugging routines for printing hex characters and
 * memory, which again must be relocatable.
 */
#ifdef DEBUG
		.align	2
		.type	phexbuf,#object
phexbuf:	.space	12
		.size	phexbuf, . - phexbuf

@ phex corrupts {r0, r1, r2, r3}
phex:		adr	r3, phexbuf
		mov	r2, #0
		strb	r2, [r3, r1]
1:		subs	r1, r1, #1
		movmi	r0, r3
		bmi	puts
		and	r2, r0, #15
		mov	r0, r0, lsr #4
		cmp	r2, #10
		addge	r2, r2, #7
		add	r2, r2, #'0'
		strb	r2, [r3, r1]
		b	1b

@ puts corrupts {r0, r1, r2, r3}
puts:		loadsp	r3, r1
1:		ldrb	r2, [r0], #1
		teq	r2, #0
		moveq	pc, lr
2:		writeb	r2, r3
		mov	r1, #0x00020000
3:		subs	r1, r1, #1
		bne	3b
		teq	r2, #'\n'
		moveq	r2, #'\r'
		beq	2b
		teq	r0, #0
		bne	1b
		mov	pc, lr
@ putc corrupts {r0, r1, r2, r3}
putc:
		mov	r2, r0
		mov	r0, #0
		loadsp	r3, r1
		b	2b

@ memdump corrupts {r0, r1, r2, r3, r10, r11, r12, lr}
memdump:	mov	r12, r0
		mov	r10, lr
		mov	r11, #0
2:		mov	r0, r11, lsl #2
		add	r0, r0, r12
		mov	r1, #8
		bl	phex
		mov	r0, #':'
		bl	putc
1:		mov	r0, #' '
		bl	putc
		ldr	r0, [r12, r11, lsl #2]
		mov	r1, #8
		bl	phex
		and	r0, r11, #7
		teq	r0, #3
		moveq	r0, #' '
		bleq	putc
		and	r0, r11, #7
		add	r11, r11, #1
		teq	r0, #7
		bne	1b
		mov	r0, #'\n'
		bl	putc
		cmp	r11, #64
		blt	2b
		mov	pc, r10
#endif

		.ltorg

#ifdef CONFIG_ARM_VIRT_EXT
.align 5
__hyp_reentry_vectors:
		W(b)	.			@ reset
		W(b)	.			@ undef
		W(b)	.			@ svc
		W(b)	.			@ pabort
		W(b)	.			@ dabort
		W(b)	__enter_kernel		@ hyp
		W(b)	.			@ irq
		W(b)	.			@ fiq
#endif /* CONFIG_ARM_VIRT_EXT */

__enter_kernel:
		mov	r0, #0			@ must be 0
 ARM(		mov	pc, r4	)		@ call kernel
 THUMB(		bx	r4	)		@ entry point is always ARM

reloc_code_end:

		.align
		.section ".stack", "aw", %nobits
.L_user_stack:	.space	4096
/*SH .space 4096 : 여기에 공간을 4096만큼 만들어라*/
.L_user_stack_end:
