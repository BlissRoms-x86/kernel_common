/*
 * umip.c Emulation for instruction protected by the Intel User-Mode
 * Instruction Prevention feature
 *
 * Copyright (c) 2017, Intel Corporation.
 * Ricardo Neri <ricardo.neri-calderon@linux.intel.com>
 */

#include <linux/uaccess.h>
#include <asm/umip.h>
#include <asm/traps.h>
#include <asm/insn.h>
#include <asm/insn-eval.h>
#include <linux/ratelimit.h>

/** DOC: Emulation for User-Mode Instruction Prevention (UMIP)
 *
 * The feature User-Mode Instruction Prevention present in recent Intel
 * processor prevents a group of instructions (sgdt, sidt, sldt, smsw, and str)
 * from being executed with CPL > 0. Otherwise, a general protection fault is
 * issued.
 *
 * Rather than relaying to the user space the general protection fault caused by
 * the UMIP-protected instructions (in the form of a SIGSEGV signal), it can be
 * trapped and emulate the result of such instructions to provide dummy values.
 * This allows to both conserve the current kernel behavior and not reveal the
 * system resources that UMIP intends to protect (i.e., the locations of the
 * global descriptor and interrupt descriptor tables, the segment selectors of
 * the local descriptor table, the value of the task state register and the
 * contents of the CR0 register).
 *
 * This emulation is needed because certain applications (e.g., WineHQ and
 * DOSEMU2) rely on this subset of instructions to function.
 *
 * The instructions protected by UMIP can be split in two groups. Those which
 * return a kernel memory address (sgdt and sidt) and those which return a
 * value (sldt, str and smsw).
 *
 * For the instructions that return a kernel memory address, applications
 * such as WineHQ rely on the result being located in the kernel memory space,
 * not the actual location of the table. The result is emulated as a hard-coded
 * value that, lies close to the top of the kernel memory. The limit for the GDT
 * and the IDT are set to zero.
 *
 * Given that sldt and str are not commonly used in programs that run on WineHQ
 * or DOSEMU2, they are not emulated.
 *
 * The instruction smsw is emulated to return the value that the register CR0
 * has at boot time as set in the head_32.
 *
 * Also, emulation is provided only for 32-bit processes; 64-bit processes
 * that attempt to use the instructions that UMIP protects will receive the
 * SIGSEGV signal issued as a consequence of the general protection fault.
 *
 * Care is taken to appropriately emulate the results when segmentation is
 * used. That is, rather than relying on USER_DS and USER_CS, the function
 * insn_get_addr_ref() inspects the segment descriptor pointed by the
 * registers in pt_regs. This ensures that we correctly obtain the segment
 * base address and the address and operand sizes even if the user space
 * application uses a local descriptor table.
 */

#define UMIP_DUMMY_GDT_BASE 0xfffe0000
#define UMIP_DUMMY_IDT_BASE 0xffff0000

/*
 * The SGDT and SIDT instructions store the contents of the global descriptor
 * table and interrupt table registers, respectively. The destination is a
 * memory operand of X+2 bytes. X bytes are used to store the base address of
 * the table and 2 bytes are used to store the limit. In 32-bit processes, the
 * only processes for which emulation is provided, X has a value of 4.
 */
#define UMIP_GDT_IDT_BASE_SIZE 4
#define UMIP_GDT_IDT_LIMIT_SIZE 2

#define	UMIP_INST_SGDT	0	/* 0F 01 /0 */
#define	UMIP_INST_SIDT	1	/* 0F 01 /1 */
#define	UMIP_INST_SMSW	3	/* 0F 01 /4 */

/**
 * identify_insn() - Identify a UMIP-protected instruction
 * @insn:	Instruction structure with opcode and ModRM byte.
 *
 * From the instruction opcode and the reg part of the ModRM byte, identify,
 * if any, a UMIP-protected instruction.
 *
 * Return: a constant that identifies a specific UMIP-protected instruction.
 * -EINVAL when not an UMIP-protected instruction.
 */
static int identify_insn(struct insn *insn)
{
	/* By getting modrm we also get the opcode. */
	insn_get_modrm(insn);

	/* All the instructions of interest start with 0x0f. */
	if (insn->opcode.bytes[0] != 0xf)
		return -EINVAL;

	if (insn->opcode.bytes[1] == 0x1) {
		switch (X86_MODRM_REG(insn->modrm.value)) {
		case 0:
			return UMIP_INST_SGDT;
		case 1:
			return UMIP_INST_SIDT;
		case 4:
			return UMIP_INST_SMSW;
		default:
			return -EINVAL;
		}
	}
	/* SLDT AND STR are not emulated */
	return -EINVAL;
}

/**
 * emulate_umip_insn() - Emulate UMIP instructions with dummy values
 * @insn:	Instruction structure with operands
 * @umip_inst:	Instruction to emulate
 * @data:	Buffer into which the dummy values will be copied
 * @data_size:	Size of the emulated result
 *
 * Emulate an instruction protected by UMIP. The result of the emulation
 * is saved in the provided buffer. The size of the results depends on both
 * the instruction and type of operand (register vs memory address). Thus,
 * the size of the result needs to be updated.
 *
 * Result: 0 if success, -EINVAL on error while emulating.
 */
static int emulate_umip_insn(struct insn *insn, int umip_inst,
			     unsigned char *data, int *data_size)
{
	unsigned long dummy_base_addr, dummy_value;
	unsigned short dummy_limit = 0;

	if (!data || !data_size || !insn)
		return -EINVAL;
	/*
	 * These two instructions return the base address and limit of the
	 * global and interrupt descriptor table, respectively. According to the
	 * Intel Software Development manual, the base address can be 24-bit,
	 * 32-bit or 64-bit. Limit is always 16-bit. If the operand size is
	 * 16-bit, the returned value of the base address is supposed to be a
	 * zero-extended 24-byte number. However, it seems that a 32-byte number
	 * is always returned irrespective of the operand size.
	 */

	if (umip_inst == UMIP_INST_SGDT || umip_inst == UMIP_INST_SIDT) {
		/* SGDT and SIDT do not use registers operands. */
		if (X86_MODRM_MOD(insn->modrm.value) == 3)
			return -EINVAL;

		if (umip_inst == UMIP_INST_SGDT)
			dummy_base_addr = UMIP_DUMMY_GDT_BASE;
		else
			dummy_base_addr = UMIP_DUMMY_IDT_BASE;

		*data_size = UMIP_GDT_IDT_LIMIT_SIZE + UMIP_GDT_IDT_BASE_SIZE;

		memcpy(data + 2, &dummy_base_addr, UMIP_GDT_IDT_BASE_SIZE);
		memcpy(data, &dummy_limit, UMIP_GDT_IDT_LIMIT_SIZE);

	} else if (umip_inst == UMIP_INST_SMSW) {
		dummy_value = CR0_STATE;

		/*
		 * Even though the CR0 register has 4 bytes, the number
		 * of bytes to be copied in the result buffer is determined
		 * by whether the operand is a register or a memory location.
		 * If operand is a register, return as many bytes as the operand
		 * size. If operand is memory, return only the two least
		 * siginificant bytes of CR0.
		 */
		if (X86_MODRM_MOD(insn->modrm.value) == 3)
			*data_size = insn->opnd_bytes;
		else
			*data_size = 2;

		memcpy(data, &dummy_value, *data_size);
	/* STR and SLDT  are not emulated */
	} else {
		return -EINVAL;
	}

	return 0;
}

/**
 * force_sig_info_umip_fault() - Force a SIGSEGV with SEGV_MAPERR
 * @addr:	Address that caused the signal
 * @regs:	Register set containing the instruction pointer
 *
 * Force a SIGSEGV signal with SEGV_MAPERR as the error code. This function is
 * intended to be used to provide a segmentation fault when the result of the
 * UMIP emulation could not be copied to the user space memory.
 *
 * Return: none
 */
static void force_sig_info_umip_fault(void __user *addr, struct pt_regs *regs)
{
	siginfo_t info;
	struct task_struct *tsk = current;

	tsk->thread.cr2		= (unsigned long)addr;
	tsk->thread.error_code	= X86_PF_USER | X86_PF_WRITE;
	tsk->thread.trap_nr	= X86_TRAP_PF;

	info.si_signo	= SIGSEGV;
	info.si_errno	= 0;
	info.si_code	= SEGV_MAPERR;
	info.si_addr	= addr;
	force_sig_info(SIGSEGV, &info, tsk);

	if (!(show_unhandled_signals && unhandled_signal(tsk, SIGSEGV)))
		return;

	pr_err_ratelimited("%s[%d] umip emulation segfault ip:%lx sp:%lx error:%x in %lx\n",
			   tsk->comm, task_pid_nr(tsk), regs->ip,
			   regs->sp, X86_PF_USER | X86_PF_WRITE,
			   regs->ip);
}

/**
 * fixup_umip_exception() - Fixup #GP faults caused by UMIP
 * @regs:	Registers as saved when entering the #GP trap
 *
 * The instructions sgdt, sidt, str, smsw, sldt cause a general protection
 * fault if executed with CPL > 0 (i.e., from user space). If the offending
 * user-space process is 32-bit, this function fixes the exception up and
 * provides dummy values for the sgdt, sidt and smsw; str and sldt are not
 * fixed up. Also 64-bit user-space processes are not fixed up.
 *
 * If operands are memory addresses, results are copied to user-
 * space memory as indicated by the instruction pointed by EIP using the
 * registers indicated in the instruction operands. If operands are registers,
 * results are copied into the context that was saved when entering kernel mode.
 *
 * Result: true if emulation was successful; false if not.
 */
bool fixup_umip_exception(struct pt_regs *regs)
{
	int not_copied, nr_copied, reg_offset, dummy_data_size, umip_inst;
	/* 10 bytes is the maximum size of the result of UMIP instructions */
	unsigned char dummy_data[10] = { 0 };
	unsigned long seg_base, *reg_addr;
	unsigned char buf[MAX_INSN_SIZE];
	void __user *uaddr;
	struct insn insn;
	char seg_defs;

	/* Do not emulate 64-bit processes. */
	if (user_64bit_mode(regs))
		return false;

	/*
	 * Use the segment base in case user space used a different code
	 * segment, either in protected (e.g., from an LDT), virtual-8086
	 * or long (via the FS or GS registers) modes. In most of the cases
	 * seg_base will be zero as in USER_CS.
	 */
	seg_base = insn_get_seg_base(regs, NULL, offsetof(struct pt_regs, ip));
	if (seg_base == -1L)
		return false;

	not_copied = copy_from_user(buf, (void __user *)(seg_base + regs->ip),
				    sizeof(buf));
	nr_copied = sizeof(buf) - not_copied;

	/*
	 * The copy_from_user above could have failed if user code is protected
	 * by a memory protection key. Give up on emulation in such a case.
	 * Should we issue a page fault?
	 */
	if (!nr_copied)
		return false;

	insn_init(&insn, buf, nr_copied, user_64bit_mode(regs));

	/*
	 * Override the default operand and address sizes with what is specified
	 * in the code segment descriptor. The instruction decoder only sets
	 * the address size it to either 4 or 8 address bytes and does nothing
	 * for the operand bytes. This OK for most of the cases, but we could
	 * have special cases where, for instance, a 16-bit code segment
	 * descriptor is used.
	 * If there is an address override prefix, the instruction decoder
	 * correctly updates these values, even for 16-bit defaults.
	 */
	seg_defs = insn_get_code_seg_defaults(regs);
	if (seg_defs == -EINVAL)
		return false;

	insn.addr_bytes = (unsigned char)INSN_CODE_SEG_ADDR_SZ(seg_defs);
	insn.opnd_bytes = (unsigned char)INSN_CODE_SEG_OPND_SZ(seg_defs);

	insn_get_length(&insn);
	if (nr_copied < insn.length)
		return false;

	umip_inst = identify_insn(&insn);
	if (umip_inst < 0)
		return false;

	if (emulate_umip_insn(&insn, umip_inst, dummy_data, &dummy_data_size))
		return false;

	/*
	 * If operand is a register, write result to the copy of the register
	 * value that was pushed to the stack when entering into kernel mode.
	 * Upon exit, the value we write will be restored to the actual hardware
	 * register.
	 */
	if (X86_MODRM_MOD(insn.modrm.value) == 3) {
		reg_offset = insn_get_modrm_rm_off(&insn, regs);

		/*
		 * Negative values are usually errors. In memory addressing,
		 * the exception is -EDOM. Since we expect a register operand,
		 * all negative values are errors.
		 */
		if (reg_offset < 0)
			return false;

		reg_addr = (unsigned long *)((unsigned long)regs + reg_offset);
		memcpy(reg_addr, dummy_data, dummy_data_size);
	} else {
		uaddr = insn_get_addr_ref(&insn, regs);
		if ((unsigned long)uaddr == -1L)
			return false;

		nr_copied = copy_to_user(uaddr, dummy_data, dummy_data_size);
		if (nr_copied  > 0) {
			/*
			 * If copy fails, send a signal and tell caller that
			 * fault was fixed up.
			 */
			force_sig_info_umip_fault(uaddr, regs);
			return true;
		}
	}

	/* increase IP to let the program keep going */
	regs->ip += insn.length;
	return true;
}
