
#include "gdbstub_sys.h"

#ifdef __STRICT_ANSI__
#define asm __asm__
#endif

#define SERIAL_COM1 0x3f8
#define SERIAL_COM2 0x2f8
#define SERIAL_PORT SERIAL_COM1

#define NUM_IDT_ENTRIES 32
void const* const dbg_int_handlers[];
 /*****************************************************************************
  * BSS Data
  ****************************************************************************/

struct dbg_idt_gate dbg_idt_gates[NUM_IDT_ENTRIES];
static struct dbg_state    dbg_state;

/*****************************************************************************
 * Misc. Functions
 ****************************************************************************/

void* dbg_sys_memset(void* ptr, int data, size_t len)
{
	//set memory function
	printf("memset function runs %x: %d, %d", ptr, data, len);
}

/*
 * Get current code segment (CS register).
 */

uint32_t dbg_get_cs(void)
{
	//dbg_get_cs function
	printf("dbg_get_cs function runs\n");
	return 0;
}

/*****************************************************************************
 * Interrupt Management Functions
 ****************************************************************************/

 /*
  * Initialize idt_gates with the interrupt handlers.
  */
int dbg_init_gates(void)
{
	size_t   i;
	uint16_t cs;

	cs = dbg_get_cs();
	for (i = 0; i < NUM_IDT_ENTRIES; i++) 
	{
		dbg_idt_gates[i].flags = 0x8E00;
		dbg_idt_gates[i].segment = cs;
		dbg_idt_gates[i].offset_low =((uint32_t)dbg_int_handlers[i]) & 0xffff;
		dbg_idt_gates[i].offset_high =((uint32_t)dbg_int_handlers[i] >> 16) & 0xffff;
	}

	return 0;
}

/*
 * Load a new IDT.
 */
int dbg_load_idt(struct dbg_idtr* idtr)
{
	

	return 0;
}

/*
 * Get current IDT.
 */
int dbg_store_idt(struct dbg_idtr* idtr)
{
	

	return 0;
}

/*
 * Hook a vector of the current IDT.
 */
int dbg_hook_idt(uint8_t vector, const void* function)
{
	struct dbg_idtr      idtr;
	struct dbg_idt_gate* gates;

	dbg_store_idt(&idtr);
	gates = (struct dbg_idt_gate*)idtr.offset;
	gates[vector].flags = 0x8E00;
	gates[vector].segment = dbg_get_cs();
	gates[vector].offset_low = (((uint32_t)function)) & 0xffff;
	gates[vector].offset_high = (((uint32_t)function) >> 16) & 0xffff;

	return 0;
}

/*
 * Initialize IDT gates and load the new IDT.
 */
int dbg_init_idt(void)
{
	struct dbg_idtr idtr;

	dbg_init_gates();
	idtr.len = sizeof(dbg_idt_gates) - 1;
	idtr.offset = (uint32_t)dbg_idt_gates;
	dbg_load_idt(&idtr);

	return 0;
}

/*
 * Common interrupt handler routine.
 */
void dbg_int_handler(struct dbg_interrupt_state* istate)
{
	dbg_interrupt(istate);
}

/*
 * Debug interrupt handler.
 */
void dbg_interrupt(struct dbg_interrupt_state* istate)
{
	dbg_sys_memset(&dbg_state.registers, 0, sizeof(dbg_state.registers));

	/* Translate vector to signal */
	switch (istate->vector) {
	case 1:  dbg_state.signum = 5; break;
	case 3:  dbg_state.signum = 5; break;
	default: dbg_state.signum = 7;
	}

	/* Load Registers */
	dbg_state.registers[DBG_CPU_I386_REG_EAX] = istate->eax;
	dbg_state.registers[DBG_CPU_I386_REG_ECX] = istate->ecx;
	dbg_state.registers[DBG_CPU_I386_REG_EDX] = istate->edx;
	dbg_state.registers[DBG_CPU_I386_REG_EBX] = istate->ebx;
	dbg_state.registers[DBG_CPU_I386_REG_ESP] = istate->esp;
	dbg_state.registers[DBG_CPU_I386_REG_EBP] = istate->ebp;
	dbg_state.registers[DBG_CPU_I386_REG_ESI] = istate->esi;
	dbg_state.registers[DBG_CPU_I386_REG_EDI] = istate->edi;
	dbg_state.registers[DBG_CPU_I386_REG_PC] = istate->eip;
	dbg_state.registers[DBG_CPU_I386_REG_CS] = istate->cs;
	dbg_state.registers[DBG_CPU_I386_REG_PS] = istate->eflags;
	dbg_state.registers[DBG_CPU_I386_REG_SS] = istate->ss;
	dbg_state.registers[DBG_CPU_I386_REG_DS] = istate->ds;
	dbg_state.registers[DBG_CPU_I386_REG_ES] = istate->es;
	dbg_state.registers[DBG_CPU_I386_REG_FS] = istate->fs;
	dbg_state.registers[DBG_CPU_I386_REG_GS] = istate->gs;

	//dbg_main(&dbg_state);

	/* Restore Registers */
	istate->eax = dbg_state.registers[DBG_CPU_I386_REG_EAX];
	istate->ecx = dbg_state.registers[DBG_CPU_I386_REG_ECX];
	istate->edx = dbg_state.registers[DBG_CPU_I386_REG_EDX];
	istate->ebx = dbg_state.registers[DBG_CPU_I386_REG_EBX];
	istate->esp = dbg_state.registers[DBG_CPU_I386_REG_ESP];
	istate->ebp = dbg_state.registers[DBG_CPU_I386_REG_EBP];
	istate->esi = dbg_state.registers[DBG_CPU_I386_REG_ESI];
	istate->edi = dbg_state.registers[DBG_CPU_I386_REG_EDI];
	istate->eip = dbg_state.registers[DBG_CPU_I386_REG_PC];
	istate->cs = dbg_state.registers[DBG_CPU_I386_REG_CS];
	istate->eflags = dbg_state.registers[DBG_CPU_I386_REG_PS];
	istate->ss = dbg_state.registers[DBG_CPU_I386_REG_SS];
	istate->ds = dbg_state.registers[DBG_CPU_I386_REG_DS];
	istate->es = dbg_state.registers[DBG_CPU_I386_REG_ES];
	istate->fs = dbg_state.registers[DBG_CPU_I386_REG_FS];
	istate->gs = dbg_state.registers[DBG_CPU_I386_REG_GS];
}

/*****************************************************************************
 * I/O Functions
 ****************************************************************************/

 /*
  * Write to I/O port.
  */
void dbg_io_write_8(uint16_t port, uint8_t val)
{
	//dbg_io_write_8
	printf("dbg_io_write_8 function runs\n");
	return 0;
}

/*
 * Read from I/O port.
 */
uint8_t dbg_io_read_8(uint16_t port)
{
	//dbg_io_write_8
	printf("dbg_io_read_8 function runs\n");
	//returns number of bytes
	return 0;
	
}

/*****************************************************************************
 * NS16550 Serial Port (IO)
 ****************************************************************************/

#define SERIAL_THR 0
#define SERIAL_RBR 0
#define SERIAL_LSR 5

int dbg_serial_getc(void)
{
	/* Wait for data */
	while ((dbg_io_read_8(SERIAL_PORT + SERIAL_LSR) & 1) == 0);
	return dbg_io_read_8(SERIAL_PORT + SERIAL_RBR);
}

int dbg_serial_putchar(int ch)
{
	/* Wait for THRE (bit 5) to be high */
	while ((dbg_io_read_8(SERIAL_PORT + SERIAL_LSR) & (1 << 5)) == 0);
	dbg_io_write_8(SERIAL_PORT + SERIAL_THR, ch);
	return ch;
}

/*****************************************************************************
 * Debugging System Functions
 ****************************************************************************/

 /*
  * Write one character to the debugging stream.
  */
int dbg_sys_putchar(int ch)
{
	return dbg_serial_putchar(ch);
}

/*
 * Read one character from the debugging stream.
 */
int dbg_sys_getc(void)
{
	return dbg_serial_getc() & 0xff;
}

/*
 * Read one byte from memory.
 */
int dbg_sys_mem_readb(address addr, char* val)
{
	*val = *(volatile char*)addr;
	return 0;
}

/*
 * Write one byte to memory.
 */
int dbg_sys_mem_writeb(address addr, char val)
{
	*(volatile char*)addr = val;
	return 0;
}

/*
 * Continue program execution.
 */
int dbg_sys_continue(void)
{
	//dbg_sys_continue
	printf("dbg_sys_continue function runs\n");
	return 0;
}

/*
 * Single step the next instruction.
 */
int dbg_sys_step(void)
{
	//dbg_sys_step
	printf("dbg_sys_step function runs\n");
	return 0;
}

/*
 * Debugger init function.
 *
 * Hooks the IDT to enable debugging.
 */
void dbg_start(void)
{
	/* Hook current IDT. */
	dbg_hook_idt(1, dbg_int_handlers[1]);
	dbg_hook_idt(3, dbg_int_handlers[3]);

	/* Interrupt to start debugging. */
	asm volatile ("int3");
}