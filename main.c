#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <limits.h>
#include <ctype.h>
#include <gc.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#define UNUSED(var) ((void)(var))
#define SV_FORMAT(sv) (int)(sv).len, (sv).str
#define STACK_SZ 1024
#define PRG_SZ   1024
#define DATA_SZ  (1024 * 4)
#define TOKEN_STK_SZ (1024 * 2)
#define STACK_END_PTR(stack) ((stack) + STACK_SZ)
#define INST_iAx(op, ax) {.as_u32 = ((ax) << 8)|(op)}
#define INT24_MAX (1L << 23)
#define INT24_MIN (-INT24_MAX)

typedef union _word {
	int64_t as_i64;
	uint64_t as_u64;
	double as_f64;
	int32_t as_pair[2];
	void *as_ptr;
} WORD;

typedef union _inst {
	uint32_t as_u32;
	int32_t as_i32;
	uint8_t as_bytes[4];
	struct {
		uint8_t i;
		uint8_t a;
		uint8_t b;
		uint8_t c;
	} iABC;
	struct {
		uint8_t i;
		uint8_t a;
		uint16_t bx;
	} iABx;
} INST;

typedef struct _sv {
	size_t len;
	char *str;
} StrView;

typedef struct _svp {
	StrView fst;
	StrView snd;
} StrViewPair;

typedef struct _tokstk {
	size_t len;
	StrView entries[TOKEN_STK_SZ];
} TokenStack;

typedef struct _label {
	StrView name;
	uint64_t value;
} Label;

#define LABELTBL_SZ 1024

typedef struct _labeltbl {
	size_t len;
	Label entries[LABELTBL_SZ];
} LabelTbl;

typedef struct _macrodef {
	TokenStack *args;
	TokenStack *tokens;
} MacroDef;

typedef struct _vm {
	WORD *fp;
	WORD *sp;
	INST *ip;
	jmp_buf trp;
	int status;
	WORD *stack;
	INST *exe;
	INST *exe_end;
	uint8_t *data;
	uint8_t *data_end;
	TokenStack *tokstk;
	LabelTbl *deftbl;
	LabelTbl *restbl;
	LabelTbl *constants;
	LabelTbl *macros;
} Vm;

enum opcode {
	op_nop = 0,
	op_halt,
	op_break,     /* break on next instruction and start debugging */
	op_ldi,       /* iABx fp[A] := Bx */
	op_ldw,       /* iABC-f64  fp[A] := immi64 */
	op_ldf,       /* iABC-f64  fp[A] := immf64 */
	op_lfp,       /* iABC fp[A] := fp */
	op_lsp,       /* iABC fp[A] := sp */
	op_lip,       /* iABC fp[A] := ip */
	op_alloca,    /* iAx  sp := sp + Ax */
	op_malloc,    /* iABx fp[A] := malloc(Bx) */
	op_enter,     /* ixx  push fp; fp := sp */
	op_leave,     /* ixx  sp := fp; fp := pop */
	op_mov,       /* iABC fp[A] := fp[B] */
	op_fetch,     /* iABC fp[A] := ((WORD *)fp[B])[C] */
	op_store,     /* iABC ((WORD *)fp[A])[B] := fp[C] */
	op_fetchb,
	op_storeb,
	op_puts,      /* iABC puts((addr)fp[A]) */
	op_preg,
	op_addp,      /* iABC fp[A] := (WORD *)fp[B] + fp[C] */
	op_subp,      /* iABC fp[A] := (WORD *)fp[B] - fp[C] */
	op_and,       /* iABC fp[A] := fp[B] & fp[C] */
	op_or,        /* iABC fp[A] := fp[B] | fp[C] */
	op_xor,       /* iABC fp[A] := fp[B] ^ fp[C] */
	op_shl,       /* iABC fp[A] := fp[B] << fp[C] */
	op_shr,       /* iABC fp[A] := fp[B] >> fp[C] */
	op_inc,       /* iABC fp[A] := fp[B] + C */
	op_dec,       /* iABC fp[A] := fp[B] - C */
	op_addi,      /* iABC fp[A] := fp[B] + fp[C] */
	op_subi,      /* iABC fp[A] := fp[B] - fp[C] */
	op_muli,      /* iABC fp[A] := fp[B] * fp[C] */
	op_divi,      /* iABC fp[A] := fp[B] / fp[C] */
	op_mod,       /* iABC fp[A] := fp[B] % fp[C] */
	op_addf,
	op_subf,
	op_mulf,
	op_divf,
	op_eq,        /* iABC fp[A] := fp[B] == fp[C] */
	op_jmp,       /* iAx  ip :=  ip + Ax */
	op_jez,       /* iABx if fp[A] == 0 then ip := ip + Bx */
	op_jnz,
	op_jlz,
	op_jlez,
	op_jgz,
	op_jgez,
	op_call,      /* iABC ip := fp[A] */
	op_call0,     /* iABC ip := immu64 */
	op_ret,
	OP_COUNT,
};

enum trapcode {
	TRAP_OK = 0,
	TRAP_STACKOVERFLOW,
	TRAP_STACKUNDERFLOW,
	TRAP_INVALIDOPCODE,
	TRAP_USERINTERUPT,
};

static StrView sv_next_token(StrView sv, StrView *rest);
void vm_init(Vm *vm);
char *instruction_tostr(INST *inst, INST **next_inst);
char *trapcode_tostr(enum trapcode code);
char *opcode_tostr(enum opcode op);
int vm_debug(Vm *vm);
int vm_run(Vm *vm, int step_debug);
void vm_dump_stack(Vm *vm);
uint64_t hash(void *buff, size_t size);
int sv_pair_list_find_fst(StrViewPair *list, size_t len, StrView sv, StrView *ret);
StrView sv_slurp_file(char *filepath);
StrView sv_from_cstr(char *str);
char *sv_to_cstr(StrView sv);
int sv_eq(StrView s1, StrView s2);
int sv_eq_cstr(StrView sv, char *cstr);
int sv_to_int(StrView sv, int64_t *ret);
int sv_to_f64(StrView sv, double *ret);
StrView sv_trim_ws(StrView sv);
StrView sv_strip_ws(StrView sv);
LabelTbl *make_labeltbl(void);
void push_label(LabelTbl *tbl, StrView name, uintptr_t addr);
int lookup_label(LabelTbl *tbl, StrView name, uint64_t *ret);
int chin(int ch, char *cstr);
int islabelchar(int ch);
int try_parse_char(int ch, StrView sv, StrView *ret);
int try_parse_str(char *cstr, StrView sv, StrView *ret);
int try_parse_float(TokenStack *tokstk, double *ret);
int try_parse_int(TokenStack *tokstk, int64_t *ret);
void resolve_labels(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl);
int invoke_macro(Vm *vm);
int assemble(Vm *vm);
int assemble_executable_section(Vm *vm);
int assemble_data_section(Vm *vm);
void disassemble(Vm *vm);
TokenStack *make_token_stack(void);
void push_token(TokenStack *stk, StrView token);
TokenStack *copy_token_stack(TokenStack *stk);
void populate_token_stack(TokenStack *stk, StrView sv);
void reverse_token_stack(TokenStack *stk);
StrView pop_token(TokenStack *stk);
StrView peek_token(TokenStack *stk);

#define IS_JCC(op) ((op) >= op_jez && (op) <= op_jgez)
#define VM_TRAP(vm_ptr, value)					\
	do {										\
		(vm_ptr)->status = value;				\
		longjmp((vm)->trp, value);				\
	} while (0)

char *
instruction_tostr(INST *inst, INST **next_inst)
{
	static char buff[255];
	enum opcode op = inst->iABC.i;
	switch (op) {
	case op_nop:
	case op_break:
	case op_ret:
	case op_enter:
	case op_leave: {
		snprintf(buff, sizeof(buff), "%s;", opcode_tostr(op));
		inst++;
	} break;
	case op_halt:
	case op_alloca:
	case op_malloc:
	case op_jmp: {
		int32_t ax = inst->as_i32 >> 8;
		snprintf(buff, sizeof(buff),
				 "%s %d;", opcode_tostr(op), ax);
		inst += 1;
	} break;
	case op_ldw: {
		int8_t a = (int8_t)inst->iABC.a;
		inst += 1;
		int64_t b = *((int64_t *)inst);
		snprintf(buff, sizeof(buff),
				 "%s %d, %ld;", opcode_tostr(op), a, b);
		inst += 2;
	} break;
	case op_ldf: {
		int8_t a = (int8_t)inst->iABC.a;
		inst += 1;
		double b = *((double *)inst);
		snprintf(buff, sizeof(buff),
				 "%s %d, %g;", opcode_tostr(op), a, b);
		inst += 2;
	} break;
	case op_lfp:
	case op_lsp:
	case op_lip:
	case op_call:
	case op_preg:
	case op_puts: {
		int8_t a = (int8_t)inst->iABC.a;
		snprintf(buff, sizeof(buff), "%s %d;", opcode_tostr(op), a);
		inst += 1;
	} break;
	case op_mov: {
		int8_t a = (int8_t)inst->iABC.a;
		int8_t b = (int8_t)inst->iABC.b;
		snprintf(buff, sizeof(buff), "%s %d, %d;",
				 opcode_tostr(op), a, b);
		inst += 1;
	} break;
	case op_fetch:
	case op_store:
	case op_fetchb:
	case op_storeb:
	case op_inc:
	case op_dec:
	case op_addp:
	case op_subp:
	case op_and:
	case op_or:
	case op_xor:
	case op_shl:
	case op_shr:
	case op_addi:
	case op_subi:
	case op_muli:
	case op_divi:
	case op_mod:
	case op_addf:
	case op_subf:
	case op_mulf:
	case op_divf:
	case op_eq: {
		int8_t a = (int8_t)inst->iABC.a;
		int8_t b = (int8_t)inst->iABC.b;
		int8_t c = (int8_t)inst->iABC.c;
		snprintf(buff, sizeof(buff),
				 "%s %d, %d, %d;", opcode_tostr(op), a, b, c);
		inst += 1;
	} break;
	case op_ldi:
	case op_jez:
	case op_jnz:
	case op_jlz:
	case op_jlez:
	case op_jgz:
	case op_jgez: {
		int8_t a = (int8_t)inst->iABx.a;
		int16_t bx = (int16_t)inst->iABx.bx;
		snprintf(buff, sizeof(buff),
				 "%s %d, %d;", opcode_tostr(op), a, bx);
		inst += 1;
	} break;
	case op_call0: {
		inst++;
		void *p = *((void **)inst);
		snprintf(buff, sizeof(buff),
				 "%s %p;", opcode_tostr(op), p);
		inst += 2;
	} break;
	case OP_COUNT:
	default: assert(!"Invalid instruction");
	}
	if (next_inst) *next_inst = inst;
	return buff;
}

char *
trapcode_tostr(enum trapcode code)
{
	switch (code) {
	case TRAP_OK:				return "OK";
	case TRAP_STACKOVERFLOW:	return "STACKOVERFLOW";
	case TRAP_STACKUNDERFLOW:	return "STACKUNDERFLOW";
	case TRAP_INVALIDOPCODE:	return "INVALIDOPCODE";
	case TRAP_USERINTERUPT:		return "USERINTERUPT";
	default: assert(!"invalid trap code");
	}
	return NULL;
}


char *
opcode_tostr(enum opcode op)
{
	switch (op) {
	case op_nop:	return "nop";
	case op_break:	return "break";
	case op_halt:	return "halt";
	case op_ldi:	return "ldi";
	case op_ldw:	return "ldw";
	case op_ldf:	return "ldf";
	case op_lfp:	return "lfp";
	case op_lsp:	return "lsp";
	case op_lip:	return "lip";
	case op_alloca: return "alloca";
	case op_malloc: return "malloc";
	case op_enter:	return "enter";
	case op_leave:	return "leave";
	case op_mov:	return "mov";
	case op_fetch:	return "fetch";
	case op_store:	return "store";
	case op_fetchb: return "fetchb";
	case op_storeb: return "storeb";
	case op_puts:	return "puts";
	case op_preg:	return "preg";
	case op_addp:	return "addp";
	case op_subp:	return "subp";
	case op_and:	return "and";
	case op_or:		return "or";
	case op_xor:	return "xor";
	case op_shl:	return "shl";
	case op_shr:	return "shr";
	case op_inc:	return "inc";
	case op_dec:	return "dec";
	case op_addi:	return "addi";
	case op_subi:	return "subi";
	case op_muli:	return "muli";
	case op_divi:	return "divi";
	case op_mod:	return "mod";
	case op_addf:	return "addf";
	case op_subf:	return "subf";
	case op_mulf:	return "mulf";
	case op_divf:	return "divf";
	case op_eq:		return "eq";
	case op_jmp:	return "jmp";
	case op_jez:	return "jez";
	case op_jnz:	return "jnz";
	case op_jlz:	return "jlz";
	case op_jlez:	return "jlez";
	case op_jgz:	return "jgz";
	case op_jgez:	return "jgez";
	case op_call:	return "call";
	case op_call0:	return "call0";
	case op_ret:	return "ret";
	case OP_COUNT:	return "OP_COUNT";
	default: assert(!"Invalid opcode");
	}
	return NULL;
}

void
vm_init(Vm *vm)
{
	vm->stack		= GC_MALLOC(sizeof(vm->stack[0]) * STACK_SZ);
	vm->exe			= GC_MALLOC(sizeof(vm->exe[0]) * PRG_SZ);
	vm->fp			= vm->stack;
	vm->sp			= vm->stack;
	vm->ip			= vm->exe;
	vm->exe_end		= vm->exe;
	vm->data		= 0;
	vm->deftbl		= make_labeltbl();
	vm->restbl		= make_labeltbl();
	vm->constants	= make_labeltbl();
	vm->macros  	= make_labeltbl();
	vm->tokstk		= make_token_stack();
}

int
vm_debug(Vm *vm)
{
	static char buff[255];
	void *addr;
	char *in;
	StrView sv;
	for (;;) {
		addr = vm->ip;
		in = instruction_tostr(addr, NULL);
		printf("%p: %s\n: ", addr, in);
		in = fgets(buff, sizeof(buff), stdin);
		if (in == NULL) {
			VM_TRAP(vm, TRAP_USERINTERUPT);
		}
		sv = sv_strip_ws(sv_from_cstr(in));
		if (sv.len == 0) continue;
		if (sv_eq_cstr(sv, "q")) {
			VM_TRAP(vm, TRAP_USERINTERUPT);
		} else if (sv_eq_cstr(sv, "n")) {
			break;
		} else if (sv_eq_cstr(sv, "c")) {
			return 0;
		} else if (sv_eq_cstr(sv, "list")) {
			disassemble(vm);
		} else if (sv_eq_cstr(sv, "sd")) {
			vm_dump_stack(vm);
		} else {
			printf("Unrecognized command.\n");
		}
	}
	return 1;
}

static WORD *
vm_index_frame(Vm *vm, int64_t idx)
{
	WORD *ptr = vm->fp + idx;
	if (ptr < vm->stack) {
		VM_TRAP(vm, TRAP_STACKUNDERFLOW);
		return NULL;
	}
	if (ptr >= vm->sp) {
		VM_TRAP(vm, TRAP_STACKOVERFLOW);
		return NULL;
	}
	return ptr;
}

int
vm_run(Vm *vm, int step_debug)
{
	if (setjmp(vm->trp)) {
		vm_dump_stack(vm);
		return -1;
	}
	for (;;) {
		assert(vm->ip < vm->exe_end);
		if (step_debug) {
			step_debug = vm_debug(vm);
		}
		INST inst = *vm->ip++;
		enum opcode oc = inst.iABC.i;
		switch (oc) {
		case op_nop: break;
		case op_break: {
			step_debug = 1;
		} break;
		case op_halt: {
			int32_t ax = inst.as_i32 >> 8;
			vm_dump_stack(vm);
			return ax;
		} break;
		case op_ldi: {
			int8_t a = (int8_t)inst.iABx.a;
			WORD *w = vm_index_frame(vm, a);
			w->as_i64 = (int16_t)inst.iABx.bx;
		} break;
		case op_ldw: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			w->as_i64 = *((int64_t *)vm->ip);
			vm->ip += 2;
		} break;
		case op_ldf: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			w->as_f64 = *((double *)vm->ip);
			vm->ip += 2;
		} break;
		case op_lfp: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			w->as_ptr = vm->fp;
		} break;
		case op_lsp: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			w->as_ptr = vm->sp;
		} break;
		case op_lip: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			w->as_ptr = (void *)vm->ip;
		} break;
		case op_alloca: {
			int64_t ax = inst.as_i32 >> 8;
			vm->sp += ax;
			if (vm->sp >= STACK_END_PTR(vm->stack)) {
				VM_TRAP(vm, TRAP_STACKOVERFLOW);
			}
			if (vm->sp < vm->fp || vm->sp < vm->stack) {
				VM_TRAP(vm, TRAP_STACKUNDERFLOW);
			}
		} break;
		case op_malloc: {
			int8_t a = (int8_t)inst.iABx.a;
			uint16_t bx = inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			w->as_ptr = GC_MALLOC(bx);
		} break;
		case op_enter: {
			vm->sp[0].as_ptr = vm->fp;
			vm->sp++;
			vm->fp = vm->sp;
			if (vm->sp >= STACK_END_PTR(vm->stack)) {
				VM_TRAP(vm, TRAP_STACKOVERFLOW);
			}
		} break;
		case op_leave: {
			vm->sp = vm->fp - 1;
			vm->fp = vm->fp[-1].as_ptr;
		} break;
		case op_mov: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			*wa = *wb;
		} break;
		case op_fetch: {
			/* iABC fp[A] := ((WORD *)fp[B])[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *w = vm_index_frame(vm, b);
			WORD *ptr = w->as_ptr;
			w = vm_index_frame(vm, a);
			*w = ptr[c];
		} break;
		case op_store: {
			/* iABC ((WORD *)fp[A])[B] := fp[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *w = vm_index_frame(vm, a);
			WORD *ptr = w->as_ptr;
			w = vm_index_frame(vm, c);
			ptr[b] = *w;
		} break;
		case op_fetchb: {
			/* iABC fp[A] := ((char *)fp[B])[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *w = vm_index_frame(vm, b);
			int8_t *ptr = w->as_ptr;
			w = vm_index_frame(vm, a);
			w->as_i64 = ptr[c];
		} break;
		case op_storeb: {
			/* iABC ((char *)fp[A])[B] := fp[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *w = vm_index_frame(vm, a);
			int8_t *ptr = w->as_ptr;
			w = vm_index_frame(vm, c);
			ptr[b] = w->as_i64;
		} break;
		case op_puts: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			puts(w->as_ptr);
		} break;
		case op_preg: {
			int8_t a = (int8_t)inst.iABC.a;
			WORD *w = vm_index_frame(vm, a);
			printf("%ld\n", w->as_i64);
		} break;
		case op_addp: {
			/* iABC fp[A] := (WORD *)fp[B] + fp[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *w = vm_index_frame(vm, b);
			WORD *ptr = w->as_ptr;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_ptr = ptr + wc->as_i64;
		} break;
		case op_subp: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *w = vm_index_frame(vm, b);
			WORD *ptr = w->as_ptr;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_ptr = ptr - wc->as_i64;
		} break;
		case op_and: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_u64 = wb->as_u64 & wc->as_u64;
		} break;
		case op_or: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_u64 = wb->as_u64 | wc->as_u64;
		} break;
		case op_xor: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_u64 = wb->as_u64 ^ wc->as_u64;
		} break;
		case op_shl: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_u64 = wb->as_u64 << wc->as_u64;
		} break;
		case op_shr: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_u64 = wb->as_u64 >> wc->as_u64;
		} break;
		case op_inc: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			wa->as_i64 = wb->as_i64 + c;
		} break;
		case op_dec: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			wa->as_i64 = wb->as_i64 - c;
		} break;
		case op_addi: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_i64 = wb->as_i64 + wc->as_i64;
		} break;
		case op_subi: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_i64 = wb->as_i64 - wc->as_i64;
		} break;
		case op_muli: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_i64 = wb->as_i64 * wc->as_i64;
		} break;
		case op_divi: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_i64 = wb->as_i64 / wc->as_i64;
		} break;
		case op_mod: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_i64 = wb->as_i64 % wc->as_i64;
		} break;
		case op_addf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_f64 = wb->as_f64 + wc->as_f64;
		} break;
		case op_subf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_f64 = wb->as_f64 - wc->as_f64;
		} break;
		case op_mulf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_f64 = wb->as_f64 * wc->as_f64;
		} break;
		case op_divf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_f64 = wb->as_f64 / wc->as_f64;
		} break;
		case op_eq: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *wa = vm_index_frame(vm, a);
			WORD *wb = vm_index_frame(vm, b);
			WORD *wc = vm_index_frame(vm, c);
			wa->as_i64 = wb->as_i64 == wc->as_i64;
		} break;
		case op_jmp: {
			int32_t ax = inst.as_i32 >> 8;
			vm->ip += ax;
		} break;
		case op_jez: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			if (w->as_i64 == 0) {
				vm->ip += bx;
			}
		} break;
		case op_jnz: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			if (w->as_i64 != 0) {
				vm->ip += bx;
			}
		} break;
		case op_jlz: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			if (w->as_i64 < 0) {
				vm->ip += bx;
			}
		} break;
		case op_jlez: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			if (w->as_i64 <= 0) {
				vm->ip += bx;
			}
		} break;
		case op_jgz: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			if (w->as_i64 > 0) {
				vm->ip += bx;
			}
		} break;
		case op_jgez: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			WORD *w = vm_index_frame(vm, a);
			if (w->as_i64 >= 0) {
				vm->ip += bx;
			}
		} break;
		case op_call: {
			int8_t a = (int8_t)inst.iABC.a;
			INST *addr = vm->fp[a].as_ptr;
			vm->sp[0].as_ptr = vm->ip;
			vm->sp++;
			if (vm->sp >= STACK_END_PTR(vm->stack)) {
				VM_TRAP(vm, TRAP_STACKOVERFLOW);
			}
			vm->ip = addr;
		} break;
		case op_call0: {
			INST *addr = *((INST **)vm->ip);
			vm->ip += 2;
			vm->sp[0].as_ptr = vm->ip;
			vm->sp++;
			if (vm->sp >= STACK_END_PTR(vm->stack)) {
				VM_TRAP(vm, TRAP_STACKOVERFLOW);
			}
			vm->ip = addr;
		} break;
		case op_ret: {
			vm->sp--;
			if (vm->sp < vm->stack) {
				VM_TRAP(vm, TRAP_STACKUNDERFLOW);
			}
			vm->ip = vm->sp[0].as_ptr;
		} break;
		case OP_COUNT:
		default: VM_TRAP(vm, TRAP_INVALIDOPCODE);
		}
	}
}

TokenStack *
make_token_stack(void)
{
	TokenStack *tstck = GC_MALLOC(sizeof(*tstck));
	tstck->len = 0;
	return tstck;
}

void
push_token(TokenStack *stk, StrView token)
{
	assert(stk->len < TOKEN_STK_SZ);
	stk->entries[stk->len++] = token;
}

TokenStack *
copy_token_stack(TokenStack *stk)
{
	TokenStack *new_stk = make_token_stack();
	memcpy(new_stk->entries, stk->entries, stk->len * sizeof(*stk->entries));
	new_stk->len = stk->len;
	return new_stk;
}

void
reverse_token_stack(TokenStack *stk)
{
	if (stk->len == 0) return;
	StrView token;
	size_t i = 0;
	size_t j = stk->len - 1;
	while (i < j) {
		token = stk->entries[i];
		stk->entries[i] = stk->entries[j];
		stk->entries[j] = token;
		i++;
		j--;
	}
}

void
populate_token_stack(TokenStack *stk, StrView sv)
{
	StrView token;
	while (sv.len) {
		token = sv_next_token(sv, &sv);
		if (token.len == 0) break;
		if (sv_eq_cstr(token, "\"")) {
			push_token(stk, token);
			token.len = 0;
			token.str = sv.str;
			while (sv.len) {
				if (*sv.str == '"') {
					break;
				}
				sv.str++;
				sv.len--;
				token.len++;
			}
			push_token(stk, token);
			token = sv_next_token(sv, &sv);
		}
		push_token(stk, token);
	}
	reverse_token_stack(stk);
}

StrView
pop_token(TokenStack *stk)
{
	assert(stk->len > 0);
	stk->len--;
	return stk->entries[stk->len];
}

StrView
peek_token(TokenStack *stk)
{
	assert(stk->len > 0);
	return stk->entries[stk->len - 1];
}

LabelTbl *
make_labeltbl(void)
{
	LabelTbl *tbl = GC_MALLOC(sizeof(*tbl));
	tbl->len = 0;
	return tbl;
}

void
push_label(LabelTbl *tbl, StrView name, uint64_t value)
{
	assert(tbl->len < LABELTBL_SZ);
	tbl->entries[tbl->len].value = value;
	tbl->entries[tbl->len++].name = name;
}

int
lookup_label(LabelTbl *tbl, StrView name, uint64_t *ret)
{
	for (size_t i = 0; i < tbl->len; ++i) {
		if (sv_eq(name, tbl->entries[i].name)) {
			*ret = tbl->entries[i].value;
			return 1;
		}
	}
	return 0;
}

uint64_t
hash(void *buff, size_t size)
{
	uint8_t *str = buff;
	uint64_t h = 5381;
	while (size--) {
		h = ((h << 5) + h) + (*str++);
	}
	return h;
}

StrView
sv_from_cstr(char *str)
{
	return (StrView) {
		.len = strlen(str),
		.str = str,
	};
}

char *
sv_to_cstr(StrView sv)
{
	static char buff[255];
	assert(sv.len < sizeof(buff) - 1);
	memcpy(buff, sv.str, sv.len);
	buff[sv.len] = '\0';
	return buff;
}

int
sv_pair_list_find_fst(StrViewPair *list, size_t len, StrView sv, StrView *ret)
{
	for (size_t i = 0; i < len; ++i) {
		if (sv_eq(list[i].fst, sv)) {
			*ret = list[i].snd;
			return 1;
		}
	}
	return 0;
}

StrView
sv_slurp_file(char *filepath)
{
	StrView sv = {0};
	int fd = open(filepath, O_RDONLY, 0);
	if (fd < 0) goto err0;
	struct stat sb = {0};
	if (fstat(fd, &sb) < 0) goto err1;
	sv.len = sb.st_size;
	sv.str = GC_MALLOC(sv.len);
	read(fd, sv.str, sv.len);
	close(fd);
	return sv;
err1:
	close(fd);
err0:
	perror("Error in sv_map_file");
	return (StrView) {0};
}

int
sv_eq(StrView s1, StrView s2)
{
	if (s1.len != s2.len) return 0;
	while (s1.len) {
		if (*s1.str++ != *s2.str++) return 0;
		s1.len--;
		s2.len--;
	}
	return 1;
}

int
sv_eq_cstr(StrView sv, char *cstr)
{
	return sv_eq(sv, sv_from_cstr(cstr));
}

int
chin(int ch, char *cstr)
{
	for (; *cstr != '\0'; cstr++) {
		if (*cstr == ch) {
			return 1;
		}
	}
	return 0;
}

int
islabelchar(int ch)
{
	return !isspace(ch)
		&& !chin(ch, "+-*&^#!,/\"'[]{}()|=:;`~");
}

int
sv_to_int(StrView sv, int64_t *ret)
{
	char *endptr;
	int64_t n = strtol(sv.str, &endptr, 10);
	if (sv.str + sv.len == endptr) {
		*ret = n;
		return 1;
	}
	return 0;
}

int
sv_to_f64(StrView sv, double *ret)
{
	char *endptr;
	double n = strtod(sv.str, &endptr);
	if (sv.str + sv.len == endptr) {
		*ret = n;
		return 1;
	}
	return 0;
}

static int
isdelim(int ch)
{
	return isspace(ch) || chin(ch, "+-*&^#!,/\"'[]{}()|=:;`~");
}

static StrView
eat_ws_and_comments(StrView sv)
{
	sv = sv_trim_ws(sv);
	if (sv.len && *sv.str == '#') {
		while (sv.len && *sv.str != '\n') {
			sv.str++;
			sv.len--;
		}
		return eat_ws_and_comments(sv);
	}
	return sv;
}


static StrView
sv_chop_by_delim(StrView sv, StrView *rest)
{
	StrView token = {
		.len = 0,
		.str = sv.str,
	};
	while (sv.len && !isdelim(*sv.str)) {
		sv.len--;
		sv.str++;
		token.len++;
	}
	*rest = sv;
	return token;
}

static StrView
sv_next_token(StrView sv, StrView *rest)
{
	sv = eat_ws_and_comments(sv);
	StrView token = {
		.len = 0,
		.str = sv.str,
	};
	if (sv.len && isdelim(*sv.str)) {
		sv.len--;
		sv.str++;
		token.len++;
		*rest = sv;
		return token;
	}
	token = sv_chop_by_delim(sv, &sv);
	*rest = sv;
	return token;
}

StrView
sv_trim_ws(StrView sv)
{
	while (sv.len && isspace(*sv.str)) {
		sv.str++;
		sv.len--;
	}
	return sv;
}

StrView
sv_strip_ws(StrView sv)
{
	sv = sv_trim_ws(sv);
	while (sv.len && isspace(sv.str[sv.len - 1])) {
		sv.len--;
	}
	return sv;
}

int
try_parse_char(int ch, StrView sv, StrView *ret)
{
	if (sv.len && *sv.str == ch) {
		sv.str++;
		sv.len--;
		*ret = sv;
		return 1;
	}
	return 0;
}

int
try_parse_str(char *cstr, StrView sv, StrView *ret)
{
	while (*cstr != '\0') {
		if (sv.len == 0 || *cstr != *sv.str) {
			return 0;
		}
		cstr++;
		sv.str++;
		sv.len--;
	}
	if (sv.len && isdelim(*sv.str)) {
		*ret = sv;
		return 1;
	}
	return 0;
}

int
try_parse_float(TokenStack *tokstk, double *ret)
{
	StrView token = pop_token(tokstk);
	if (sv_eq_cstr(token, "-")) {
		if (sv_to_f64(peek_token(tokstk), ret)) {
			pop_token(tokstk);
			*ret = -*ret;
			return 1;
		} else {
			push_token(tokstk, token);
			return 0;
		}
	} else if (sv_to_f64(token, ret)) {
		return 1;
	} else {
		push_token(tokstk, token);
		return 0;
	}

}

int
try_parse_int(TokenStack *tokstk, int64_t *ret)
{
	StrView token = pop_token(tokstk);
	if (sv_eq_cstr(token, "-")) {
		if (sv_to_int(peek_token(tokstk), ret)) {
			pop_token(tokstk);
			*ret = -*ret;
			return 1;
		} else {
			push_token(tokstk, token);
			return 0;
		}
	} else if (sv_to_int(token, ret)) {
		return 1;
	} else {
		push_token(tokstk, token);
		return 0;
	}

}

static void
push_inst(Vm *vm, INST inst)
{
	assert(vm->exe_end < vm->exe + PRG_SZ);
	*vm->exe_end++ = inst;
}

#define PARSE_COMMA()							\
	do {										\
		token = pop_token(vm->tokstk);			\
		assert(sv_eq_cstr(token, ","));			\
	} while (0)

#define PARSE_ABC(op)													\
	do {																\
		if (!try_parse_int(vm->tokstk, &a)) {							\
			assert(lookup_label(vm->constants, token, &w.as_u64));		\
			a = w.as_i64;												\
		}																\
		assert(a >= INT8_MIN && a <= INT8_MAX);							\
		PARSE_COMMA();													\
		if (!try_parse_int(vm->tokstk, &b)) {							\
			assert(lookup_label(vm->constants, token, &w.as_u64));		\
			b = w.as_i64;												\
		}																\
		assert(b >= INT8_MIN && b <= INT8_MAX);							\
		PARSE_COMMA();													\
		if (!try_parse_int(vm->tokstk, &c)) {							\
			assert(lookup_label(vm->constants, token, &w.as_u64));		\
			c = w.as_i64;												\
		}																\
		assert(c >= INT8_MIN && c <= INT8_MAX);							\
		push_inst(vm, (INST){.iABC = {.i = (op), .a = a, .b = b, .c = c}}); \
	} while (0)

#define PARSE_ABx(op)													\
	do {																\
		if (!try_parse_int(vm->tokstk, &a)) {							\
			assert(lookup_label(vm->constants, token, &w.as_u64));		\
			a = w.as_i64;												\
		}																\
		assert(a >= INT8_MIN && a <= INT8_MAX);							\
		PARSE_COMMA();													\
		if (try_parse_int(vm->tokstk, &b)) {							\
			assert(b >= INT16_MIN && b <= INT16_MAX);					\
		} else {														\
			token = pop_token(vm->tokstk);								\
			if (lookup_label(vm->constants, token, &w.as_u64)) {		\
				b = w.as_i64;											\
			} else {													\
				push_label(vm->restbl, token, (uintptr_t)(vm->exe_end)); \
				b = 0;													\
			}															\
		}																\
		assert(b >= INT16_MIN && b <= INT16_MAX);						\
		push_inst(vm, (INST){.iABx = {.i = (op), .a = a, .bx = b}});	\
	} while (0)

#define PARSE_Ax(op)													\
	do {																\
		if (try_parse_int(vm->tokstk, &a)) {							\
			assert(a >= INT24_MIN && a <= INT24_MAX);					\
		} else {														\
			token = pop_token(vm->tokstk);								\
			if (lookup_label(vm->constants, token, &w.as_u64)) {		\
				a = w.as_i64;											\
			} else {													\
				push_label(vm->restbl, token, (uintptr_t)(vm->exe_end)); \
				a = 0;													\
			}															\
			assert(a >= INT24_MIN && a <= INT24_MAX);					\
		}																\
		push_inst(vm, (INST)INST_iAx((op), a));							\
	} while (0)

void
resolve_labels(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl)
{
	uintptr_t addr1 = 0, addr2 = 0;
	StrView name;
	INST *inst = 0;
	for (size_t i = 0; i < restbl->len; ++i) {
		name = restbl->entries[i].name;
		addr1 = restbl->entries[i].value;
		assert(lookup_label(deftbl, name, &addr2) && "Error label undefined");
		inst = (INST *)addr1;
		if (inst == NULL) {
			vm->ip = (INST *)addr2;
			continue;
		}
		int op = inst->iABC.i;
		if (op == op_call0 || op == op_ldw) {
			inst++;
			*((uintptr_t *)inst) = addr2;
		} else if (op == op_jmp) {
			int64_t offset = (INST *)addr2 - ((INST *)addr1 + 1);
			assert(offset >= INT24_MIN && offset <= INT24_MAX);
			*inst = (INST)INST_iAx(op, offset);
		} else if (IS_JCC(op)) {
			int64_t offset = (INST *)addr2 - ((INST *)addr1 + 1);
			assert(offset >= INT16_MIN && offset <= INT16_MAX);
			inst->iABx.bx = offset;
		} else {
			assert(!"invalid opcode");
		}
	}
}

int
invoke_macro(Vm *vm)
{
	StrView token = pop_token(vm->tokstk);
	WORD w;
	if (sv_eq_cstr(token, "macro")) {
		token = pop_token(vm->tokstk);
		MacroDef *md = GC_MALLOC(sizeof(*md));
		md->args = make_token_stack();
		md->tokens = make_token_stack();
		while (sv_eq_cstr(peek_token(vm->tokstk), ",")) {
			pop_token(vm->tokstk);
			push_token(md->args, pop_token(vm->tokstk));
		}
		reverse_token_stack(md->args);
		assert(sv_eq_cstr(pop_token(vm->tokstk), "{"));
		while (!sv_eq_cstr(peek_token(vm->tokstk), "}")) {
			push_token(md->tokens, pop_token(vm->tokstk));
		}
		pop_token(vm->tokstk);
		reverse_token_stack(md->tokens);
		push_label(vm->macros, token, (uint64_t)md);
	} else if (sv_eq_cstr(token, "const")) {
		token = pop_token(vm->tokstk);
		assert(sv_eq_cstr(pop_token(vm->tokstk), "="));
		if (try_parse_int(vm->tokstk, &w.as_i64)) {
			push_label(vm->constants, token, w.as_u64);
			sv_eq_cstr(pop_token(vm->tokstk), ";");
			return 1;
		} else if (try_parse_float(vm->tokstk, &w.as_f64)) {
			push_label(vm->constants, token, w.as_u64);
			sv_eq_cstr(pop_token(vm->tokstk), ";");
			return 1;
		} else {
			token = pop_token(vm->tokstk);
			if (lookup_label(vm->constants, token, &w.as_u64)) {
				push_label(vm->constants, token, w.as_u64);
				sv_eq_cstr(pop_token(vm->tokstk), ";");
				return 1;
			} else {
				assert(!"Invalid constant value");
			}
		}
	} else if (sv_eq_cstr(token, "include")) {
		assert(sv_eq_cstr(pop_token(vm->tokstk), "\""));
		token = pop_token(vm->tokstk);
		StrView sv_asm = sv_slurp_file(sv_to_cstr(token));
		TokenStack *ts = vm->tokstk;
		vm->tokstk = make_token_stack();
		populate_token_stack(vm->tokstk, sv_asm);
		assemble(vm);
		vm->tokstk = ts;
		assert(sv_eq_cstr(pop_token(vm->tokstk), "\""));
		sv_eq_cstr(pop_token(vm->tokstk), ";");
		return 1;
	} else if (lookup_label(vm->macros, token, &w.as_u64)) {
		static StrViewPair bindings[32];
		MacroDef *md = w.as_ptr;
		TokenStack *args = copy_token_stack(md->args);
		TokenStack *tokens = copy_token_stack(md->tokens);
		TokenStack *out_stk = make_token_stack();
		size_t nargs = md->args->len;
		assert(nargs < 32);
		for (size_t i = 0; i < nargs; ++i) {
			bindings[i].fst = pop_token(args);
			bindings[i].snd = pop_token(vm->tokstk);
			if (sv_eq_cstr(peek_token(vm->tokstk), ",")) {
				pop_token(vm->tokstk);
			} else {
				assert((i == md->args->len - 1)
					   && sv_eq_cstr(pop_token(vm->tokstk), ";"));
			}
		}
		while (tokens->len) {
			token = pop_token(tokens);
			sv_pair_list_find_fst(bindings, nargs, token, &token);
			push_token(out_stk, token);
		}
		while (out_stk->len) {
			token = pop_token(out_stk);
			push_token(vm->tokstk, token);
		}
	} else {
		assert(!"Undefined macro");
	}
	return 0;
}

int
assemble_data_section(Vm *vm)
{
	/* defining data:
	 * db *bytes*
	 * df *floats*
	 * dw *words*
	 * pad *padding bytes*
	 */
	WORD w;
	int64_t word;
	double flt;
	StrView token;
	while (vm->tokstk->len > 0) {
		token = pop_token(vm->tokstk);
		if (sv_eq_cstr(token, "section")) {
			return 1;
		} else if (sv_eq_cstr(token, "db")) {
			for (;;) {
				token = pop_token(vm->tokstk);
				if (sv_eq_cstr(token, "rep")) {
					assert(try_parse_int(vm->tokstk, &word));
					vm->data_end += (uint64_t)word;
					break;
				} else if (sv_eq_cstr(token, "\"")) {
					token = pop_token(vm->tokstk);
					while (token.len) {
						*vm->data_end++ = *token.str++;
						token.len--;
					}
					assert(sv_eq_cstr(pop_token(vm->tokstk), "\""));
				} else if (lookup_label(vm->constants, token, &w.as_u64)) {
					assert(w.as_i64 >= INT8_MIN && w.as_i64 <= INT8_MAX);
					*vm->data_end++ = word;
				} else {
					push_token(vm->tokstk, token);
					assert(try_parse_int(vm->tokstk, &word));
					assert(word >= INT8_MIN && word <= INT8_MAX);
					*vm->data_end++ = word;
				}
				if (!sv_eq_cstr(peek_token(vm->tokstk), ",")) {
					break;
				} else {
					pop_token(vm->tokstk);
				}
			}
		} else if (sv_eq_cstr(token, "df")) {
			for (;;) {
				token = pop_token(vm->tokstk);
				if (sv_eq_cstr(token, "rep")) {
					assert(try_parse_int(vm->tokstk, &word));
					vm->data_end += (sizeof(WORD) * (uint64_t)word);
					break;
				} else if (lookup_label(vm->constants, token, &w.as_u64)) {
					*((double *)vm->data_end) = w.as_f64;
					vm->data_end += sizeof(WORD);
				} else {
					push_token(vm->tokstk, token);
					assert(try_parse_float(vm->tokstk, &flt));
					*((double *)vm->data_end) = flt;
					vm->data_end += sizeof(WORD);
				}
				if (!sv_eq_cstr(peek_token(vm->tokstk), ",")) {
					break;
				} else {
					pop_token(vm->tokstk);
				}
			}
		} else if (sv_eq_cstr(token, "dw")) {
			for (;;) {
				token = pop_token(vm->tokstk);
				if (sv_eq_cstr(token, "rep")) {
					assert(try_parse_int(vm->tokstk, &word));
					vm->data_end += (sizeof(WORD) * (uint64_t)word);
					break;
				} else if (lookup_label(vm->constants, token, &w.as_u64)) {
					*((int64_t *)vm->data_end) = w.as_i64;
					vm->data_end += sizeof(WORD);
				} else {
					push_token(vm->tokstk, token);
					assert(try_parse_int(vm->tokstk, &word));
					*((int64_t *)vm->data_end) = word;
					vm->data_end += sizeof(WORD);
				}
				if (!sv_eq_cstr(peek_token(vm->tokstk), ",")) {
					break;
				} else {
					pop_token(vm->tokstk);
				}
			}
		} else if (sv_eq_cstr(token, "!")) {
			invoke_macro(vm);
			continue;
		} else {
			push_label(vm->deftbl, token, (uintptr_t)(vm->data_end));
			assert(sv_eq_cstr(pop_token(vm->tokstk), ":"));
			continue;
		}
		assert(sv_eq_cstr(pop_token(vm->tokstk), ";"));
	}
	return 0;
}

int
assemble_executable_section(Vm *vm)
{
	int64_t a, b, c;
	WORD w;
	StrView token;
	while (vm->tokstk->len > 0) {
		token = pop_token(vm->tokstk);
		int i = 0;
		for (; i < OP_COUNT; ++i) {
			if (sv_eq(token, sv_from_cstr(opcode_tostr(i)))) {
				switch ((enum opcode)i) {
				case op_nop:
				case op_break: {
					push_inst(vm, (INST){.iABC = {.i = i}});
				} break;
				case op_halt:
				case op_alloca:
				case op_malloc:
				case op_jmp: {
					PARSE_Ax(i);
				} break;
				case op_ldw: {
					if (!try_parse_int(vm->tokstk, &a)) {
						assert(lookup_label(vm->constants, token, &w.as_u64));
						a = w.as_i64;
					}
					assert(a >= INT8_MIN && a <= INT8_MAX);
					PARSE_COMMA();
					if (try_parse_int(vm->tokstk, &w.as_i64)) {
						push_inst(vm, (INST){.iABC = {.i = i, .a = a}});
						push_inst(vm, (INST){.as_i32 = w.as_pair[0]});
						push_inst(vm, (INST){.as_i32 = w.as_pair[1]});
					} else {
						token = pop_token(vm->tokstk);
						if (lookup_label(vm->constants, token, &w.as_u64)) {
							push_inst(vm, (INST){.iABC = {.i = i, .a = a}});
							push_inst(vm, (INST){.as_i32 = w.as_pair[0]});
							push_inst(vm, (INST){.as_i32 = w.as_pair[1]});
						} else {
							push_label(vm->restbl, token, (uintptr_t)(vm->exe_end));
							push_inst(vm, (INST){.iABC = {.i = i, .a = a}});
							push_inst(vm, (INST){.as_i32 = 0});
							push_inst(vm, (INST){.as_i32 = 0});
						}
					}
				} break;
				case op_ldf: {
					if (!try_parse_int(vm->tokstk, &a)) {
						assert(lookup_label(vm->constants, token, &w.as_u64));
						a = w.as_i64;
					}
					assert(a >= INT8_MIN && a <= INT8_MAX);
					PARSE_COMMA();
					if (!try_parse_float(vm->tokstk, &w.as_f64)) {
						assert(lookup_label(vm->constants, token, &w.as_u64));
					}
					push_inst(vm, (INST){.iABC = {.i = i, .a = a}});
					push_inst(vm, (INST){.as_i32 = w.as_pair[0]});
					push_inst(vm, (INST){.as_i32 = w.as_pair[1]});
				} break;
				case op_lfp:
				case op_lsp:
				case op_lip:
				case op_call:
				case op_preg:
				case op_puts: {
					if (!try_parse_int(vm->tokstk, &a)) {
						assert(lookup_label(vm->constants, token, &w.as_u64));
						a = w.as_i64;
					}
					assert(a >= INT8_MIN && a <= INT8_MAX);
					push_inst(vm, (INST){.iABC = {.i = i, .a = a}});
				} break;
				case op_enter:
				case op_leave: {
					push_inst(vm, (INST)INST_iAx(i, 0));
				} break;
				case op_mov: {
					if (!try_parse_int(vm->tokstk, &a)) {
						assert(lookup_label(vm->constants, token, &w.as_u64));
						a = w.as_i64;
					}
					assert(a >= INT8_MIN && a <= INT8_MAX);
					PARSE_COMMA();
					if (!try_parse_int(vm->tokstk, &b)) {
						assert(lookup_label(vm->constants, token, &w.as_u64));
						b = w.as_i64;
					}
					assert(b >= INT8_MIN && b <= INT8_MAX);
					push_inst(vm, (INST){.iABC = {.i = i, .a = a, .b = b}});
				} break;
				case op_fetch:
				case op_store:
				case op_fetchb:
				case op_storeb:
				case op_inc:
				case op_dec:
				case op_addp:
				case op_subp:
				case op_and:
				case op_or:
				case op_xor:
				case op_shl:
				case op_shr:
				case op_addi:
				case op_subi:
				case op_muli:
				case op_divi:
				case op_mod:
				case op_addf:
				case op_subf:
				case op_mulf:
				case op_divf:
				case op_eq: {
					PARSE_ABC(i);
				} break;
				case op_ldi:
				case op_jez:
				case op_jnz:
				case op_jlz:
				case op_jlez:
				case op_jgz:
				case op_jgez: {
					PARSE_ABx(i);
				} break;
				case op_call0: {
					if (try_parse_int(vm->tokstk, &c)) {
						w = (WORD){.as_i64 = c};
						push_inst(vm, (INST){.iABC = {.i = i}});
						push_inst(vm, (INST){.as_i32 = w.as_pair[0]});
						push_inst(vm, (INST){.as_i32 = w.as_pair[1]});
					} else {
						token = pop_token(vm->tokstk);
						if (lookup_label(vm->constants, token, &w.as_u64)) {
							push_inst(vm, (INST){.iABC = {.i = i}});
							push_inst(vm, (INST){.as_i32 = w.as_pair[0]});
							push_inst(vm, (INST){.as_i32 = w.as_pair[1]});
						} else {
							push_label(vm->restbl, token, (uintptr_t)(vm->exe_end));
							push_inst(vm, (INST){.iABC = {.i = i}});
							push_inst(vm, (INST){.as_i32 = 0});
							push_inst(vm, (INST){.as_i32 = 0});
						}
					}
				} break;
				case op_ret: {
					push_inst(vm, (INST){.iABC = {.i = i}});
				} break;
				case OP_COUNT:
				default: assert(!"Invalid instruction");
				}
				break;
			}
		}
		if (i == OP_COUNT) {
			if (sv_eq_cstr(token, "!")) {
				invoke_macro(vm);
				continue;
			} else if (sv_eq_cstr(token, "section")) {
				return 1;
			} else if (sv_eq_cstr(token, "entry")) {
				token = pop_token(vm->tokstk);
				push_label(vm->restbl, token, 0);
			} else {
				push_label(vm->deftbl, token, (uintptr_t)(vm->exe_end));
				assert(sv_eq_cstr(pop_token(vm->tokstk), ":"));
				continue;
			}
		}
		assert(sv_eq_cstr(pop_token(vm->tokstk), ";"));
	}
	return 0;
}

int
assemble(Vm *vm)
{
	int ds = 0, es = 0;
	StrView token;
	while (vm->tokstk->len > 0) {
		token = pop_token(vm->tokstk);
		if (sv_eq_cstr(token, "section")) {
		begin:
			token = pop_token(vm->tokstk);
			if (sv_eq_cstr(token, "executable")) {
				assert(sv_eq_cstr(pop_token(vm->tokstk), ":"));
				assert(es == 0);
				es = 1;
				if (assemble_executable_section(vm)) {
					goto begin;
				} else {
					break;
				}
			} else if (sv_eq_cstr(token, "data")) {
				assert(sv_eq_cstr(pop_token(vm->tokstk), ":"));
				assert(ds == 0);
				ds = 1;
				vm->data = GC_MALLOC(DATA_SZ);
				vm->data_end = vm->data;
				if (assemble_data_section(vm)) {
					goto begin;
				} else {
					break;
				}
			} else {
				assert(!"Error Unknown section type");
			}
		} else if (sv_eq_cstr(token, "!")) {
			invoke_macro(vm);
		}
	}
	resolve_labels(vm, vm->deftbl, vm->restbl);
	return 0;
}

void
vm_dump_stack(Vm *vm)
{
	WORD value, *sp = vm->stack;
	printf("fp: %p\n", (void *)vm->fp);
	printf("sp: %p\n", (void *)vm->sp);
	printf("ip: %p\n", (void *)vm->ip);
	printf("stack@%p:\n", (void *)vm->stack);
	int i = 0;
	while (sp < vm->sp) {
		value = *sp++;
		printf("%d\t% 16ld; %- 12.12f; %#lx\n", i++, value.as_i64, value.as_f64, value.as_i64);
	}
}

void
disassemble(Vm *vm)
{
	INST *inst = vm->exe;
	printf("address       : instruction\n");
	while (inst < vm->exe_end) {
		void *addr = inst;
		char *str = instruction_tostr(inst, &inst);
		printf("%p: %s\n", addr, str);
	}
}

int
main(void)
{
	GC_INIT();
	Vm vm = {0};
	vm_init(&vm);
	StrView sv_asm = sv_slurp_file("rot13.yasm");
	populate_token_stack(vm.tokstk, sv_asm);
	assemble(&vm);
	if (vm_run(&vm, 0) < 0) {
		fprintf(stderr, "Error trap signalled: %s\n", trapcode_tostr(vm.status));
	}
	return 0;
}
