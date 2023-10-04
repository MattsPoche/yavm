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
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdbool.h>

#define UNUSED(var) ((void)(var))
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

typedef struct _vm {
	WORD *fp;
	WORD *sp;
	INST *ip;
	jmp_buf trp;
	int status;
	WORD *stack;
	INST *exe;
	uint8_t *data;
	uint8_t *data_end;
} Vm;

typedef struct _sv {
	size_t len;
	char *str;
} StrView;

typedef struct _label {
	StrView name;
	uintptr_t addr;
} Label;

#define LABELTBL_SZ 1024

typedef struct _labeltbl {
	size_t len;
	Label entries[LABELTBL_SZ];
} LabelTbl;

enum opcode {
	op_nop = 0,
	op_halt,
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
	op_addp,      /* iABC fp[A] := (WORD *)fp[B] + fp[C] */
	op_subp,      /* iABC fp[A] := (WORD *)fp[B] - fp[C] */
	op_and,       /* iABC fp[A] := fp[B] & fp[C] */
	op_or,        /* iABC fp[A] := fp[B] | fp[C] */
	op_xor,       /* iABC fp[A] := fp[B] ^ fp[C] */
	op_shl,       /* iABC fp[A] := fp[B] << fp[C] */
	op_shr,       /* iABC fp[A] := fp[B] >> fp[C] */
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
};

void vm_init(Vm *vm);
char *trapcode_tostr(enum trapcode code);
char *opcode_tostr(enum opcode op);
int vm_run(Vm *vm);
void vm_dump_stack(Vm *vm);
uint64_t hash(void *buff, size_t size);
StrView sv_map_file(char *filepath);
StrView sv_from_cstr(char *str);
int sv_eq(StrView s1, StrView s2);
int sv_eq_cstr(StrView sv, char *cstr);
int sv_to_int(StrView sv, int64_t *ret);
int sv_to_f64(StrView sv, double *ret);
StrView sv_trim_ws(StrView sv);
LabelTbl *make_labeltbl(void);
void push_label(LabelTbl *tbl, StrView name, uintptr_t addr);
uintptr_t lookup_label(LabelTbl *tbl, StrView name);
int chin(int ch, char *cstr);
int islabelchar(int ch);
int try_parse_char(int ch, StrView sv, StrView *ret);
int try_parse_str(char *cstr, StrView sv, StrView *ret);
int try_parse_float(StrView sv, StrView *sv_ret, double *ret);
int try_parse_int(StrView sv, StrView *sv_ret, int64_t *ret);
int try_parse_label(StrView sv, StrView *ret, StrView *label_name);
void resolve_labels(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl);
int assemble(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl, StrView sv);
int assemble_executable_segment(Vm *vm, LabelTbl *deftbl,
								LabelTbl *restbl, StrView sv, StrView *ret);
int assemble_data_segment(Vm *vm, LabelTbl *deftbl,
						  LabelTbl *restbl, StrView sv, StrView *ret);


#define IS_JCC(op) ((op) >= op_jez && (op) <= op_jgez)
#define VM_TRAP(vm_ptr, value)					\
	do {										\
		(vm_ptr)->status = value;				\
		longjmp((vm)->trp, value);				\
	} while (0)

char *
trapcode_tostr(enum trapcode code)
{
	switch (code) {
	case TRAP_OK: return "OK";
	case TRAP_STACKOVERFLOW: return "STACKOVERFLOW";
	case TRAP_STACKUNDERFLOW: return "STACKUNDERFLOW";
	case TRAP_INVALIDOPCODE: return "INVALIDOPCODE";
	default: assert(!"invalid trap code");
	}
	return NULL;
}


char *
opcode_tostr(enum opcode op)
{
	switch (op) {
	case op_nop: return "nop";
	case op_halt: return "halt";
	case op_ldi: return "ldi";
	case op_ldw: return "ldw";
	case op_ldf: return "ldf";
	case op_lfp: return "lfp";
	case op_lsp: return "lsp";
	case op_lip: return "lip";
	case op_alloca: return "alloca";
	case op_malloc: return "malloc";
	case op_enter: return "enter";
	case op_leave: return "leave";
	case op_mov: return "mov";
	case op_fetch: return "fetch";
	case op_store: return "store";
	case op_fetchb: return "fetchb";
	case op_storeb: return "storeb";
	case op_puts: return "puts";
	case op_addp: return "addp";
	case op_subp: return "subp";
	case op_and: return "and";
	case op_or: return "or";
	case op_xor: return "xor";
	case op_shl: return "shl";
	case op_shr: return "shr";
	case op_addi: return "addi";
	case op_subi: return "subi";
	case op_muli: return "muli";
	case op_divi: return "divi";
	case op_mod: return "mod";
	case op_addf: return "addf";
	case op_subf: return "subf";
	case op_mulf: return "mulf";
	case op_divf: return "divf";
	case op_eq: return "eq";
	case op_jmp: return "jmp";
	case op_jez: return "jez";
	case op_jnz: return "jnz";
	case op_jlz: return "jlz";
	case op_jlez: return "jlez";
	case op_jgz: return "jgz";
	case op_jgez: return "jgez";
	case op_call: return "call";
	case op_call0: return "call0";
	case op_ret: return "ret";
	case OP_COUNT: return "OP_COUNT";
	default: assert(!"Invalid opcode");
	}
	return NULL;
}

void
vm_init(Vm *vm)
{
	vm->stack = GC_MALLOC(sizeof(vm->stack[0]) * STACK_SZ);
	vm->exe = GC_MALLOC(sizeof(vm->exe[0]) * PRG_SZ);
	vm->fp = vm->stack;
	vm->sp = vm->stack;
	vm->ip = vm->exe;
	vm->data = 0;
}

int
vm_run(Vm *vm)
{
	if (setjmp(vm->trp)) {
		vm_dump_stack(vm);
		return -1;
	}
	for (;;) {
		INST inst = *vm->ip++;
		enum opcode oc = inst.iABC.i;
		switch (oc) {
		case op_nop: break;
		case op_halt: {
			int32_t ax = inst.as_i32 >> 8;
			vm_dump_stack(vm);
			return ax;
		} break;
		case op_ldi: {
			int8_t a = (int8_t)inst.iABx.a;
			vm->fp[a].as_i64 = (int16_t)inst.iABx.bx;
		} break;
		case op_ldw: {
			int8_t a = (int8_t)inst.iABC.a;
			vm->fp[a].as_i64 = *((int64_t *)vm->ip);
			vm->ip += 2;
		} break;
		case op_ldf: {
			int8_t a = (int8_t)inst.iABC.a;
			vm->fp[a].as_f64 = *((double *)vm->ip);
			vm->ip += 2;
		} break;
		case op_lfp: {
			int8_t a = (int8_t)inst.iABC.a;
			vm->fp[a].as_ptr = vm->fp;
		} break;
		case op_lsp: {
			int8_t a = (int8_t)inst.iABC.a;
			vm->fp[a].as_ptr = vm->sp;
		} break;
		case op_lip: {
			int8_t a = (int8_t)inst.iABC.a;
			vm->fp[a].as_ptr = (void *)vm->ip;
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
			vm->fp[a].as_ptr = GC_MALLOC(bx);
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
			vm->fp[a] = vm->fp[b];
		} break;
		case op_fetch: {
			/* iABC fp[A] := ((WORD *)fp[B])[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *ptr = vm->fp[b].as_ptr;
			vm->fp[a] = ptr[c];
		} break;
		case op_store: {
			/* iABC ((WORD *)fp[A])[B] := fp[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *ptr = vm->fp[a].as_ptr;
			ptr[b] = vm->fp[c];
		} break;
		case op_fetchb: {
			/* iABC fp[A] := ((WORD *)fp[B])[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			int8_t *ptr = vm->fp[b].as_ptr;
			vm->fp[a].as_i64 = ptr[c];
		} break;
		case op_storeb: {
			/* iABC ((WORD *)fp[A])[B] := fp[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			int8_t *ptr = vm->fp[a].as_ptr;
			ptr[b] = vm->fp[c].as_i64;
		} break;
		case op_puts: {
			int8_t a = (int8_t)inst.iABC.a;
			puts(vm->fp[a].as_ptr);
		} break;
		case op_addp: {
			/* iABC fp[A] := (WORD *)fp[B] + fp[C] */
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *ptr = vm->fp[b].as_ptr;
			vm->fp[a].as_ptr = ptr + vm->fp[c].as_i64;
		} break;
		case op_subp: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			WORD *ptr = vm->fp[b].as_ptr;
			vm->fp[a].as_ptr = ptr - vm->fp[c].as_i64;
		} break;
		case op_and: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_u64 = vm->fp[b].as_u64 & vm->fp[c].as_u64;
		} break;
		case op_or: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_u64 = vm->fp[b].as_u64 | vm->fp[c].as_u64;
		} break;
		case op_xor: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_u64 = vm->fp[b].as_u64 ^ vm->fp[c].as_u64;
		} break;
		case op_shl: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_u64 = vm->fp[b].as_u64 << vm->fp[c].as_u64;
		} break;
		case op_shr: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_u64 = vm->fp[b].as_u64 >> vm->fp[c].as_u64;
		} break;
		case op_addi: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_i64 = vm->fp[b].as_i64 + vm->fp[c].as_i64;
		} break;
		case op_subi: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_i64 = vm->fp[b].as_i64 - vm->fp[c].as_i64;
		} break;
		case op_muli: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_i64 = vm->fp[b].as_i64 * vm->fp[c].as_i64;
		} break;
		case op_divi: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_i64 = vm->fp[b].as_i64 / vm->fp[c].as_i64;
		} break;
		case op_mod: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_i64 = vm->fp[b].as_i64 % vm->fp[c].as_i64;
		} break;
		case op_addf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_f64 = vm->fp[b].as_f64 + vm->fp[c].as_f64;
		} break;
		case op_subf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_f64 = vm->fp[b].as_f64 - vm->fp[c].as_f64;
		} break;
		case op_mulf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_f64 = vm->fp[b].as_f64 * vm->fp[c].as_f64;
		} break;
		case op_divf: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_f64 = vm->fp[b].as_f64 / vm->fp[c].as_f64;
		} break;
		case op_eq: {
			int8_t a = (int8_t)inst.iABC.a;
			int8_t b = (int8_t)inst.iABC.b;
			int8_t c = (int8_t)inst.iABC.c;
			vm->fp[a].as_i64 = vm->fp[b].as_i64 == vm->fp[c].as_i64;
		} break;
		case op_jmp: {
			int32_t ax = inst.as_i32 >> 8;
			vm->ip += ax;
		} break;
		case op_jez: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			if (vm->fp[a].as_i64 == 0) {
				vm->ip += bx;
			}
		} break;
		case op_jnz: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			if (vm->fp[a].as_i64 != 0) {
				vm->ip += bx;
			}
		} break;
		case op_jlz: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			if (vm->fp[a].as_i64 < 0) {
				vm->ip += bx;
			}
		} break;
		case op_jlez: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			if (vm->fp[a].as_i64 <= 0) {
				vm->ip += bx;
			}
		} break;
		case op_jgz: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			if (vm->fp[a].as_i64 > 0) {
				vm->ip += bx;
			}
		} break;
		case op_jgez: {
			int8_t a = (int8_t)inst.iABx.a;
			int16_t bx = (int16_t)inst.iABx.bx;
			if (vm->fp[a].as_i64 >= 0) {
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

LabelTbl *
make_labeltbl(void)
{
	LabelTbl *tbl = GC_MALLOC(sizeof(*tbl));
	tbl->len = 0;
	return tbl;
}

void
push_label(LabelTbl *tbl, StrView name, uintptr_t addr)
{
	assert(tbl->len < LABELTBL_SZ);
	tbl->entries[tbl->len].addr = addr;
	tbl->entries[tbl->len++].name = name;
}

uintptr_t
lookup_label(LabelTbl *tbl, StrView name)
{
	for (size_t i = 0; i < tbl->len; ++i) {
		if (sv_eq(name, tbl->entries[i].name)) {
			return tbl->entries[i].addr;
		}
	}
	assert(!"label undefined");
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

StrView
sv_map_file(char *filepath)
{
	StrView sv = {0};
	int fd = open(filepath, O_RDONLY, 0);
	if (fd < 0) goto err0;
	struct stat sb = {0};
	if (fstat(fd, &sb) < 0) goto err1;
	sv.len = sb.st_size;
	sv.str = mmap(NULL, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (sv.str == MAP_FAILED) goto err1;
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
		return sv_trim_ws(sv);
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

static size_t
push_inst(INST *prg, size_t idx, INST inst)
{
	assert(idx < PRG_SZ);
	prg[idx++] = inst;
	return idx;
}

#define PARSE_COMMA()							\
	do {										\
		token = sv_next_token(sv, &sv);			\
		assert(sv_eq_cstr(token, ","));			\
	} while (0)

#define PARSE_ABC(op)													\
	do {																\
		token = sv_next_token(sv, &sv);									\
		assert(sv_to_int(token, &a));									\
		assert(a >= INT8_MIN && a <= INT8_MAX);							\
		PARSE_COMMA();													\
		token = sv_next_token(sv, &sv);									\
		assert(sv_to_int(token, &b));									\
		assert(b >= INT8_MIN && b <= INT8_MAX);							\
		PARSE_COMMA();													\
		token = sv_next_token(sv, &sv);									\
		assert(sv_to_int(token, &c));									\
		assert(c >= INT8_MIN && c <= INT8_MAX);							\
		idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = (op), .a = a, .b = b, .c = c}}); \
	} while (0)

#define PARSE_ABx(op)													\
	do {																\
		token = sv_next_token(sv, &sv);									\
		assert(sv_to_int(token, &a));									\
		assert(a >= INT8_MIN && a <= INT8_MAX);							\
		PARSE_COMMA();													\
		token = sv_next_token(sv, &sv);									\
		if (sv_to_int(token, &b)) {										\
			assert(b >= INT16_MIN && b <= INT16_MAX);					\
		} else {														\
			push_label(restbl, token, (uintptr_t)(&vm->exe[idx]));		\
			b = 0;														\
		}																\
		idx = push_inst(vm->exe, idx, (INST){.iABx = {.i = (op), .a = a, .bx = b}}); \
	} while (0)

#define PARSE_Ax(op)													\
	do {																\
		token = sv_next_token(sv, &sv);									\
		if (sv_to_int(token, &a)) {										\
			assert(a >= INT24_MIN && a <= INT24_MAX);					\
		} else {														\
			push_label(restbl, token, (uintptr_t)(&vm->exe[idx]));		\
			a = 0;														\
		}																\
		idx = push_inst(vm->exe, idx, (INST)INST_iAx((op), a));			\
	} while (0)

void
resolve_labels(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl)
{
	uintptr_t addr1 = 0, addr2 = 0;
	StrView name;
	INST *inst = 0;
	for (size_t i = 0; i < restbl->len; ++i) {
		name = restbl->entries[i].name;
		addr1 = restbl->entries[i].addr;
		addr2 = lookup_label(deftbl, name);
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
assemble_data_segment(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl, StrView sv, StrView *ret)
{
	/* defining data:
	 * db *bytes*
	 * df *floats*
	 * dw *words*
	 * pad *padding bytes*
	 */
	int64_t word;
	double flt;
	StrView token;
	UNUSED(restbl);
	while (sv.len) {
		token = sv_next_token(sv, &sv);
		if (token.len == 0) break;
		if (sv_eq_cstr(token, "segment")) {
			*ret = sv;
			return 1;
		} else if (sv_eq_cstr(token, "db")) {
			for (;;) {
				token = sv_next_token(sv, &sv);
				if (sv_eq_cstr(token, "rep")) {
					assert(sv_to_int(sv_next_token(sv, &sv), &word));
					vm->data_end += (uint64_t)word;
					break;
				} else if (sv_eq_cstr(token, "\"")) {
					while (!try_parse_char('"', sv, &sv)) {
						assert(sv.len);
						*vm->data_end++ = *sv.str++;
						sv.len--;
					}
				} else {
					assert(sv_to_int(token, &word));
					assert(word >= INT8_MIN && word <= INT8_MAX);
					*vm->data_end++ = word;
				}
				sv = eat_ws_and_comments(sv);
				if (!try_parse_char(',', sv, &sv)) {
					break;
				}
			}
		} else if (sv_eq_cstr(token, "df")) {
			for (;;) {
				token = sv_next_token(sv, &sv);
				if (sv_eq_cstr(token, "rep")) {
					assert(sv_to_int(sv_next_token(sv, &sv), &word));
					vm->data_end += (sizeof(WORD) * (uint64_t)word);
					break;
				} else {
					assert(sv_to_f64(token, &flt));
					*((double *)vm->data_end) = flt;
					vm->data_end += sizeof(WORD);
				}
				sv = eat_ws_and_comments(sv);
				if (!try_parse_char(',', sv, &sv)) {
					break;
				}
			}
		} else if (sv_eq_cstr(token, "dw")) {
			for (;;) {
				token = sv_next_token(sv, &sv);
				if (sv_eq_cstr(token, "rep")) {
					assert(sv_to_int(sv_next_token(sv, &sv), &word));
					vm->data_end += (sizeof(WORD) * (uint64_t)word);
					break;
				} else {
					assert(sv_to_int(token, &word));
					*((int64_t *)vm->data_end) = word;
					vm->data_end += sizeof(WORD);
				}
				sv = eat_ws_and_comments(sv);
				if (!try_parse_char(',', sv, &sv)) {
					break;
				}
			}
		} else {
			push_label(deftbl, token, (uintptr_t)(vm->data_end));
			assert(sv_eq_cstr(sv_next_token(sv, &sv), ":"));
			continue;
		}
		assert(sv_eq_cstr(sv_next_token(sv, &sv), ";"));
	}
	return 0;
}

int
assemble_executable_segment(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl, StrView sv, StrView *ret)
{
	int64_t a, b, c;
	WORD w;
	size_t idx = 0;
	StrView token;
	while (sv.len) {
		token = sv_next_token(sv, &sv);
		if (token.len == 0) {
			*ret = sv;
			return 1;
		}
		int i = 0;
		for (; i < OP_COUNT; ++i) {
			if (sv_eq(token, sv_from_cstr(opcode_tostr(i)))) {
				switch ((enum opcode)i) {
				case op_nop: {
					idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i}});
				} break;
				case op_halt:
				case op_ldi:
				case op_alloca:
				case op_malloc:
				case op_jmp: {
					PARSE_Ax(i);
				} break;
				case op_ldw: {
					token = sv_next_token(sv, &sv);
					assert(sv_to_int(token, &a));
					assert(a >= INT8_MIN && a <= INT8_MAX);
					PARSE_COMMA();
					token = sv_next_token(sv, &sv);
					if (sv_to_int(token, &c)) {
						w = (WORD){.as_i64 = c};
						idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = op_ldw, .a = a}});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = w.as_pair[0]});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = w.as_pair[1]});
					} else {
						push_label(restbl, token, (uintptr_t)(&vm->exe[idx]));
						idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i, .a = a}});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = 0});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = 0});
					}
				} break;
				case op_ldf: {
					token = sv_next_token(sv, &sv);
					assert(sv_to_int(token, &a));
					assert(a >= INT8_MIN && a <= INT8_MAX);
					PARSE_COMMA();
					token = sv_next_token(sv, &sv);
					assert(sv_to_f64(token, &w.as_f64));
					idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i, .a = a}});
					idx = push_inst(vm->exe, idx, (INST){.as_i32 = w.as_pair[0]});
					idx = push_inst(vm->exe, idx, (INST){.as_i32 = w.as_pair[1]});
				} break;
				case op_lfp:
				case op_lsp:
				case op_lip:
				case op_call:
				case op_puts: {
					token = sv_next_token(sv, &sv);
					assert(sv_to_int(token, &a));
					assert(a >= INT8_MIN && a <= INT8_MAX);
					idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i, .a = a}});
				} break;
				case op_enter:
				case op_leave: {
					idx = push_inst(vm->exe, idx, (INST)INST_iAx(i, 0));
				} break;
				case op_mov: {
					token = sv_next_token(sv, &sv);
					assert(sv_to_int(token, &a));
					assert(a >= INT8_MIN && a <= INT8_MAX);
					PARSE_COMMA();
					token = sv_next_token(sv, &sv);
					assert(sv_to_int(token, &b));
					assert(b >= INT8_MIN && b <= INT8_MAX);
					idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i, .a = a, .b = b}});
				} break;
				case op_fetch:
				case op_store:
				case op_fetchb:
				case op_storeb:
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
				case op_jez:
				case op_jnz:
				case op_jlz:
				case op_jlez:
				case op_jgz:
				case op_jgez: {
					PARSE_ABx(i);
				} break;
				case op_call0: {
					token = sv_next_token(sv, &sv);
					if (sv_to_int(token, &c)) {
						w = (WORD){.as_i64 = c};
						idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i}});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = w.as_pair[0]});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = w.as_pair[1]});
					} else {
						push_label(restbl, token, (uintptr_t)(&vm->exe[idx]));
						idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i}});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = 0});
						idx = push_inst(vm->exe, idx, (INST){.as_i32 = 0});
					}
				} break;
				case op_ret: {
					idx = push_inst(vm->exe, idx, (INST){.iABC = {.i = i}});
				} break;
				case OP_COUNT:
				default: assert(!"Invalid instruction");
				}
				break;
			}
		}
		if (i == OP_COUNT) {
			if (sv_eq_cstr(token, "segment")) {
			} else if (sv_eq_cstr(token, "entry")) {
				token = sv_next_token(sv, &sv);
				push_label(restbl, token, 0);
			} else {
				push_label(deftbl, token, (uintptr_t)(&vm->exe[idx]));
				printf("%.*s\n", (int)token.len, token.str);
				assert(sv_eq_cstr(sv_next_token(sv, &sv), ":"));
			}
		}
		assert(sv_eq_cstr(sv_next_token(sv, &sv), ";"));
	}
	*ret = sv;
	return 0;
}

int
assemble(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl, StrView sv)
{
	int ds = 0, es = 0;
	StrView token;
	assert(sv_eq_cstr(sv_next_token(sv, &sv), "segment"));
	while (sv.len) {
		token = sv_next_token(sv, &sv);
		if (token.len == 0) break;
		if (sv_eq_cstr(token, "executable")) {
			assert(sv_eq_cstr(sv_next_token(sv, &sv), ":"));
			assert(es == 0);
			es = 1;
			if (assemble_executable_segment(vm, deftbl, restbl, sv, &sv)) {
				continue;
			} else {
				break;
			}
		} else if (sv_eq_cstr(token, "data")) {
			assert(sv_eq_cstr(sv_next_token(sv, &sv), ":"));
			assert(ds == 0);
			ds = 1;
			vm->data = GC_MALLOC(DATA_SZ);
			vm->data_end = vm->data;
			if (assemble_data_segment(vm, deftbl, restbl, sv, &sv)) {
				continue;
			} else {
				break;
			}
		} else {
			assert(!"Error Unknown segment type");
		}
	}
	resolve_labels(vm, deftbl, restbl);
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

int
main(void)
{
	GC_INIT();
	printf("OP_COUNT = %d\n", OP_COUNT);
	Vm vm = {0};
	vm_init(&vm);
	LabelTbl *deftbl = make_labeltbl();
	LabelTbl *restbl = make_labeltbl();
	StrView sv_asm = sv_map_file("test.yasm");
	assemble(&vm, deftbl, restbl, sv_asm);
	if (vm_run(&vm) < 0) {
		fprintf(stderr, "Error trap signalled: %s\n", trapcode_tostr(vm.status));
	}
	return 0;
}
