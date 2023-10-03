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

#define STACK_SZ 1024
#define PRG_SZ   1024
#define STACK_END_PTR(stack) ((stack) + STACK_SZ)
#define INST_iAx(op, ax) {.as_u32 = ((ax) << 8)|(op)}
#define INT24_MAX (1L << 23)
#define INT24_MIN (-INT24_MAX)

typedef union _word {
	int64_t as_i64;
	uint64_t as_u64;
	double as_f64;
	union _word *as_ptr;
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
	INST *prg;
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
	op_eq,        /* iABC fp[A] := fp[B] == fp[C] */
	op_jmp,       /* iAx  ip :=  ip + Ax */
	op_jez,       /* iABx if fp[A] == 0 then ip := ip + Bx */
	op_jnz,
	op_jlz,
	op_jlez,
	op_jgz,
	op_jgez,
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
StrView sv_trim_ws(StrView sv);
LabelTbl *make_labeltbl(void);
void push_label(LabelTbl *tbl, StrView name, uintptr_t addr);
uintptr_t lookup_label(LabelTbl *tbl, StrView name);
int chin(int ch, char *cstr);
int islabelchar(int ch);
int try_parse_char(int ch, StrView sv, StrView *ret);
int try_parse_str(char *cstr, StrView sv, StrView *ret);
int try_parse_int(StrView sv, StrView *sv_ret, int64_t *ret);
int try_parse_label(StrView sv, StrView *ret, StrView *label_name);
void resolve_labels(LabelTbl *deftbl, LabelTbl *restbl);
int assemble(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl, StrView sv);

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
	case op_eq: return "eq";
	case op_jmp: return "jmp";
	case op_jez: return "jez";
	case op_jnz: return "jnz";
	case op_jlz: return "jlz";
	case op_jlez: return "jlez";
	case op_jgz: return "jgz";
	case op_jgez: return "jgez";
	default: assert(!"Invalid opcode");
	}
	return NULL;
}

void
vm_init(Vm *vm)
{
	vm->stack = GC_MALLOC(sizeof(vm->stack[0]) * STACK_SZ);
	vm->prg = GC_MALLOC(sizeof(vm->prg[0]) * PRG_SZ);
	vm->fp = vm->stack;
	vm->sp = vm->stack;
	vm->ip = vm->prg;
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
	printf("(push_lable) label: %.*s\n", (int)name.len, name.str);
	if (name.len == 0) {
		assert(!"what is happening?");
	}
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
	printf("label: %.*s\n", (int)name.len, name.str);
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

static int
isdelim(int ch)
{
	return isspace(ch) || chin(ch, "+-*&^#!,/\"'[]{}()|=:;`~");
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
	*ret = sv;
	return 1;
}

int
try_parse_int(StrView sv, StrView *sv_ret, int64_t *ret)
{
	StrView sv_int = {
		.len = 0,
		.str = sv.str,
	};
	if (sv.len && (isdigit(*sv.str) || *sv.str == '-')) {
		sv.len--;
		sv.str++;
		sv_int.len++;
	} else {
		return 0;
	}
	while (sv.len && isdigit(*sv.str)) {
		sv.len--;
		sv.str++;
		sv_int.len++;
	}
	if (sv.len == 0 || !isdelim(*sv.str)) {
		return 0;
	}
	*ret = atol(sv_int.str);
	*sv_ret = sv;
	return 1;
}

int
try_parse_label(StrView sv, StrView *ret, StrView *label_name)
{
	StrView name = {
		.len = 0,
		.str = sv.str,
	};
	if (sv.len && isdigit(*sv.str)) {
		return 0;
	}
	while (sv.len && islabelchar(*sv.str)) {
		sv.len--;
		sv.str++;
		name.len++;
	}
	*ret = sv;
	*label_name = name;
	return 1;
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
		sv = eat_ws_and_comments(sv);			\
		assert(try_parse_char(',', sv, &sv));	\
		sv = eat_ws_and_comments(sv);			\
	} while (0)

#define PARSE_ABC(op)													\
	do {																\
		sv = eat_ws_and_comments(sv);									\
		assert(try_parse_int(sv, &sv, &a));								\
		assert(a >= INT8_MIN && a <= INT8_MAX);							\
		PARSE_COMMA();													\
		assert(try_parse_int(sv, &sv, &b));								\
		assert(b >= INT8_MIN && b <= INT8_MAX);							\
		PARSE_COMMA();													\
		assert(try_parse_int(sv, &sv, &c));								\
		assert(c >= INT8_MIN && c <= INT8_MAX);							\
		idx = push_inst(vm->prg, idx, (INST){.iABC = {.i = (op), .a = a, .b = b, .c = c}}); \
	} while (0)

#define PARSE_ABx(op)													\
	do {																\
		sv = eat_ws_and_comments(sv);									\
		assert(try_parse_int(sv, &sv, &a));								\
		assert(a >= INT8_MIN && a <= INT8_MAX);							\
		PARSE_COMMA();													\
		if (try_parse_int(sv, &sv, &b)) {								\
			assert(b >= INT16_MIN && b <= INT16_MAX);					\
		} else {														\
			assert(try_parse_label(sv, &sv, &label_name));				\
			push_label(restbl, label_name, (uintptr_t)(&vm->prg[idx]));	\
			b = 0;														\
		}																\
		idx = push_inst(vm->prg, idx, (INST){.iABx = {.i = (op), .a = a, .bx = b}}); \
	} while (0)

#define PARSE_Ax(op)													\
	do {																\
		sv = eat_ws_and_comments(sv);									\
		if (try_parse_int(sv, &sv, &a)) {								\
			assert(a >= INT24_MIN && a <= INT24_MAX);					\
		} else {														\
			assert(try_parse_label(sv, &sv, &label_name));				\
			push_label(restbl, label_name, (uintptr_t)(&vm->prg[idx]));	\
			a = 0;														\
		}																\
		idx = push_inst(vm->prg, idx, (INST)INST_iAx((op), a));			\
	} while (0)

void
resolve_labels(LabelTbl *deftbl, LabelTbl *restbl)
{
	uintptr_t addr1 = 0, addr2 = 0;
	StrView name;
	INST *inst = 0;
	for (size_t i = 0; i < restbl->len; ++i) {
		name = restbl->entries[i].name;
		printf("label: %.*s\n", (int)name.len, name.str);
		addr1 = restbl->entries[i].addr;
		addr2 = lookup_label(deftbl, name);
		inst = (INST *)addr1;
		int64_t offset = (INST *)addr2 - ((INST *)addr1 + 1);
		int op = inst->iABC.i;
		printf("offset = %ld\n", offset);
		if (op == op_jmp) {
			assert(offset >= INT24_MIN && offset <= INT24_MAX);
			*inst = (INST)INST_iAx(op, offset);
		} else if (IS_JCC(op)) {
			assert(offset >= INT16_MIN && offset <= INT16_MAX);
			inst->iABx.bx = offset;
		} else {
			assert(!"invalid opcode");
		}
	}
}

int
assemble(Vm *vm, LabelTbl *deftbl, LabelTbl *restbl, StrView sv)
{
	int64_t a, b, c;
	StrView label_name;
	size_t idx = 0;
	while (sv.len) {
		sv = eat_ws_and_comments(sv);
		if (sv.len == 0) break;
		if (try_parse_str("nop", sv, &sv)) {
			idx = push_inst(vm->prg, idx, (INST)INST_iAx(op_nop, 0));
		} else if (try_parse_str("halt", sv, &sv)) {
			PARSE_Ax(op_halt);
		} else if (try_parse_str("ldi", sv, &sv)) {
			PARSE_ABx(op_ldi);
		} else if (try_parse_str("lfp", sv, &sv)) {
			sv = eat_ws_and_comments(sv);
			assert(try_parse_int(sv, &sv, &a));
			assert(a >= INT8_MIN && a <= INT8_MAX);
			idx = push_inst(vm->prg, idx, (INST){.iABC = {.i = op_lfp, .a = a}});
		} else if (try_parse_str("lsp", sv, &sv)) {
			sv = eat_ws_and_comments(sv);
			assert(try_parse_int(sv, &sv, &a));
			assert(a >= INT8_MIN && a <= INT8_MAX);
			idx = push_inst(vm->prg, idx, (INST){.iABC = {.i = op_lsp, .a = a}});
		} else if (try_parse_str("lip", sv, &sv)) {
			sv = eat_ws_and_comments(sv);
			assert(try_parse_int(sv, &sv, &a));
			assert(a >= INT8_MIN && a <= INT8_MAX);
			idx = push_inst(vm->prg, idx, (INST){.iABC = {.i = op_lip, .a = a}});
		} else if (try_parse_str("alloca", sv, &sv)) {
			PARSE_Ax(op_alloca);
		} else if (try_parse_str("enter", sv, &sv)) {
			idx = push_inst(vm->prg, idx, (INST)INST_iAx(op_enter, 0));
		} else if (try_parse_str("leave", sv, &sv)) {
			idx = push_inst(vm->prg, idx, (INST)INST_iAx(op_leave, 0));
		} else if (try_parse_str("mov", sv, &sv)) {
			sv = eat_ws_and_comments(sv);		\
			assert(try_parse_int(sv, &sv, &a));
			assert(a >= INT8_MIN && a <= INT8_MAX);
			PARSE_COMMA();
			assert(try_parse_int(sv, &sv, &b));
			assert(b >= INT8_MIN && b <= INT8_MAX);
			idx = push_inst(vm->prg, idx, (INST){.iABC = {.i = op_mov, .a = a, .b = b}});
		} else if (try_parse_str("fetch", sv, &sv)) {
			PARSE_ABC(op_fetch);
		} else if (try_parse_str("store", sv, &sv)) {
			PARSE_ABC(op_store);
		} else if (try_parse_str("addp", sv, &sv)) {
			PARSE_ABC(op_addp);
		} else if (try_parse_str("subp", sv, &sv)) {
			PARSE_ABC(op_subp);
		} else if (try_parse_str("and", sv, &sv)) {
			PARSE_ABC(op_and);
		} else if (try_parse_str("or", sv, &sv)) {
			PARSE_ABC(op_or);
		} else if (try_parse_str("xor", sv, &sv)) {
			PARSE_ABC(op_xor);
		} else if (try_parse_str("shl", sv, &sv)) {
			PARSE_ABC(op_shl);
		} else if (try_parse_str("shr", sv, &sv)) {
			PARSE_ABC(op_shr);
		} else if (try_parse_str("addi", sv, &sv)) {
			PARSE_ABC(op_addi);
		} else if (try_parse_str("subi", sv, &sv)) {
			PARSE_ABC(op_subi);
		} else if (try_parse_str("muli", sv, &sv)) {
			PARSE_ABC(op_muli);
		} else if (try_parse_str("divi", sv, &sv)) {
			PARSE_ABC(op_divi);
		} else if (try_parse_str("mod", sv, &sv)) {
			PARSE_ABC(op_mod);
		} else if (try_parse_str("eq", sv, &sv)) {
			PARSE_ABC(op_eq);
		} else if (try_parse_str("jmp", sv, &sv)) {
			PARSE_Ax(op_jmp);
		} else if (try_parse_str("jez", sv, &sv)) {
			PARSE_ABx(op_jez);
		} else if (try_parse_str("jnz", sv, &sv)) {
			PARSE_ABx(op_jnz);
		} else if (try_parse_str("jlz", sv, &sv)) {
			PARSE_ABx(op_jlz);
		} else if (try_parse_str("jlez", sv, &sv)) {
			PARSE_ABx(op_jlez);
		} else if (try_parse_str("jgz", sv, &sv)) {
			PARSE_ABx(op_jgz);
		} else if (try_parse_str("jgez", sv, &sv)) {
			PARSE_ABx(op_jgez);
		} else if (try_parse_label(sv, &sv, &label_name)) {
			sv = eat_ws_and_comments(sv);
			assert(try_parse_char(':', sv, &sv));
			push_label(deftbl, label_name, (uintptr_t)(&vm->prg[idx]));
			continue;
		}
		sv = eat_ws_and_comments(sv);
		assert(try_parse_char(';', sv, &sv));
	}
	resolve_labels(deftbl, restbl);
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
		printf("%d\t% 16ld; %- 12g; %#lx\n", i++, value.as_i64, value.as_f64, value.as_i64);
	}
}

INST prg[] = {
	[0] = {.iABC = {.i = op_enter}},
	[1] = INST_iAx(op_alloca, 6),
	[2] = {.iABx = {.i = op_ldi, .a = 0, .bx = 0}},
	[3] = {.iABx = {.i = op_ldi, .a = 1, .bx = 1}},
	[4] = {.iABx = {.i = op_ldi, .a = 3, .bx = 0}},
	[5] = {.iABx = {.i = op_ldi, .a = 4, .bx = 1}},
	[6] = {.iABx = {.i = op_ldi, .a = 5, .bx = 20}},
	// start of loop
	[7] = {.iABC = {.i = op_addi, .a = 2, .b = 0, .c = 1}},
	[8] = {.iABC = {.i = op_mov, .a = 0, .b = 1}},
	[9] = {.iABC = {.i = op_mov, .a = 1, .b = 2}},
	[10] = {.iABC = {.i = op_addi, .a = 3, .b = 3, .c = 4}},
	[11] = {.iABC = {.i = op_eq, .a = 2, .b = 3, .c = 5}},
	[12] = {.iABx = {.i = op_jez, .a = 2, .bx = -6}},
	{.iABC = {.i = op_halt}},
};

char fib_asm[] = {
	"enter;"
	"alloca 6;"
	"ldi 0, 0;"
	"ldi 1, 1;"
	"ldi 3, 0;"
	"ldi 4, 1;"
	"ldi 5, 20;\n"
	"# start of loop\n"
	".loop:"
	"addi 2, 0, 1;"
	"mov 0, 1;"
	"mov 1, 2;"
	"addi 3, 3, 4;"
	"eq 2, 3, 5;"
	"jez 2, .loop;"
	"halt 0;"
};

int
main(void)
{
	GC_INIT();
	Vm vm = {0};
	vm_init(&vm);
	LabelTbl *deftbl = make_labeltbl();
	LabelTbl *restbl = make_labeltbl();
	assemble(&vm, deftbl, restbl, sv_from_cstr(fib_asm));
	if (vm_run(&vm) < 0) {
		fprintf(stderr, "Error trap signalled: %s\n", trapcode_tostr(vm.status));
	}
	return 0;
}
