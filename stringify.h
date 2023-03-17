#ifndef PERFAWARE_STRINGIFY_H
#define PERFAWARE_STRINGIFY_H

#include <iostream>

#include "parse.h"

/**************************************************
 * Forward declarations for core functions.
 **************************************************/
void print_instruction(const instruction_t &instr);

/**************************************************
 * Parser string conversion utilities.
 **************************************************/
static void dbg_print_instr(const instruction_t &instr) {
  printf("{ opcode:%d, mod:%d, rm:%x, reg:%d, wbit:%d, dbit:%d, disp:%d, data:%d }\n", instr.opcode, instr.mod, instr.rm, instr.reg, instr.wbit, instr.dbit, instr.disp, instr.data);
}

static constexpr std::string_view
// opcodes
  MOV_STR  = "mov",
  PUSH_STR = "push",
  POP_STR  = "pop",
  XCHG_STR = "xchg",
  IN_STR   = "in",
  OUT_STR  = "out",
  ADD_STR  = "add",
  ADC_STR  = "adc",
  SUB_STR  = "sub",
  SBB_STR  = "sbb",
  INC_STR  = "inc",
  AAA_STR  = "aaa",
  DAA_STR  = "daa",
  DEC_STR  = "dec",
  NEG_STR  = "neg",
  CMP_STR  = "cmp",
  AAS_STR  = "aas",
  DAS_STR  = "das",
  MUL_STR  = "mul",
  IMUL_STR = "imul",
  AAM_STR  = "aam",
  DIV_STR  = "div",
  IDIV_STR = "idiv",
  AAD_STR  = "aad",
  CBW_STR  = "cbw",
  CWD_STR  = "cwd",
  NOT_STR  = "not",
  SHL_STR  = "shl",
  SHR_STR  = "shr",
  SAR_STR  = "sar",
  ROL_STR  = "rol",
  ROR_STR  = "ror",
  RCL_STR  = "rcl",
  RCR_STR  = "rcr",
  AND_STR  = "and",
  TEST_STR = "test",
  OR_STR   = "or",
  XOR_STR  = "xor",

// string manipulation
REP_STR  = "rep",
  MOVS_STR = "movs",
  CMPS_STR = "cmps",
  SCAS_STR = "scas",
  LODS_STR = "lods",
  STOS_STR = "stos",
  CALL_STR = "call",

// output to
XLAT_STR  = "xlat",
  LEA_STR   = "lea",
  LDS_STR   = "lds",
  LES_STR   = "les",
  LAHF_STR  = "lahf",
  SAHF_STR  = "sahf",
  PUSHF_STR = "pushf",
  POPF_STR  = "popf",

// unconditional jump
JMP_STR    = "jmp",

// return from call
RET_STR    = "ret",

// conditional jump
JE_STR     = "je",
  JL_STR     = "jl",
  JLE_STR    = "jle",
  JB_STR     = "jb",
  JBE_STR    = "jbe",
  JP_STR     = "jp",
  JO_STR     = "jo",
  JS_STR     = "js",
  JNE_STR    = "jne",
  JNL_STR    = "jnl",
  JNLE_STR   = "jnle",
  JNB_STR    = "jnb",
  JNBE_STR   = "jnbe",
  JNP_STR    = "jnp",
  JNO_STR    = "jno",
  JNS_STR    = "jns",
  LOOP_STR   = "loop",
  LOOPZ_STR  = "loopz",
  LOOPNZ_STR = "loopnz",
  JCXZ_STR   = "jcxz",

// interrupt
INT_STR    = "int",
  INTO_STR   = "into",
  IRET_STR   = "iret",

// processor control
CLC_STR    = "clc",
  CMC_STR    = "cmc",
  STC_STR    = "stc",
  CLD_STR    = "cld",
  STD_STR    = "std",
  CLI_STR    = "cli",
  STI_STR    = "sti",
  HLT_STR    = "hlt",
  WAIT_STR   = "wait",
  LOCK_STR   = "lock",

// half word-length registers
AL_STR = "al",
  CL_STR = "cl",
  DL_STR = "dl",
  BL_STR = "bl",
  AH_STR = "ah",
  CH_STR = "ch",
  DH_STR = "dh",
  BH_STR = "bh",

// word-length registers
AX_STR = "ax",
  CX_STR = "cx",
  DX_STR = "dx",
  BX_STR = "bx",
  SP_STR = "sp",
  BP_STR = "bp",
  SI_STR = "si",
  DI_STR = "di",

// segment registers
CS_STR = "cs",
  DS_STR = "ds",
  ES_STR = "es",
  SS_STR = "ss";

/**
 * Convert opcode to a string representation.
 */
static std::string_view str_opcode(op_t op) {
  switch (op) {
    // mov
    case MOV_IMM_REG:    // fallthru
    case MOV_IMM_MEM:    // fallthru
    case MOV_MEM_TO_ACC: // fallthru
    case MOV_ACC_TO_MEM: // fallthru
    case MOV_RM:
      return MOV_STR;

      // push
    case PUSH_RM:     // fallthru
    case PUSH_REG:    // fallthru
    case PUSH_SEG_ES: // fallthru
    case PUSH_SEG_CS: // fallthru
    case PUSH_SEG_SS: // fallthru
    case PUSH_SEG_DS:
      return PUSH_STR;

      // pop
    case POP_RM:     // fallthru
    case POP_REG:    // fallthru
    case POP_SEG_ES: // fallthru
    case POP_SEG_SS: // fallthru
    case POP_SEG_DS:
      return POP_STR;

      // pop
    case XCHG_RM: // fallthru
    case XCHG_REG_ACC:
      return XCHG_STR;

      // in
    case IN_FIX: // fallthru
    case IN_VAR:
      return IN_STR;

      // out
    case OUT_FIX: // fallthru
    case OUT_VAR:
      return OUT_STR;

      // output to
    case XLAT: return XLAT_STR;
    case LEA: return LEA_STR;
    case LDS: return LDS_STR;
    case LES: return LES_STR;
    case LAHF: return LAHF_STR;
    case SAHF: return SAHF_STR;
    case PUSHF: return PUSHF_STR;
    case POPF: return POPF_STR;

      // add
    case ADD_RM:     // fallthru
    case ADD_IMM_RM: // fallthru
    case ADD_IMM_ACC:
      return ADD_STR;

      // add with carry
    case ADC_RM:     // fallthru
    case ADC_IMM_RM: // fallthru
    case ADC_IMM_ACC:
      return ADC_STR;

    case INC_RM: // fallthru
    case INC_REG:
      return INC_STR;

    case AAA: return AAA_STR;
    case DAA: return DAA_STR;

    case DEC_RM: // fallthru
    case DEC_REG:
      return DEC_STR;

    case NEG: return NEG_STR;

      // sub
    case SUB_RM:     // fallthru
    case SUB_IMM_RM: // fallthru
    case SUB_IMM_ACC:
      return SUB_STR;

      // sub with borrow
    case SBB_RM:     // fallthru
    case SBB_IMM_RM: // fallthru
    case SBB_IMM_ACC:
      return SBB_STR;

      // compare
    case CMP_RM:     // fallthru
    case CMP_IMM_RM: // fallthru
    case CMP_IMM_ACC:
      return CMP_STR;

    case AAS: return AAS_STR;
    case DAS: return DAS_STR;
    case MUL: return MUL_STR;
    case IMUL: return IMUL_STR;
    case AAM: return AAM_STR;
    case DIV: return DIV_STR;
    case IDIV: return IDIV_STR;
    case AAD: return AAD_STR;
    case CBW: return CBW_STR;
    case CWD: return CWD_STR;
    case NOT: return NOT_STR;
    case SHL: return SHL_STR;
    case SHR: return SHR_STR;
    case SAR: return SAR_STR;
    case ROL: return ROL_STR;
    case ROR: return ROR_STR;
    case RCL: return RCL_STR;
    case RCR: return RCR_STR;

      // logic
    case AND_RM: // fallthru
    case AND_IMM_RM: // fallthru
    case AND_IMM_ACC:
      return AND_STR;

    case TEST_RM: // fallthru
    case TEST_IMM_RM: // fallthru
    case TEST_IMM_ACC:
      return TEST_STR;

    case OR_RM: // fallthru
    case OR_IMM_RM: // fallthru
    case OR_IMM_ACC:
      return OR_STR;

    case XOR_RM: // fallthru
    case XOR_IMM_RM: // fallthru
    case XOR_IMM_ACC:
      return XOR_STR;

      // string manipulation
    case REP: return REP_STR;
    case MOVS: return MOVS_STR;
    case CMPS: return CMPS_STR;
    case SCAS: return SCAS_STR;
    case LODS: return LODS_STR;
    case STOS: return STOS_STR;

      // control transfer
    case CALL_DIR_SEG:        // fallthru
    case CALL_INDIR_SEG:      // fallthru
    case CALL_DIR_INTERSEG:   // fallthru
    case CALL_INDIR_INTERSEG:
      return CALL_STR;

      // unconditional jump
    case JMP_INDIR_SEG:      // fallthru
    case JMP_DIR_SEG:        // fallthru
    case JMP_DIR_INTERSEG:   // fallthru
    case JMP_DIR_SEG_SHORT:  // fallthru
    case JMP_INDIR_INTERSEG:
      return JMP_STR;

      // return from call
    case RET_SEG:                // fallthru
    case RET_INTERSEG:           // fallthru
    case RET_SEG_IMM_TO_SP:      // fallthru
    case RET_INTERSEG_IMM_TO_SP:
      return RET_STR;

      // conditional jump
    case JE: return JE_STR;
    case JL: return JL_STR;
    case JLE: return JLE_STR;
    case JB: return JB_STR;
    case JBE: return JBE_STR;
    case JP: return JP_STR;
    case JO: return JO_STR;
    case JS: return JS_STR;
    case JNE: return JNE_STR;
    case JNL: return JNL_STR;
    case JNLE: return JNLE_STR;
    case JNB: return JNB_STR;
    case JNBE: return JNBE_STR;
    case JNP: return JNP_STR;
    case JNO: return JNO_STR;
    case JNS: return JNS_STR;
    case LOOP: return LOOP_STR;
    case LOOPZ: return LOOPZ_STR;
    case LOOPNZ: return LOOPNZ_STR;
    case JCXZ: return JCXZ_STR;

      // interrupt
    case INT: // fallthru
    case INT3:
      return INT_STR;

    case INTO: return INTO_STR;
    case IRET: return IRET_STR;

      // processor control
    case CLC: return CLC_STR;
    case CMC: return CMC_STR;
    case STC: return STC_STR;
    case CLD: return CLD_STR;
    case STD: return STD_STR;
    case CLI: return CLI_STR;
    case STI: return STI_STR;
    case HLT: return HLT_STR;
    case WAIT: return WAIT_STR;
    case LOCK: return LOCK_STR;

    default: { throw std::invalid_argument("[str_opcode] Invalid opcode"); }
  }
}

/**
 * Convert half-word register code to a string representation.
 */
static std::string_view str_half_register(half_register_t reg) {
  switch (reg) {
    case AL: return AL_STR;
    case CL: return CL_STR;
    case DL: return DL_STR;
    case BL: return BL_STR;
    case AH: return AH_STR;
    case CH: return CH_STR;
    case DH: return DH_STR;
    case BH: return BH_STR;
    default: { throw std::invalid_argument("Invalid half register"); }
  }
}

/**
 * Convert word-length register code to a string representation.
 */
static std::string_view str_wide_register(wide_register_t reg) {
  switch (reg) {
    case AX: return AX_STR;
    case CX: return CX_STR;
    case DX: return DX_STR;
    case BX: return BX_STR;
    case SP: return SP_STR;
    case BP: return BP_STR;
    case SI: return SI_STR;
    case DI: return DI_STR;
    default: { throw std::invalid_argument("Invalid wide register"); }
  }
}

/**
 * Convert two-bit segment register code into a string representation.
 */
static inline std::string_view str_seg_register(segment_register_t reg) {
  switch (reg) {
    case ES: return ES_STR;
    case CS: return CS_STR;
    case DS: return DS_STR;
    case SS: return SS_STR;
    default: { throw std::invalid_argument("Invalid segment register"); }
  }
}

/**
 * Convert a register code to a string representation.
 *
 * @param reg Register code.
 * @param wide Whether the register is half or full word-length.
 */
static inline std::string_view str_register(u8 reg, bool wide) {
  return wide
         ? str_wide_register((wide_register_t)reg)
         : str_half_register((half_register_t)reg);
}

static std::string str_calculated_reg(const instruction_t &instr) {
  std::string result = "[";

  switch (instr.rm) {
    case 0:
      result += "bx + si";
      break;
    case 1:
      result += "bx + di";
      break;
    case 2:
      result += "bp + si";
      break;
    case 3:
      result += "bp + di";
      break;
    case 4:
      result += "si";
      break;
    case 5:
      result += "di";
      break;
    case 6:
      // We already handled the direct addressing mode special case above.
      result += "bp";
      break;
    case 7:
      result += "bx";
      break;

    default:
      throw std::invalid_argument("Invalid instruction r/m field");
  }

  // Add the displacement to the reg field. We must account for the fact that the displacement field is signed when
  // we choose an arithmetic operator.
  if (instr.disp != 0) {
    if (instr.disp < 0) result += " - ";
    else result += " + ";
    result += std::to_string(abs(instr.disp));
  }
  result += "]";

  return result;
}

/**
 * Handle mapping in Table 4-10 in the 8086 reference manual. Format calculated address and displacement.
 */
static inline std::string str_reg_mem_field_encoding(const instruction_t &instr) {
  std::string result;

  if (instr.opcode == MOV_IMM_REG) {
    // Immediate to register mov should only care about the data field.
    result = std::to_string(instr.data);

  } else if (instr.mod == MOD_REG_NO_DISP) {
    // Register to register mov without displacement can just take the register value.
    result = std::string(str_register(instr.rm, instr.wbit));

  } else if (is_direct_addressing_mode(instr.mod, instr.rm) || instr.opcode == MOV_ACC_TO_MEM || instr.opcode == MOV_MEM_TO_ACC) {
    result = "[" + std::to_string(instr.addr) + "]";

  } else {
    // Other cases involve a calculated register with potential displacement.
    result = str_calculated_reg(instr);
  }

  return result;
}

static inline void print_no_args(const instruction_t &instr) {
  std::cout << str_opcode(instr.opcode) << std::endl;
}

static inline void print_prefix(const instruction_t &instr) {
  std::cout << str_opcode(instr.opcode) << " ";
}

static inline std::string str_instr_arg_size(const instruction_t &instr) {
  // Memory modes require a size to avoid ambiguity.
  if (instr.mod == MOD_MEM_BYTE_DISP || instr.mod == MOD_MEM_WORD_DISP || instr.mod == MOD_MEM_NO_DISP) {
    return (instr.wbit) ? "word " : "byte ";
  }
  return "";
}

/**
 * Print the ASM 8086 string representation of a parsed mov instruction to stdout.
 */
static inline void print_mov(const instruction_t &instr) {
  print_prefix(instr);

  std::string
    src = str_reg_mem_field_encoding(instr),
    dst = (instr.opcode == MOV_IMM_MEM)
          ? str_instr_arg_size(instr) + std::to_string(instr.data)
          : std::string(str_register(instr.reg, instr.wbit));

  // If D bit is not set, then reg field is the src.
  if (!instr.dbit) std::swap(src, dst);

  std::cout
    << dst
    << ", "
    << src
    << std::endl;
}

static void print_push_pop(const instruction_t &instr) {
  print_prefix(instr);

  switch (instr.opcode) {
    case PUSH_REG: // fallthru
    case POP_REG:
      std::cout
        // Half register push is not supported by the ASM 8086 standard.
        << str_register(instr.reg, true)
        << std::endl;
      break;

    case PUSH_RM: // fallthru
    case POP_RM:
      std::cout
        // Byte sized push is not supported by tha ASM 8086 standard.
        << "word "
        << str_reg_mem_field_encoding(instr)
        << std::endl;
      break;

    case PUSH_SEG_ES: // fallthru
    case PUSH_SEG_CS: // fallthru
    case PUSH_SEG_SS: // fallthru
    case PUSH_SEG_DS: // fallthru
    case POP_SEG_ES:  // fallthru
    case POP_SEG_SS:  // fallthru
    case POP_SEG_DS:
      std::cout
        << str_seg_register((segment_register_t) instr.reg)
        << std::endl;
      break;

    default: throw std::invalid_argument("[print_push] Opcode not supported");
  }
}

static void print_inc(const instruction_t &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

  switch (instr.opcode) {
    case INC_REG: // fallthru
    case DEC_REG:
      std::cout << str_register(instr.reg, instr.wbit) << std::endl;
      break;

    default: {
      std::cout
        << str_instr_arg_size(instr)
        << str_reg_mem_field_encoding(instr)
        << std::endl;
      break;
    }
  }
}

static inline void print_xchg(const instruction_t &instr) {
  print_prefix(instr);

  std::string
    src= str_reg_mem_field_encoding(instr),
    dst = str_instr_arg_size(instr) + std::string(str_register(instr.reg, instr.wbit));

  // Prefer memory location as the dst
  if (instr.mod == MOD_MEM_WORD_DISP || instr.mod == MOD_MEM_BYTE_DISP || instr.mod == MOD_MEM_NO_DISP) {
    std::swap(src, dst);
  }

  std::cout
    << dst
    << ", "
    << src
    << std::endl;
}

static inline void print_in_out(const instruction_t &instr) {
  print_prefix(instr);

  std::string
    src = std::string(str_register(instr.reg, instr.wbit)),
    dst = (instr.opcode == IN_FIX || instr.opcode == OUT_FIX)
          ? std::to_string(instr.data)
          : std::string(DX_STR);

  if (instr.opcode == OUT_FIX || instr.opcode == OUT_VAR) std::swap(src, dst);

  std::cout
    << src
    << ", "
    << dst
    << std::endl;
}

static inline void print_load(const instruction_t &instr) {
  print_prefix(instr);

  std::string
    src = std::string(str_register(instr.reg, instr.wbit)),
    dst = str_reg_mem_field_encoding(instr);

  if (instr.dbit) std::swap(src, dst);

  std::cout
    << src
    << ", "
    << dst
    << std::endl;
}

static void print_add_sub_cmp(const instruction_t &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

  std::string src, dst;

  switch (instr.opcode) {
    case ADD_IMM_RM:   // fallthru
    case ADD_IMM_ACC:  // fallthru
    case ADC_IMM_RM:   // fallthru
    case ADC_IMM_ACC:  // fallthru
    case SUB_IMM_RM:   // fallthru
    case SUB_IMM_ACC:  // fallthru
    case SBB_IMM_RM:   // fallthru
    case SBB_IMM_ACC:  // fallthru
    case CMP_IMM_RM:   // fallthru
    case CMP_IMM_ACC:  // fallthru
    case AND_IMM_RM:   // fallthru
    case AND_IMM_ACC:  // fallthru
    case TEST_IMM_RM:  // fallthru
    case TEST_IMM_ACC: // fallthru
    case OR_IMM_RM:    // fallthru
    case OR_IMM_ACC:   // fallthru
    case XOR_IMM_RM:   // fallthru
    case XOR_IMM_ACC:  // fallthru
    {
      src = (instr.mod == MOD_REG_NO_DISP)
            ? str_register(instr.rm, instr.wbit)
            : str_instr_arg_size(instr) + str_reg_mem_field_encoding(instr);
      dst = std::to_string(instr.data);
      break;
    }

    default: {
      src = str_register(instr.reg, instr.wbit);
      dst = str_reg_mem_field_encoding(instr);
      break;
    }
  }

  if (instr.dbit) std::swap(src, dst);

  std::cout
    << dst
    << ", "
    << src
    << std::endl;
}

static void print_shift_rotate(const instruction_t &instr) {
  print_prefix(instr);
  std::cout
    << str_instr_arg_size(instr)
    << str_reg_mem_field_encoding(instr)
    << ", "
    << ((instr.vbit) ? std::string(CL_STR) : "1")
    << std::endl;
}

static void print_rep(const instruction_t &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " "
    << str_opcode((op_t) instr.data)
    << ((instr.wbit) ? "w" : "b")
    << std::endl;
}

static void print_control_transfer(const instruction_t &instr) {
  print_prefix(instr);

  switch (instr.opcode) {
    case CALL_INDIR_SEG:
    case JMP_INDIR_SEG:
      std::cout << str_reg_mem_field_encoding(instr);
      break;

    case RET_SEG_IMM_TO_SP:
    case RET_INTERSEG_IMM_TO_SP:
    case INT:
      std::cout << std::to_string((i16)instr.data);
      break;

    case RET_SEG:
    case RET_INTERSEG:
      break;

    case INT3:
      std::cout << "3";
      break;

    default: throw std::invalid_argument("Invalid control transfer");
  }

  std::cout << std::endl;
}

static inline void print_conditional_jump(const instruction_t &instr) {
  print_prefix(instr);
  std::cout
    << "label" + std::to_string(instr.jump_target)
    << "; "
    << std::to_string((i8)instr.data)
    << std::endl;
}

void print_instruction(const instruction_t &instr) {
  // Handle this instruction being a jump target.
  if (instr.is_labeled) printf("\nlabel%zu:\n", instr.label);

  switch (instr.opcode) {
    case MOV_IMM_REG:
    case MOV_IMM_MEM:
    case MOV_RM:
    case MOV_ACC_TO_MEM:
    case MOV_MEM_TO_ACC:
      print_mov(instr);
      break;

    case PUSH_REG:
    case PUSH_RM:
    case PUSH_SEG_ES:
    case PUSH_SEG_CS:
    case PUSH_SEG_SS:
    case PUSH_SEG_DS:
    case POP_REG:
    case POP_RM:
    case POP_SEG_ES:
    case POP_SEG_DS:
    case POP_SEG_SS:
      print_push_pop(instr);
      break;

    case XCHG_REG_ACC:
    case XCHG_RM:
      print_xchg(instr);
      break;

    case IN_FIX:
    case IN_VAR:
    case OUT_FIX:
    case OUT_VAR:
      print_in_out(instr);
      break;

    case XLAT:
    case LAHF:
    case SAHF:
    case PUSHF:
    case POPF:
    case AAA:
    case DAA:
    case AAS:
    case DAS:
    case AAM:
    case AAD:
    case CBW:
    case CWD:
    case INTO:
    case IRET:
    case CLC:
    case CMC:
    case STC:
    case CLD:
    case STD:
    case CLI:
    case STI:
    case HLT:
    case WAIT:
      print_no_args(instr);
      break;

    case LEA:
    case LDS:
    case LES:
      print_load(instr);
      break;

    case ADD_RM:
    case ADD_IMM_RM:
    case ADD_IMM_ACC:
    case ADC_RM:
    case ADC_IMM_RM:
    case ADC_IMM_ACC:
    case SUB_RM:
    case SUB_IMM_RM:
    case SUB_IMM_ACC:
    case SBB_RM:
    case SBB_IMM_RM:
    case SBB_IMM_ACC:
    case CMP_RM:
    case CMP_IMM_RM:
    case CMP_IMM_ACC:
    case AND_RM:
    case AND_IMM_RM:
    case AND_IMM_ACC:
    case TEST_RM:
    case TEST_IMM_RM:
    case TEST_IMM_ACC:
    case OR_RM:
    case OR_IMM_RM:
    case OR_IMM_ACC:
    case XOR_RM:
    case XOR_IMM_RM:
    case XOR_IMM_ACC:
      print_add_sub_cmp(instr);
      break;

    case INC_RM:
    case INC_REG:
    case DEC_RM:
    case DEC_REG:
    case NEG:
    case MUL:
    case IMUL:
    case DIV:
    case IDIV:
    case NOT:
      print_inc(instr);
      break;

    case SHL:
    case SHR:
    case SAR:
    case ROL:
    case ROR:
    case RCL:
    case RCR:
      print_shift_rotate(instr);
      break;

    case REP:
      print_rep(instr);
      break;

    case CALL_INDIR_SEG:
    case JMP_INDIR_SEG:
    case RET_SEG:
    case RET_INTERSEG:
    case RET_SEG_IMM_TO_SP:
    case RET_INTERSEG_IMM_TO_SP:
    case INT:
    case INT3:
      print_control_transfer(instr);
      break;

    case JE:
    case JL:
    case JLE:
    case JB:
    case JBE:
    case JP:
    case JO:
    case JS:
    case JNE:
    case JNL:
    case JNLE:
    case JNB:
    case JNBE:
    case JNP:
    case JNO:
    case JNS:
    case LOOP:
    case LOOPZ:
    case LOOPNZ:
    case JCXZ:
      print_conditional_jump(instr);
      break;

    case LOCK:
      print_prefix(instr);
      break;

    default: {
      dbg_print_instr(instr);
      throw std::invalid_argument("No print function associated with opcode");
    }
  }
}

#endif //PERFAWARE_STRINGIFY_H
