/**************************************************
 * ASM 8086 parser data types.
 **************************************************/

#pragma once

#include <cstdint>

#define u8 uint8_t
#define i8 int8_t
#define u16 uint16_t
#define i16 int16_t

static constexpr u16 concat(u8 hi, u8 lo) 
{ 
  return ((u16)hi << 8) | (u16)lo; 
}

// Common first bytes of two-byte opcodes. These occupy the lower byte in order to facilitate matching against them
// during parsing.
constexpr u16 FB_0x80 = 0x80; // 1000 0000
constexpr u16 FB_0xff = 0xff; // 1111 1111
constexpr u16 FB_0xfe = 0xfe; // 1111 1110
constexpr u16 FB_0xf6 = 0xf6; // 1111 0110
constexpr u16 FB_0xd0 = 0xd0; // 1101 0000
constexpr u16 FB_0xd4 = 0xd4; // 1101 0100
constexpr u16 FB_0xd5 = 0xd5; // 1101 0101
constexpr u16 FB_0xc6 = 0xc6; // 1100 0110
constexpr u16 FB_0x8e = 0x8e; // 1000 1110

enum op_t : u16 
{
  /** mov */
  // concat indicates that the instruction requires two bytes to fully represent.
  MOV_IMM_MEM    = concat(FB_0xc6, 0x00),      // 1100 0110 0000 0000
  MOV_IMM_REG    = 0xb0,                       // .... .... 1011 0000
  MOV_RM         = 0x88,                       // .... .... 1000 1000
  MOV_MEM_TO_ACC = 0xa0,                       // .... .... 1010 0000
  MOV_ACC_TO_MEM = 0xa2,                       // .... .... 1010 0010

  /** push */
  PUSH_RM        = concat(FB_0xff, 0x30),      // 1111 1111 0011 0000
  PUSH_REG       = 0x50,                       // .... .... 0101 0000
  PUSH_SEG_ES    = 0x06,                       // .... .... 0000 0110
  PUSH_SEG_CS    = 0x0e,                       // .... .... 0000 1110
  PUSH_SEG_SS    = 0x16,                       // .... .... 0001 0110
  PUSH_SEG_DS    = 0x1e,                       // .... .... 0001 1110

  /** pop */
  POP_RM         = 0x8f,                       // .... .... 1000 1111
  POP_REG        = 0x58,                       // .... .... 0101 1000
  POP_SEG_ES     = 0x07,                       // .... .... 0000 0111
  POP_SEG_SS     = 0x17,                       // .... .... 0001 0111
  POP_SEG_DS     = 0x1f,                       // .... .... 0001 1111

  /** xchg */
  XCHG_RM        = 0x86,                       // .... .... 1000 0110
  XCHG_REG_ACC   = 0x90,                       // .... .... 1001 0000

  /** in/out */
  IN_FIX         = 0xe4,                       // .... .... 1110 0100
  IN_VAR         = 0xec,                       // .... .... 1110 1100
  OUT_FIX        = 0xe6,                       // .... .... 1110 0110
  OUT_VAR        = 0xee,                       // .... .... 1110 1110

  /** output to */
  XLAT           = 0xd7,                       // .... .... 1101 0111
  LEA            = 0x8d,                       // .... .... 1000 1101
  LDS            = 0xc5,                       // .... .... 1100 0101
  LES            = 0xc4,                       // .... .... 1100 0100
  LAHF           = 0x9f,                       // .... .... 1001 1111
  SAHF           = 0x9e,                       // .... .... 1001 1110
  PUSHF          = 0x9c,                       // .... .... 1001 1100
  POPF           = 0x9d,                       // .... .... 1001 1101

  /** add */
  ADD_RM         = 0x00,                       // .... .... 0000 0000
  ADD_IMM_RM     = concat(FB_0x80, 0x00),      // 1000 0000 0000 0000
  ADD_IMM_ACC    = 0x04,                       // .... .... 0000 0100

  /** add with carry */
  ADC_RM         = 0x10,                       // .... .... 0001 0000
  ADC_IMM_RM     = concat(FB_0x80, 0x10),      // 1000 0000 0001 0000
  ADC_IMM_ACC    = 0x14,                       // .... .... 0001 0100

  /** increment */
  INC_RM         = concat(FB_0xfe, 0x00),      // 1111 1110 0000 0000
  INC_REG        = 0x40,                       // .... .... 0100 0000
  AAA            = 0x37,                       // .... .... 0011 0111
  DAA            = 0x27,                       // .... .... 0010 0111

  /** decrement */
  DEC_REG        = 0x48,                       // .... .... 0100 1000
  DEC_RM         = concat(FB_0xfe, 0x08),      // 1111 1110 0000 1000
  NEG            = concat(FB_0xf6, 0x18),      // 1111 0110 0001 1000

  /** sub */
  SUB_RM         = 0x28,                       // .... .... 0010 1000
  SUB_IMM_ACC    = 0x2c,                       // .... .... 0010 1100
  SUB_IMM_RM     = concat(FB_0x80, 0x28),      // 1000 0000 0010 1000

  /** sub with borrow */
  SBB_RM         = 0x18,                       // .... .... 0001 1000
  SBB_IMM_ACC    = 0x1c,                       // .... .... 0001 1100
  SBB_IMM_RM     = concat(FB_0x80, 0x18),      // 1000 0000 0001 1000

  /** cmp */
  CMP_RM         = 0x38,                       // .... .... 0011 1000
  CMP_IMM_ACC    = 0x3c,                       // .... .... 0011 1100
  CMP_IMM_RM     = concat(FB_0x80, 0x38),      // 1000 0000 0011 1000
  AAS            = 0x3f,                       // .... .... 0011 1111
  DAS            = 0x2f,                       // .... .... 0010 1111

  /** mul */
  MUL            = concat(FB_0xf6, 0x20),      // 1111 0110 0010 0000
  IMUL           = concat(FB_0xf6, 0x28),      // 1111 0110 0010 1000
  AAM            = concat(FB_0xd4, 0x0a),      // 1101 0100 0000 1010
  DIV            = concat(FB_0xf6, 0x30),      // 1111 0110 0011 0000
  IDIV           = concat(FB_0xf6, 0x38),      // 1111 0110 0011 1000
  AAD            = concat(FB_0xd5, 0x0a),      // 1101 0101 0000 1010
  CBW            = 0x98,                       // .... .... 1001 1000
  CWD            = 0x99,                       // .... .... 1001 1001

  /** logic */
  NOT            = concat(FB_0xf6, 0x10),      // 1111 0110 0001 0000
  SHL            = concat(FB_0xd0, 0x20),      // 1101 0000 0010 0000
  SHR            = concat(FB_0xd0, 0x28),      // 1101 0000 0010 1000
  SAR            = concat(FB_0xd0, 0x38),      // 1101 0000 0011 1000
  ROL            = concat(FB_0xd0, 0x00),      // 1101 0000 0000 0000
  ROR            = concat(FB_0xd0, 0x08),      // 1101 0000 0000 1000
  RCL            = concat(FB_0xd0, 0x10),      // 1101 0000 0001 0000
  RCR            = concat(FB_0xd0, 0x18),      // 1101 0000 0001 1000

  AND_RM         = 0x20,                       // .... .... 0010 0000
  AND_IMM_ACC    = 0x24,                       // .... .... 0010 0100
  AND_IMM_RM     = concat(FB_0x80, 0x20),      // 1000 0000 0010 0000

  TEST_RM        = 0x84,                       // .... .... 1000 0100
  TEST_IMM_ACC   = 0xa8,                       // .... .... 1010 1000
  TEST_IMM_RM    = concat(FB_0xf6, 0x00),      // 1111 0110 0000 0000

  OR_RM          = 0x08,                       // .... .... 0000 1000
  OR_IMM_ACC     = 0x0c,                       // .... .... 0000 1100
  OR_IMM_RM      = concat(FB_0x80, 0x08),      // 1000 0000 1000 0000

  XOR_RM         = 0x30,                       // .... .... 0011 0000
  XOR_IMM_ACC    = 0x34,                       // .... .... 0011 0100
  XOR_IMM_RM     = concat(FB_0x80, 0x30),      // 0011 0100 0011 1000

  /** string manipulation */
  REP            = 0xf2,                       // .... .... 1111 0010
  MOVS           = 0xa4,                       // .... .... 1010 0100
  CMPS           = 0xa6,                       // .... .... 1010 0110
  SCAS           = 0xae,                       // .... .... 1010 1110
  LODS           = 0xac,                       // .... .... 1010 1100
  STOS           = 0xaa,                       // .... .... 1010 1010

  /** control transfer */
  CALL_DIR_SEG        = 0xe8,                  // .... .... 1110 1000
  CALL_DIR_INTERSEG   = 0x9a,                  // .... .... 1001 1010
  CALL_INDIR_SEG      = concat(FB_0xff, 0x10), // 1111 1111 0001 0000
  CALL_INDIR_INTERSEG = concat(FB_0xff, 0x18), // 1111 1111 0001 1000

  /** unconditional jump */
  JMP_DIR_SEG        = 0xe9,                   // .... .... 1110 1001
  JMP_DIR_SEG_SHORT  = 0xeb,                   // .... .... 1110 1011
  JMP_DIR_INTERSEG   = 0xea,                   // .... .... 1110 1010
  JMP_INDIR_SEG      = concat(FB_0xff, 0x20),  // 1111 1111 0010 0000
  JMP_INDIR_INTERSEG = concat(FB_0xff, 0x28),  // 1111 1111 0010 1000

  /** return from call */
  RET_SEG                = 0xc3,               // .... .... 1100 0011
  RET_SEG_IMM_TO_SP      = 0xc2,               // .... .... 1100 0010
  RET_INTERSEG           = 0xcb,               // .... .... 1100 1011
  RET_INTERSEG_IMM_TO_SP = 0xca,               // .... .... 1100 1010

  /** conditional jump */
  JE             = 0x74,                       // .... .... 0111 0100
  JL             = 0x7c,                       // .... .... 0111 1100
  JLE            = 0x7e,                       // .... .... 0111 1110
  JB             = 0x72,                       // .... .... 0111 0010
  JBE            = 0x76,                       // .... .... 0111 0110
  JP             = 0x7a,                       // .... .... 0111 1010
  JO             = 0x70,                       // .... .... 0111 0000
  JS             = 0x78,                       // .... .... 0111 1000
  JNE            = 0x75,                       // .... .... 0111 0101
  JNL            = 0x7d,                       // .... .... 0111 1101
  JNLE           = 0x7f,                       // .... .... 0111 1111
  JNB            = 0x73,                       // .... .... 0111 0011
  JNBE           = 0x77,                       // .... .... 0111 0111
  JNP            = 0x7b,                       // .... .... 0111 1011
  JNO            = 0x71,                       // .... .... 0111 0001
  JNS            = 0x79,                       // .... .... 0111 1001
  LOOP           = 0xe2,                       // .... .... 1110 0010
  LOOPZ          = 0xe1,                       // .... .... 1110 0001
  LOOPNZ         = 0xe0,                       // .... .... 1110 0000
  JCXZ           = 0xe3,                       // .... .... 1110 0011

  /** interrupt */
  INT            = 0xcd,                       // .... .... 1100 1101
  INT3           = 0xcc,                       // .... .... 1100 1100
  INTO           = 0xce,                       // .... .... 1100 1110
  IRET           = 0xcf,                       // .... .... 1100 1111

  /** processor control */
  CLC            = 0xf8,                       // .... .... 1111 1000
  CMC            = 0xf5,                       // .... .... 1111 0101
  STC            = 0xf9,                       // .... .... 1111 1001
  CLD            = 0xfc,                       // .... .... 1111 1100
  STD            = 0xfd,                       // .... .... 1111 1101
  CLI            = 0xfa,                       // .... .... 1111 1010
  STI            = 0xfb,                       // .... .... 1111 1011
  HLT            = 0xf4,                       // .... .... 1111 0100
  WAIT           = 0x9b,                       // .... .... 1001 1011
  ESC            = 0xd8,                       // .... .... 1101 1000
  LOCK           = 0xf0,                       // .... .... 1111 0000

  OP_INVALID,
};

/**
 * The opcode masks are used to select chunks of a byte that may contain various opcode values. These must be checked in
 * descending order to ensure we don't prematurely truncate an opcade chunk and mistake it for the wrong opcode value.
 * For example, this can happen with accumulator mov, where the high 6 bits of mem to acc and acc to mem are the same.
 */
static constexpr u16 OPCODE_MASKS[] 
{
  0xffff,  // ... 1111
  0xfffe,  // ... 1110
  0xfffc,  // ... 1100
  0xfff8,  // ... 1000
  0xfff0,  // ... 0000
};

/**
 * These masks are used to extract the opcode-relevant bits from the second byte. These should be checked in descending
 * order, similarly to {@OPCODE_MASKS}.
 */
static constexpr u8 SECOND_BYTE_MASKS[] 
{
  0xff, // 1111 1111
  0x38, // 0011 1000
  0x20, // 0010 0000
};

enum mod_t : u8 
{
  MOD_MEM_NO_DISP,
  MOD_MEM_BYTE_DISP,
  MOD_MEM_WORD_DISP,
  MOD_REG_NO_DISP,

  MOD_INVALID,
};

enum half_register_t    : u8 { AL, CL, DL, BL, AH, CH, DH, BH };
enum wide_register_t    : u8 { AX, CX, DX, BX, SP, BP, SI, DI };
enum segment_register_t : u8 { ES, CS, SS, DS };