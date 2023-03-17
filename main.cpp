#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
#include <optional>

#include "peeking_iterator.h"

#define u8 uint8_t
#define i8 int8_t
#define u16 uint16_t
#define i16 int16_t

/**************************************************
 * Helper functions to parse ASM 8086 binary.
 *
 * Assume little-endian.
 **************************************************/
static constexpr u8 ASM8086_W_BIT       = 0x01;
static constexpr u8 ASM8086_Z_BIT       = ASM8086_W_BIT;
static constexpr u8 ASM8086_D_BIT       = 0x02;
static constexpr u8 ASM8086_V_BIT       = ASM8086_D_BIT;
static constexpr u8 ASM8086_S_BIT       = ASM8086_D_BIT;
static constexpr u8 ASM8086_MODE_OFFSET = 6;
static constexpr u8 ASM8086_MODE_MASK   = (0x03 << ASM8086_MODE_OFFSET);
static constexpr u8 ASM8086_REG_MASK    = 0x07;
static constexpr u8 ASM8086_RM_MASK     = 0x07;

constexpr static inline u16 concat(u8 hi, u8 lo) { return (static_cast<u16>(hi) << 8) | static_cast<u16>(lo); }

// Avoid macros in favor of inline so we can let the compiler "do the right thing".
static inline bool asm8086_wbit(u8 byte, u8 offset)    { return !!(((ASM8086_W_BIT << offset) & byte) >> offset); }
static inline bool asm8086_wbit(u8 byte)               { return asm8086_wbit(byte, 0); }
static inline bool asm8086_dbit(u8 byte)               { return !!(ASM8086_D_BIT & byte); }
static inline bool asm8086_vbit(u8 byte)               { return !!(ASM8086_V_BIT & byte); }
static inline bool asm8086_sbit(u8 byte)               { return !!(ASM8086_S_BIT & byte); }
static inline bool asm8086_zbit(u8 byte)               { return !!(ASM8086_Z_BIT & byte); }
static inline u8   asm8086_mode(u8 byte)               { return (ASM8086_MODE_MASK & byte) >> ASM8086_MODE_OFFSET; }
static inline u8   asm8086_reg(u8 byte, u8 offset)     { return ((ASM8086_REG_MASK << offset) & byte) >> offset; }
static inline u8   asm8086_reg(u8 byte)                { return asm8086_reg(byte, 0); }
static inline u8   asm8086_reg_seg(u8 byte, u8 offset) { return 0x03 & asm8086_reg(byte, offset); }
static inline u8   asm8086_reg_seg(u8 byte)            { return asm8086_reg_seg(byte, 0); }
static inline u8   asm8086_rm(u8 byte)                 { return ASM8086_RM_MASK & byte; }
static inline u8   asm8086_op_secondary(u8 byte)       { return asm8086_reg(byte, 3); }

/**************************************************
 * ASM 8086 parser data types.
 **************************************************/
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

enum op_t : u16 {
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
static constexpr u16 OPCODE_MASKS[] {
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
static constexpr u8 SECOND_BYTE_MASKS[] {
  0xff, // 1111 1111
  0x38, // 0011 1000
  0x20, // 0010 0000
};

enum mod_t : u8 {
  MOD_MEM_NO_DISP,
  MOD_MEM_BYTE_DISP,
  MOD_MEM_WORD_DISP,
  MOD_REG_NO_DISP,

  MOD_INVALID,
};

enum half_register_t    : u8 { AL, CL, DL, BL, AH, CH, DH, BH };
enum wide_register_t    : u8 { AX, CX, DX, BX, SP, BP, SI, DI };
enum segment_register_t : u8 { ES, CS, SS, DS };

/** Parsed representation of an ASM 8086 instruction.
 */
struct instruction_t {
  // Operation that the instruction should perform.
  op_t  opcode;

  /* instruction_t mode.
   * <p> Dictates whether the instruction involves memory or registers, and whether the reg field is displaced.
   * <p> Dictates whether the displacement field is byte or word length.
   */
  mod_t mod;

  // instruction_t register field. This could be either src or dst depending on the D bit.
  u8 reg;

  // Register/memory field. This could be either the src or dst depending on the D bit.
  u8 rm;

  union {
    /* W bit. Specifies whether this is a byte or word operation.
     * <p>- Dictates whether registers are half or full width.
     * <p>- Dictates whether the data field is byte or word length.
     */
    bool wbit;

    /*
     * Z bit.
     * <p> true: repeat/loop while zero flag is clear
     * <p> false: repeat/loop while zero flag is set
     */
    bool zbit;
  };

  union {
    /* D bit. Specifies the "direction" of the operation.
     * <p> true: reg field is the dst.
     * <p> false: reg field is the src.
     */
    bool dbit;

    /*
     * V bit.
     * <p> true: shift/rotate count is specified in CL register.
     * <p> false: shift/rotate count is 1.
     */
    bool vbit;
  };

  union {
    // Displacement data. Used for calculating an address offset. Could be byte or word length depending on the mod field.
    i16 disp;

    // Memory address (offset from 0).
    u16 addr;
  };

  // Data for use in immediate-mode. Could be signed.
  u16 data;

  // Parsing metadata fields
  u8 bytes_read = 0;
  bool is_labeled = false;
  size_t label = 0;
  size_t jump_target = 0;
};

static bool is_conditional_jump_instr(const instruction_t &instr) {
  switch (instr.opcode) {
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
      return true;

    default:
      return false;
  }
}

class instruction_stream {
  using iterator_t = std::vector<instruction_t>::iterator;
  using const_iterator_t = std::vector<instruction_t>::const_iterator;

public:
  iterator_t begin() { return _stream.begin(); }
  iterator_t end() { return _stream.end(); }
  const_iterator_t cbegin() { return _stream.cbegin(); }
  const_iterator_t cend() { return _stream.cend(); }

  void push_back(instruction_t &&instr) {
    if (is_conditional_jump_instr(instr)) _jumps.push_back(_stream.size());
    _stream.push_back(instr);
  }

  /**
   * Iterates through all jump instructions observed during push_back operations and assigns labels to each distinct
   * jump target. Jump instructions are processed in FIFO order for purposes of enumerating labels.
   *
   * <p> This method is idempotent.
   *
   * <p>This method has multiple side-effects and should only be used after an entire file has been first-pass parsed:
   * <p>1. Parsed instructions in the underlying instruction container are assigned labels if they are determined to be jump targets.
   * <p>2. Clears the observed jump instructions. This means subsequent calls are no-ops.
   *
   * @throws std::out_of_range When a jump would land at a misaligned address with respect to the instruction stream.
   */
  void process_jumps() {
    if (_jumps.empty()) return;

    // Track labels.
    size_t next_label = 0;

    // Sentinel instruction. Handles case where last instruction is a jump, avoiding undefined behavior.
    _stream.push_back({
      .bytes_read = _stream.back().bytes_read,
    });
    // Use this lambda to ensure no intermediate state is leaked.
    const auto cleanup = [&](){
      _stream.pop_back();
      _jumps.clear();
    };

    for (size_t jidx : _jumps) {
      // Walk through the instruction stream until the jump target is found. A label will be assigned if one does not
      // exist already.
      instruction_t jump = _stream[jidx];
      const int direction = ((i8)jump.data > 0) ? 1 : -1;
      // Label position in the byte sequence is the start of the following instruction.
      size_t sidx = jidx + 1;
      for(
        size_t label_offset = abs((i8)jump.data);
        label_offset != 0;
        sidx += direction
      ) {
        // This protects from overflow. Jumps should always be aligned with the byte stream. If they aren't, it's likely
        // that the byte sequence was not generated by an ASM 8086 compliant compiler.
        if (_stream[sidx].bytes_read > label_offset) {
          cleanup();
          printf(
            "[offset] %d [sidx] %zu [label_offset] %zu [bytes_read] %d\n",
            (i8)jump.data, sidx, label_offset, _stream[sidx].bytes_read);
          throw std::out_of_range(
            "Jump target offset is not aligned with the byte stream. Binary may not have been compiled correctly.");
        }
        label_offset -= _stream[sidx].bytes_read;
      }

      if (_stream[sidx].is_labeled) {
        // This position has already been labeled.
        _stream[jidx].jump_target = _stream[sidx].label;
        continue;
      }
      // Create a new label.
      _stream[sidx].is_labeled = true;
      _stream[sidx].label = next_label++;
      _stream[jidx].jump_target = _stream[sidx].label;
    }

    cleanup();
  }

private:
  /** holds the instructions */
  std::vector<instruction_t> _stream;

  /** mark the indices of all jump instructions encountered in the stream */
  std::vector<size_t> _jumps;
};

static void dbg_print_instr(const instruction_t &instr) {
  printf("{ opcode:%d, mod:%d, rm:%x, reg:%d, wbit:%d, dbit:%d, disp:%d, data:%d }\n", instr.opcode, instr.mod, instr.rm, instr.reg, instr.wbit, instr.dbit, instr.disp, instr.data);
}

/**************************************************
 * Core instruction parsing logic.
 **************************************************/


/**
 * Parse either a byte or word-length chunk of the byte _stream. It is assumed according to the ASM 8086 spec that the
 * second byte contains the high bits of the data.
 */
static inline u16 parse_data(peeking_iterator<char> &byte_stream, bool word) {
  u16
    lo = (u8)*byte_stream,
    hi = (word)
         ? *(++byte_stream)
         : 0;
  return (hi << 8*sizeof(u8)) | lo;
}

/**
 * Parse the displacement data from the byte _stream. The instruction mode dictates whether the data is byte or
 * word-length.
 *
 * @param mod instruction_t mode.
 * @return Signed displacement value.
 */
static i16 parse_displacement(peeking_iterator<char> &byte_stream, mod_t mod) {
  switch (mod) {
    case MOD_MEM_BYTE_DISP: // fallthru
    case MOD_MEM_WORD_DISP: {
      const u16 disp = parse_data(++byte_stream, mod == MOD_MEM_WORD_DISP);
      if (mod == MOD_MEM_BYTE_DISP) return (i8) disp;
      return (i16) disp;
    }

    default:
      return 0;
  }
}

static inline bool is_direct_addressing_mode(mod_t mod, u8 rm) {
  return (mod == MOD_MEM_NO_DISP) && (rm == 0x06);
}

static inline instruction_t parse_one_byte_no_args(peeking_iterator<char> &byte_stream, u16 opcode) {
  ++byte_stream;
  return {
    .opcode = (op_t) opcode,

    // metadata
    .bytes_read = 1,
  };
}

static inline instruction_t parse_one_byte_with_byte_data(peeking_iterator<char> &byte_stream, u16 opcode) {
  const instruction_t instr {
    .opcode = (op_t) opcode,
    .data = parse_data(++byte_stream, false),

    // metadata
    .bytes_read = 2,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_one_byte_with_word_data(peeking_iterator<char> &byte_stream, u16 opcode) {
  const instruction_t instr {
    .opcode = (op_t) opcode,
    .data = parse_data(++byte_stream, true),

    // metadata
    .bytes_read = 3,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_rep(peeking_iterator<char> &byte_stream, u16 opcode) {
  const u8 first_byte = *byte_stream,
           second_byte = *(++byte_stream);

  instruction_t instr {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(second_byte),

    // Store the string manipulation opcode in the data field
    .data = (u16) (0xfe & second_byte),

    // metadata
    .bytes_read = 2,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_mov_imm_reg(peeking_iterator<char> &byte_stream, u16 _) {
  const u8 first_byte = *byte_stream;
  const bool wbit = asm8086_wbit(first_byte, 3);

  const instruction_t instr {
    .opcode = MOV_IMM_REG,
    .mod    = MOD_REG_NO_DISP,
    .reg    = asm8086_reg(first_byte),
    .wbit   = wbit,
    .dbit   = true,
    .data   = parse_data(++byte_stream, wbit),

    // metadata
    .bytes_read = static_cast<u8>(2 + wbit),
  };

  ++byte_stream;
  return instr;
}

static inline void parse_and_insert_displacement(instruction_t &instr, peeking_iterator<char> &byte_stream) {
  if (is_direct_addressing_mode(instr.mod, instr.rm)) {
    instr.addr = parse_data(++byte_stream, true);
    instr.bytes_read += 1 + instr.wbit;
  } else {
    instr.disp = parse_displacement(byte_stream, instr.mod);
    if (instr.mod == MOD_MEM_BYTE_DISP) ++instr.bytes_read;
    if (instr.mod == MOD_MEM_WORD_DISP) instr.bytes_read += 2;
  }
}

static inline void parse_and_insert_mod_reg_rm(instruction_t &instr, peeking_iterator<char> &byte_stream) {
  const u8 mod_reg_rm_byte = *byte_stream;

  instr.mod = (mod_t) asm8086_mode(mod_reg_rm_byte);
  instr.reg = asm8086_reg(mod_reg_rm_byte, 3);
  instr.rm = asm8086_rm(mod_reg_rm_byte);
}

static inline instruction_t parse_mov_rm(peeking_iterator<char> &byte_stream, u16 _) {
  const u8 first_byte = *byte_stream,
           second_byte = *(++byte_stream);

  instruction_t instr {
    .opcode = MOV_RM,
    .wbit   = asm8086_wbit(first_byte),
    .dbit   = asm8086_dbit(first_byte),

     // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_mov_imm_mem(peeking_iterator<char> &byte_stream, u16 _) {
  const u8 first_byte = *byte_stream,
           second_byte = *(++byte_stream);
  const auto mod = (mod_t) asm8086_mode(second_byte);
  const bool wbit = asm8086_wbit(first_byte);

  const instruction_t instr {
    .opcode = MOV_IMM_MEM,
    .mod    = mod,
    .rm     = asm8086_rm(second_byte),
    .wbit   = wbit,
    .dbit   = false,
    .disp   = parse_displacement(byte_stream, mod),
    .data   = parse_data(++byte_stream, wbit),

    // metadata
    .bytes_read = static_cast<u8>(4 + wbit + (mod == MOD_MEM_WORD_DISP)),
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_mov_acc_to_mem(peeking_iterator<char> &byte_stream, u16 _) {
  const u8 first_byte = *byte_stream;

  const instruction_t instr {
    .opcode = MOV_ACC_TO_MEM,
    .wbit   = asm8086_wbit(first_byte),
    .dbit   = false,
    .addr   = parse_data(++byte_stream, true),

    // metadata
    .bytes_read = 3,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_mov_mem_to_acc(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = MOV_MEM_TO_ACC,
    .wbit   = asm8086_wbit(*byte_stream),
    .dbit   = true,
    .addr   = parse_data(++byte_stream, true),
    .bytes_read = 3,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_push_reg(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = PUSH_REG,
    .reg    = asm8086_reg(*byte_stream),

     // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_push_rm(peeking_iterator<char> &byte_stream, u16 opcode) {
  const u8 second_byte = *(++byte_stream);

  instruction_t instr {
    .opcode = PUSH_RM,

    // metadata
    .bytes_read = 2,
  };


  parse_and_insert_mod_reg_rm(instr, byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_push_pop_seg(peeking_iterator<char> &byte_stream, u16 opcode) {
  const u8 first_byte = *byte_stream;

  instruction_t instr {
    .opcode = (op_t) opcode,
    .mod = MOD_REG_NO_DISP,
    .reg = asm8086_reg_seg(first_byte, 3),

    // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_pop_reg(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = POP_REG,
    .reg    = asm8086_reg(*byte_stream),

    // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_pop_rm(peeking_iterator<char> &byte_stream, u16 _) {
  const u8 second_byte = *(++byte_stream);

  instruction_t instr {
    .opcode = POP_RM,

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_xchg_rm(peeking_iterator<char> &byte_stream, u16 _) {
  instruction_t instr {
    .opcode = XCHG_RM,
    .wbit   = asm8086_wbit(*byte_stream),

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, ++byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_one_byte_with_reg(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = XCHG_REG_ACC,
    .mod    = MOD_REG_NO_DISP,
    .reg    = asm8086_reg(*byte_stream),
    .wbit   = true,

    // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_two_bytes_no_args(peeking_iterator<char> &byte_stream, u16 opcode) {
  ++byte_stream;
  ++byte_stream;
  return {
    .opcode = (op_t) opcode,

    // metadata
    .bytes_read = 2,
  };
}

static inline instruction_t parse_in_fix(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = IN_FIX,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),
    .data   = parse_data(++byte_stream, false),

    // metadata
    .bytes_read = 2,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_in_var(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = IN_VAR,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),

    // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_out_fix(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = OUT_FIX,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),
    .data   = parse_data(++byte_stream, false),

    // metadata
    .bytes_read = 2,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_out_var(peeking_iterator<char> &byte_stream, u16 _) {
  const instruction_t instr {
    .opcode = OUT_VAR,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),

    // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_load(peeking_iterator<char> &byte_stream, u16 opcode) {
  instruction_t instr {
    .opcode = (op_t) opcode,
    .wbit = true,
    .dbit = false,

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, ++byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_add_sub_cmp_rm(peeking_iterator<char> &byte_stream, u16 opcode) {
  instruction_t instr {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(*byte_stream),
    .dbit = asm8086_dbit(*byte_stream),

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, ++byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_shift_rotate(peeking_iterator<char> &byte_stream, u16 opcode) {
  instruction_t instr {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(*byte_stream),
    .vbit = asm8086_vbit(*byte_stream),

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, ++byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_add_sub_cmp_imm_rm(peeking_iterator<char> &byte_stream, u16 opcode) {
  const bool sign_extend = asm8086_sbit(*byte_stream);

  instruction_t instr {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(*byte_stream),
    .dbit = true,

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, ++byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  const bool is_data_word = (!sign_extend && instr.wbit);
  instr.data = parse_data(++byte_stream, is_data_word);
  instr.bytes_read += 1 + is_data_word;

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_add_sub_cmp_imm_acc(peeking_iterator<char> &byte_stream, u16 opcode) {
  const bool wbit = asm8086_wbit(*byte_stream);

  const instruction_t instr {
    .opcode = (op_t) opcode,
    .mod    = MOD_REG_NO_DISP,
    .rm     = AX,
    .wbit   = wbit,
    .dbit   = true,
    .data   = parse_data(++byte_stream, wbit),

    // metadata
    .bytes_read = static_cast<u8>(3 + wbit),
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_inc_dec_neg_rm(peeking_iterator<char> &byte_stream, u16 opcode) {
  instruction_t instr {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(*byte_stream),

    // metadata
    .bytes_read = 2,
  };

  parse_and_insert_mod_reg_rm(instr, ++byte_stream);
  parse_and_insert_displacement(instr, byte_stream);

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_inc_dec_neg_reg(peeking_iterator<char> &byte_stream, u16 opcode) {
  const instruction_t instr {
    .opcode = (op_t) opcode,
    .reg = asm8086_reg(*byte_stream),
    .wbit = true,

    // metadata
    .bytes_read = 1,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_conditional_jump(peeking_iterator<char> &byte_stream, u16 opcode) {
  const instruction_t instr {
    .opcode = (op_t) opcode,
    .data   = parse_data(++byte_stream, false),

    // metadata
    .bytes_read = 2,
  };

  ++byte_stream;
  return instr;
}

// Forward decl
static std::optional<instruction_t> parse_instruction(peeking_iterator<char> &, u16);

static std::optional<instruction_t> parse_requires_second_byte(peeking_iterator<char> &byte_stream, u16 opcode) {
  for (const u8 mask : SECOND_BYTE_MASKS) {
    u16 opcode_candidate = concat(opcode, mask & byte_stream.peek());

    // Handle special case: ADD_IMM_RM code is the same as FB_0x80 code.
    if (opcode_candidate == ADD_IMM_RM) {
      return std::make_optional(parse_add_sub_cmp_imm_rm(byte_stream, opcode_candidate));
    }

    const std::optional<instruction_t> instr = parse_instruction(byte_stream, opcode_candidate);
    if (!instr.has_value()) continue;
    return instr;
  }
  return std::nullopt;
}

static std::optional<instruction_t> match_and_parse(peeking_iterator<char> &byte_stream, u16 op) {
  std::optional<instruction_t> instr = std::nullopt;

  switch (op) {
    // If the first opcode byte matches any of these, it means that a second byte is required to fully assemble the
    // opcode. This triggers a recursive call.
    case FB_0x80:
    case FB_0xff:
    case FB_0xfe:
    case FB_0xf6:
    case FB_0xd0:
    case FB_0xd4:
    case FB_0xd5:
    case FB_0xc6:
    case FB_0x8e:        { instr = parse_requires_second_byte(byte_stream, op); } break;

    case MOV_IMM_REG:    { instr = parse_mov_imm_reg(byte_stream, op); } break;
    case MOV_IMM_MEM:    { instr = parse_mov_imm_mem(byte_stream, op); } break;
    case MOV_RM:         { instr = parse_mov_rm(byte_stream, op); } break;
    case MOV_ACC_TO_MEM: { instr = parse_mov_acc_to_mem(byte_stream, op); } break;
    case MOV_MEM_TO_ACC: { instr = parse_mov_mem_to_acc(byte_stream, op); } break;

    case PUSH_REG:       { instr = parse_push_reg(byte_stream, op); } break;
    case PUSH_RM:        { instr = parse_push_rm(byte_stream, op); } break;
    case PUSH_SEG_ES:
    case PUSH_SEG_CS:
    case PUSH_SEG_SS:
    case PUSH_SEG_DS:    { instr = parse_push_pop_seg(byte_stream, op); } break;

    case POP_REG:        { instr = parse_pop_reg(byte_stream, op); } break;
    case POP_RM:         { instr = parse_pop_rm(byte_stream, op); } break;
    case POP_SEG_ES:
    case POP_SEG_SS:
    case POP_SEG_DS:     { instr = parse_push_pop_seg(byte_stream, op); } break;

    case XCHG_RM:        { instr = parse_xchg_rm(byte_stream, op); } break;

    case XCHG_REG_ACC:   { instr = parse_one_byte_with_reg(byte_stream, op); } break;

    case IN_FIX:         { instr = parse_in_fix(byte_stream, op); } break;
    case IN_VAR:         { instr = parse_in_var(byte_stream, op); } break;
    case OUT_FIX:        { instr = parse_out_fix(byte_stream, op); } break;
    case OUT_VAR:        { instr = parse_out_var(byte_stream, op); } break;

    case LAHF:
    case SAHF:
    case PUSHF:
    case POPF:
    case XLAT:
    case AAA:
    case DAA:
    case AAS:
    case DAS:
    case CBW:
    case CWD:
    case RET_SEG:
    case RET_INTERSEG:
    case INT3:
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
    case LOCK:                   { instr = parse_one_byte_no_args(byte_stream, op); } break;

    case AAM:
    case AAD:                    { instr = parse_two_bytes_no_args(byte_stream, op); } break;

    case RET_SEG_IMM_TO_SP:
    case RET_INTERSEG_IMM_TO_SP: { instr = parse_one_byte_with_word_data(byte_stream, op); } break;

    case INT:                    { instr = parse_one_byte_with_byte_data(byte_stream, op); } break;

    case LEA:
    case LDS:
    case LES:                    { instr = parse_load(byte_stream, op); } break;

    case ADD_RM:
    case ADC_RM:
    case SUB_RM:
    case SBB_RM:
    case CMP_RM:
    case AND_RM:
    case TEST_RM:
    case OR_RM:
    case XOR_RM:                 { instr = parse_add_sub_cmp_rm(byte_stream, op); } break;

    case ADD_IMM_RM:
    case ADC_IMM_RM:
    case SUB_IMM_RM:
    case SBB_IMM_RM:
    case CMP_IMM_RM:
    case AND_IMM_RM:
    case TEST_IMM_RM:
    case OR_IMM_RM:
    case XOR_IMM_RM:             { instr = parse_add_sub_cmp_imm_rm(byte_stream, op); } break;

    case ADD_IMM_ACC:
    case ADC_IMM_ACC:
    case SUB_IMM_ACC:
    case SBB_IMM_ACC:
    case CMP_IMM_ACC:
    case AND_IMM_ACC:
    case TEST_IMM_ACC:
    case OR_IMM_ACC:
    case XOR_IMM_ACC:            { instr = parse_add_sub_cmp_imm_acc(byte_stream, op); } break;

    case INC_RM:
    case DEC_RM:
    case NEG:
    case MUL:
    case IMUL:
    case DIV:
    case IDIV:
    case NOT:
    case CALL_INDIR_SEG:
    case JMP_INDIR_SEG:          { instr = parse_inc_dec_neg_rm(byte_stream, op); } break;

    case INC_REG:
    case DEC_REG:                { instr = parse_inc_dec_neg_reg(byte_stream, op); } break;

    case SHL:
    case SHR:
    case SAR:
    case ROL:
    case ROR:
    case RCL:
    case RCR:                    { instr = parse_shift_rotate(byte_stream, op); } break;

    case REP:                    { instr = parse_rep(byte_stream, op); } break;

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
    case JCXZ:                   { instr = parse_conditional_jump(byte_stream, op); } break;

    default: break;
  }

  return instr;
}

/**
 * Parse a potentially multi-byte instruction sequence from the byte _stream.
 */
static std::optional<instruction_t> parse_instruction(peeking_iterator<char> &byte_stream, u16 opcode) {
  // The opcode masks are handled in descending order to avoid truncating the byte prematurely. Otherwise, we could pick
  // the wrong opcode value.
  for (u16 mask : OPCODE_MASKS) {
    const auto instr = match_and_parse(byte_stream, mask & opcode);
    if (!instr.has_value()) continue;
    return instr;
  }
  return std::nullopt;
}

/**************************************************
 * Parser string conversion utilities.
 **************************************************/
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

static void print_instr(const instruction_t &instr) {
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

/**************************************************
 * Main application flow.
 **************************************************/
struct UserOpts {
  std::string fpath;
};

static UserOpts parse_opts(int argc, char **argv) {
  if (argc != 2) throw std::invalid_argument("Expected 1 argument");
  return {
    .fpath = std::string(argv[1]),
  };
}

int main(int argc, char **argv) {
  // Parse commandline args.
  const UserOpts opts = parse_opts(argc, argv);

  // Read binary into buffer.
  std::ifstream input(opts.fpath, std::ios::binary);
  peeking_iterator<char> byte_stream(std::istreambuf_iterator<char>{input});

  // Disassemble the binary according to ASM 8086 grammar.
  instruction_stream instr_stream;
  while (!byte_stream.is_end()) {
    std::optional<instruction_t> instr = parse_instruction(byte_stream, (u8)*byte_stream);
    if (instr.has_value()) instr_stream.push_back(std::move(instr.value()));
    else throw std::invalid_argument("No matching instruction");
  }

  // Do a second pass to handle labels in jump instructions.
  instr_stream.process_jumps();

  // Write the parsed instruction stream to stdout.
  for (const instruction_t &instr : instr_stream) {
    print_instr(instr);
  }

  return 0;
}
