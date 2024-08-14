#include "parse.h"

#include <optional>

/**************************************************
 * Helper functions to parse ASM 8086 binary fields.
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

// Avoid macros in favor of inline so we can let the compiler "do the right thing".
static inline bool asm8086_wbit(u8 byte, u8 offset)    { return !!(((ASM8086_W_BIT << offset) & byte) >> offset); }
static inline bool asm8086_wbit(u8 byte)               { return asm8086_wbit(byte, 0); }
static inline bool asm8086_dbit(u8 byte)               { return !!(ASM8086_D_BIT & byte); }
static inline u8   asm8086_mode(u8 byte)               { return (ASM8086_MODE_MASK & byte) >> ASM8086_MODE_OFFSET; }
static inline u8   asm8086_reg(u8 byte, u8 offset)     { return ((ASM8086_REG_MASK << offset) & byte) >> offset; }
static inline u8   asm8086_reg(u8 byte)                { return asm8086_reg(byte, 0); }
static inline u8   asm8086_reg_seg(u8 byte, u8 offset) { return 0x03 & asm8086_reg(byte, offset); }
static inline u8   asm8086_rm(u8 byte)                 { return ASM8086_RM_MASK & byte; }

// Kept for posterity / completeness.
[[maybe_unused]] static inline bool asm8086_vbit(u8 byte)               { return !!(ASM8086_V_BIT & byte); }
[[maybe_unused]] static inline bool asm8086_sbit(u8 byte)               { return !!(ASM8086_S_BIT & byte); }
[[maybe_unused]] static inline bool asm8086_zbit(u8 byte)               { return !!(ASM8086_Z_BIT & byte); }
[[maybe_unused]] static inline u8   asm8086_reg_seg(u8 byte)            { return asm8086_reg_seg(byte, 0); }
[[maybe_unused]] static inline u8   asm8086_op_secondary(u8 byte)       { return asm8086_reg(byte, 3); }

/**************************************************
 * Helper functions to decode common byte patterns.
 **************************************************/
bool utils::is_direct_addressing_mode(mod_t mod, u8 rm) 
{
  return (mod == MOD_MEM_NO_DISP) && (rm == 0x06);
}

/**
 * Parse either a byte or word-length chunk of the byte _stream. It is assumed according to the ASM 8086 spec that the
 * second byte contains the high bits of the data.
 */
static inline u16 parse_data(PeekingIterator<char>& byte_stream, bool word) 
{
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
static i16 parse_displacement(PeekingIterator<char>& byte_stream, mod_t mod) 
{
  switch (mod) 
  {
    case MOD_MEM_BYTE_DISP: // fallthru
    case MOD_MEM_WORD_DISP: 
    {
      const u16 disp = parse_data(++byte_stream, mod == MOD_MEM_WORD_DISP);
      if (mod == MOD_MEM_BYTE_DISP) 
      {
        return (i8)disp;
      }
      return (i16)disp;
    }

    default:
      return 0;
  }
}

static inline void parse_and_insert_displacement(instruction_t& instr, PeekingIterator<char>& byte_stream) 
{
  if (utils::is_direct_addressing_mode(instr.mod, instr.rm)) 
  {
    instr.addr = parse_data(++byte_stream, true);
    instr.bytes_read += 1 + instr.wbit;
  } 
  else 
  {
    instr.disp = parse_displacement(byte_stream, instr.mod);
    if (instr.mod == MOD_MEM_BYTE_DISP) 
    {
        ++instr.bytes_read;
    }
    if (instr.mod == MOD_MEM_WORD_DISP) 
    {
        instr.bytes_read += 2;
    }
  }
}

static inline void parse_and_insert_mod_reg_rm(instruction_t& instr, PeekingIterator<char>& byte_stream) 
{
  const u8 mod_reg_rm_byte = *byte_stream;

  instr.mod = (mod_t)asm8086_mode(mod_reg_rm_byte);
  instr.reg = asm8086_reg(mod_reg_rm_byte, 3);
  instr.rm = asm8086_rm(mod_reg_rm_byte);
}

static inline instruction_t parse_one_byte_no_args(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  ++byte_stream;
  return {
    .opcode = (op_t)opcode,

    // metadata
    .bytes_read = 1,
  };
}

static inline instruction_t parse_two_bytes_no_args(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  ++byte_stream;
  ++byte_stream;
  return {
    .opcode = (op_t)opcode,

    // metadata
    .bytes_read = 2,
  };
}

static inline instruction_t parse_one_byte_with_reg(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const u8 reg = asm8086_reg(*byte_stream);
  instruction_t instr = parse_one_byte_no_args(byte_stream, opcode);
  instr.reg = reg;
  return instr;
}

static inline instruction_t parse_one_byte_with_wbit(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const bool wbit = asm8086_wbit(*byte_stream);
  instruction_t instr = parse_one_byte_no_args(byte_stream, opcode);
  instr.wbit = wbit;
  return instr;
}

static inline instruction_t parse_one_byte_with_byte_data(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const instruction_t instr 
  {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(*byte_stream),
    .data = parse_data(++byte_stream, false),

    // metadata
    .bytes_read = 2,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_one_byte_with_word_data(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const instruction_t instr
  {
    .opcode = (op_t) opcode,
    .wbit = asm8086_wbit(*byte_stream),
    .data = parse_data(++byte_stream, true),

    // metadata
    .bytes_read = 3,
  };

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_one_byte_with_address(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_one_byte_with_word_data(byte_stream, opcode);
  std::swap(instr.data, instr.addr);
  return instr;
}

static inline instruction_t parse_wbit_mod_reg_rm_disp(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr 
  {
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

static inline instruction_t parse_dbit_wbit_mod_reg_rm_disp(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const bool dbit = asm8086_dbit(*byte_stream);
  instruction_t instr = parse_wbit_mod_reg_rm_disp(byte_stream, opcode);
  instr.dbit = dbit;
  return instr;
}

static inline instruction_t parse_vbit_wbit_mod_reg_rm_disp(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  return parse_dbit_wbit_mod_reg_rm_disp(byte_stream, opcode);
}

static inline instruction_t parse_sbit_mod_reg_rm_disp_data(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_dbit_wbit_mod_reg_rm_disp(byte_stream, opcode);
  const bool sign_extend = instr.dbit;
  instr.dbit = true;

  const bool is_data_word = (!sign_extend && instr.wbit);
  instr.data = parse_data(byte_stream, is_data_word);
  instr.bytes_read += 1 + is_data_word;

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_one_byte_wbit_data_acc(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_one_byte_with_wbit(byte_stream, opcode);
  instr.data = parse_data(byte_stream, instr.wbit);

  instr.rm = AX;
  instr.dbit = true;
  instr.mod = MOD_REG_NO_DISP;

  ++byte_stream;
  return instr;
}

static inline instruction_t parse_rep(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_one_byte_no_args(byte_stream, opcode);

  // Store the string manipulation opcode in the data field
  instr.data = (u16)(0xfe & *byte_stream);
  instr.wbit = asm8086_wbit(*byte_stream);

  ++instr.bytes_read;
  ++byte_stream;
  return instr;
}

static inline instruction_t parse_mov_imm_reg(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const bool wbit = asm8086_wbit(*byte_stream, 3);
  const u8 reg = asm8086_reg(*byte_stream);

  instruction_t instr = (wbit)
                        ? parse_one_byte_with_word_data(byte_stream, opcode)
                        : parse_one_byte_with_byte_data(byte_stream, opcode);

  instr.reg = reg;
  instr.wbit = wbit;
  instr.dbit = true;

  return instr;
}

static inline instruction_t parse_mov_rm(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const bool dbit = asm8086_dbit(*byte_stream);
  instruction_t instr = parse_wbit_mod_reg_rm_disp(byte_stream, opcode);
  instr.dbit = dbit;
  return instr;
}

static inline instruction_t parse_mov_imm_mem(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_wbit_mod_reg_rm_disp(byte_stream, opcode);
  instr.data = parse_data(byte_stream, instr.wbit);
  instr.bytes_read = 4 + instr.wbit + (instr.mod == MOD_MEM_WORD_DISP);
  ++byte_stream;
  return instr;
}

static inline instruction_t parse_mov_acc_to_mem(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const bool wbit = asm8086_wbit(*byte_stream);
  instruction_t instr = parse_one_byte_with_address(byte_stream, opcode);
  instr.wbit = wbit;
  return instr;
}

static inline instruction_t parse_mov_mem_to_acc(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const bool wbit = asm8086_wbit(*byte_stream);
  instruction_t instr = parse_one_byte_with_address(byte_stream, opcode);
  instr.wbit = wbit;
  instr.dbit = true;
  return instr;
}

static inline instruction_t parse_push_pop_seg(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  const u8 reg = asm8086_reg_seg(*byte_stream, 3);
  instruction_t instr = parse_one_byte_no_args(byte_stream, opcode);
  instr.reg = reg;
  return instr;
}

static inline instruction_t parse_one_byte_reg_wtrue_nodisp(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_one_byte_with_reg(byte_stream, opcode);
  instr.mod = MOD_REG_NO_DISP;
  instr.wbit = true;
  return instr;
}

static inline instruction_t parse_in_out_fix(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_one_byte_with_byte_data(byte_stream, opcode);
  instr.reg = AX;
  return instr;
}

static inline instruction_t parse_in_out_var(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_one_byte_with_wbit(byte_stream, opcode);
  instr.reg = AX;
  return instr;
}

static inline instruction_t parse_load(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  instruction_t instr = parse_wbit_mod_reg_rm_disp(byte_stream, opcode);
  instr.wbit = true;
  return instr;
}

/**************************************************
 * Core instruction parsing flow.
 **************************************************/
static std::optional<instruction_t> parse_requires_second_byte(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  for (const u8 mask : SECOND_BYTE_MASKS) 
  {
    u16 opcode_candidate = concat(opcode, mask & byte_stream.peek());

    // Handle special case: ADD_IMM_RM code is the same as FB_0x80 code.
    if (opcode_candidate == ADD_IMM_RM) 
    {
      return std::make_optional(parse_sbit_mod_reg_rm_disp_data(byte_stream, opcode_candidate));
    }

    const std::optional<instruction_t> instr = parse_instruction(byte_stream, opcode_candidate);
    if (!instr.has_value()) 
    {
      continue;
    }
    return instr;
  }
  return std::nullopt;
}

static std::optional<instruction_t> match_and_parse(PeekingIterator<char>& byte_stream, u16 op) 
{
  std::optional<instruction_t> instr = std::nullopt;

  switch (op) 
  {
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

    case PUSH_REG:       { instr = parse_one_byte_with_reg(byte_stream, op); } break;
    case PUSH_RM:        { instr = parse_wbit_mod_reg_rm_disp(byte_stream, op); } break;
    case PUSH_SEG_ES:
    case PUSH_SEG_CS:
    case PUSH_SEG_SS:
    case PUSH_SEG_DS:    { instr = parse_push_pop_seg(byte_stream, op); } break;

    case POP_REG:        { instr = parse_one_byte_with_reg(byte_stream, op); } break;
    case POP_RM:         { instr = parse_wbit_mod_reg_rm_disp(byte_stream, op); } break;
    case POP_SEG_ES:
    case POP_SEG_SS:
    case POP_SEG_DS:     { instr = parse_push_pop_seg(byte_stream, op); } break;

    case XCHG_RM:        { instr = parse_wbit_mod_reg_rm_disp(byte_stream, op); } break;

    case XCHG_REG_ACC:   { instr = parse_one_byte_reg_wtrue_nodisp(byte_stream, op); } break;

    case IN_FIX:         { instr = parse_in_out_fix(byte_stream, op); } break;
    case IN_VAR:         { instr = parse_in_out_var(byte_stream, op); } break;
    case OUT_FIX:        { instr = parse_in_out_fix(byte_stream, op); } break;
    case OUT_VAR:        { instr = parse_in_out_var(byte_stream, op); } break;

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
    case XOR_RM:                 { instr = parse_dbit_wbit_mod_reg_rm_disp(byte_stream, op); } break;

    case ADD_IMM_RM:
    case ADC_IMM_RM:
    case SUB_IMM_RM:
    case SBB_IMM_RM:
    case CMP_IMM_RM:
    case AND_IMM_RM:
    case TEST_IMM_RM:
    case OR_IMM_RM:
    case XOR_IMM_RM:             { instr = parse_sbit_mod_reg_rm_disp_data(byte_stream, op); } break;

    case ADD_IMM_ACC:
    case ADC_IMM_ACC:
    case SUB_IMM_ACC:
    case SBB_IMM_ACC:
    case CMP_IMM_ACC:
    case AND_IMM_ACC:
    case TEST_IMM_ACC:
    case OR_IMM_ACC:
    case XOR_IMM_ACC:            { instr = parse_one_byte_wbit_data_acc(byte_stream, op); } break;

    case INC_RM:
    case DEC_RM:
    case NEG:
    case MUL:
    case IMUL:
    case DIV:
    case IDIV:
    case NOT:
    case CALL_INDIR_SEG:
    case JMP_INDIR_SEG:          { instr = parse_wbit_mod_reg_rm_disp(byte_stream, op); } break;

    case INC_REG:
    case DEC_REG:                { instr = parse_one_byte_reg_wtrue_nodisp(byte_stream, op); } break;

    case SHL:
    case SHR:
    case SAR:
    case ROL:
    case ROR:
    case RCL:
    case RCR:                    { instr = parse_vbit_wbit_mod_reg_rm_disp(byte_stream, op); } break;

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
    case JCXZ:                   { instr = parse_one_byte_with_byte_data(byte_stream, op); } break;

    default: break;
  }

  return instr;
}

/**
 * Parse a potentially multi-byte instruction sequence from the byte _stream.
 */
std::optional<instruction_t> parse_instruction(PeekingIterator<char>& byte_stream, u16 opcode) 
{
  // The opcode masks are handled in descending order to avoid truncating the byte prematurely. Otherwise, we could pick
  // the wrong opcode value.
  for (u16 mask : OPCODE_MASKS) 
  {
    const std::optional<instruction_t> instr = match_and_parse(byte_stream, mask & opcode);
    if (!instr.has_value()) continue;
    return instr;
  }
  return std::nullopt;
}