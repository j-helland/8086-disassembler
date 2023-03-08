#include <iostream>
#include <fstream>
#include <unordered_map>

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
static constexpr u8 ASM8086_D_BIT       = 0x02;
static constexpr u8 ASM8086_MODE_OFFSET = 6;
static constexpr u8 ASM8086_MODE_MASK   = (0x03 << ASM8086_MODE_OFFSET);
static constexpr u8 ASM8086_REG_MASK    = 0x07;
static constexpr u8 ASM8086_RM_MASK     = 0x07;

// Avoid macros in favor of inline so we can let the compiler "do the right thing".
static inline bool asm8086_wbit(u8 byte, u8 offset) { return !!(((ASM8086_W_BIT << offset) & byte) >> offset); }
static inline bool asm8086_wbit(u8 byte)            { return asm8086_wbit(byte, 0); }
static inline bool asm8086_dbit(u8 byte)            { return !!(ASM8086_D_BIT & byte); }
static inline u8 asm8086_mode(u8 byte)              { return (ASM8086_MODE_MASK & byte) >> ASM8086_MODE_OFFSET; }
static inline u8 asm8086_reg(u8 byte, u8 offset)    { return ((ASM8086_REG_MASK << offset) & byte) >> offset; }
static inline u8 asm8086_reg(u8 byte)               { return asm8086_reg(byte, 0); }
static inline u8 asm8086_rm(u8 byte)                { return ASM8086_RM_MASK & byte; }

/**************************************************
 * ASM 8086 parser data types.
 **************************************************/
/**
 * Opcodes are right-padded to a full byte to avoid an extra bitshift during parsing.
 */
enum op_t : u8 {
  /** mov */
  // Immediate-mode to register mov.
  MOV_IMM_REG    = 0xb0,  // 1011 0000
  // Immediate-mode to register/memory mov.
  MOV_IMM_MEM    = 0xc6,  // 1100 0110
  // Register/memory to register mov.
  MOV_RM         = 0x88,  // 1000 1000
  // Memory to accumulator mov. The AX register is the de facto accumulator register.
  MOV_MEM_TO_ACC = 0xa0,  // 1010 0000
  // Accumulator to memory mov. The AX register is the de facto accumulator register.
  MOV_ACC_TO_MEM = 0xa2,  // 1010 0010

  /** push */
  PUSH_RM        = 0xff,  // 1111 1111
  PUSH_REG       = 0x50,  // 0101 0000

  /** pop */
  POP_RM         = 0x8f,  // 1000 1111
  POP_REG        = 0x58,  // 0101 1000

  /** xchg */
  XCHG_RM        = 0x86, // 1000 0110
  XCHG_REG_ACC   = 0x90, // 1001 0000

  /** in/out */
  IN_FIX         = 0xe4, // 1110 0100
  IN_VAR         = 0xec, // 1110 1100
  OUT_FIX        = 0xe6, // 1110 0110
  OUT_VAR        = 0xee, // 1110 1110

  OP_INVALID,
};

/**
 * The opcode masks are used to select chunks of a byte that may contain various opcode values. These must be checked in
 * descending order to ensure we don't prematurely truncate an opcade chunk and mistake it for the wrong opcode value.
 * For example, this can happen with accumulator mov, where the high 6 bits of mem to acc and acc to mem are the same.
 */
static constexpr u8 OPCODE_MASKS[] {
  0xff,  // 1111 1111
  0xfe,  // 1111 1110
  0xfc,  // 1111 1100
  0xf8,  // 1111 1000
  0xf0,  // 1111 0000
};

enum mod_t : u8 {
  MOD_MEM_NO_DISP,
  MOD_MEM_BYTE_DISP,
  MOD_MEM_WORD_DISP,
  MOD_REG_NO_DISP,

  MOD_INVALID,
};

enum HalfRegister : u8 { AL, CL, DL, BL, AH, CH, DH, BH };
enum WideRegister : u8 { AX, CX, DX, BX, SP, BP, SI, DI };

/** Parsed representation of an ASM 8086 instruction.
 */
struct Instruction {
  // Operation that the instruction should perform.
  op_t  opcode;

  /* Instruction mode.
   * <p> Dictates whether the instruction involves memory or registers, and whether the reg field is displaced.
   * <p> Dictates whether the displacement field is byte or word length.
   */
  mod_t mod;

  // Instruction register field. This could be either src or dst depending on the D bit.
  u8    reg;

  // Register/memory field. This could be either the src or dst depending on the D bit.
  u8    rm;

  /* W bit. Specifies whether this is a byte or word operation.
   * <p>- Dictates whether registers are half or full width.
   * <p>- Dictates whether the data field is byte or word length.
   */
  bool  wbit;

  /* D bit. Specifies the "direction" of the operation.
   * <p> true: reg field is the dst.
   * <p> false: reg field is the src.
   */
  bool  dbit;

  // Displacement data. Used for calculating an address offset. Could be byte or word length depending on the mod field.
  i16   disp;

  union {
    // Data for use in immediate-mode. Could be signed.
    u16 data;

    // Memory address (offset from 0).
    u16 addr;
  };
};

/**************************************************
 * Core instruction parsing logic.
 **************************************************/
/**
 * All instruction parsing functions will adhere to this type.
 */
typedef Instruction (*InstrParseFunc)(std::istreambuf_iterator<char> &byte_stream);

/**
 * Parse either a byte or word-length chunk of the byte stream. It is assumed according to the ASM 8086 spec that the
 * second byte contains the high bits of the data.
 */
static inline u16 parse_data(std::istreambuf_iterator<char> &byte_stream, bool word) {
  u16
    lo = (u8)*byte_stream,
    hi = (word)
         ? *(++byte_stream)
         : 0;
  return (hi << 8*sizeof(u8)) | lo;
}

/**
 * Parse the displacement data from the byte stream. The instruction mode dictates whether the data is byte or
 * word-length.
 *
 * @param mod Instruction mode.
 * @return Signed displacement value.
 */
static i16 parse_displacement(std::istreambuf_iterator<char> &byte_stream, mod_t mod) {
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

static bool is_direct_addressing_mode(mod_t mod, u8 rm) {
  return (mod == MOD_MEM_NO_DISP) && (rm == 0x06);
}

static Instruction parse_mov_imm_reg(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream;
  const bool wbit = asm8086_wbit(first_byte, 3);

  return {
    .opcode = MOV_IMM_REG,
    .mod    = MOD_REG_NO_DISP,
    .reg    = asm8086_reg(first_byte),
    .wbit   = wbit,
    .dbit   = true,
    .data   = parse_data(++byte_stream, wbit),
  };
}

static Instruction parse_mov_rm(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream,
           second_byte = *(++byte_stream);

  Instruction instr {
    .opcode = MOV_RM,
    .mod    = (mod_t) asm8086_mode(second_byte),
    .reg    = asm8086_reg(second_byte, 3),
    .rm     = asm8086_rm(second_byte),
    .wbit   = asm8086_wbit(first_byte),
    .dbit   = asm8086_dbit(first_byte),
  };

  // Handle direct addressing mode.
  if (is_direct_addressing_mode(instr.mod, instr.rm)) {
    instr.addr = parse_data(++byte_stream, instr.wbit);
  } else {
    instr.disp = parse_displacement(byte_stream, instr.mod);
  }

  return instr;
}

static inline Instruction parse_mov_imm_mem(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream,
           second_byte = *(++byte_stream);
  const auto mod = (mod_t) asm8086_mode(second_byte);
  const bool wbit = asm8086_wbit(first_byte);

  return {
    .opcode = MOV_IMM_MEM,
    .mod    = mod,
    .rm     = asm8086_rm(second_byte),
    .wbit   = wbit,
    .dbit   = false,
    .disp   = parse_displacement(byte_stream, mod),
    .data   = parse_data(++byte_stream, wbit),
  };
}

static inline Instruction parse_mov_acc_to_mem(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream;

  return {
    .opcode = MOV_ACC_TO_MEM,
    .wbit   = asm8086_wbit(first_byte),
    .dbit   = false,
    .addr   = parse_data(++byte_stream, true),
  };
}

static inline Instruction parse_mov_mem_to_acc(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream;

  return {
    .opcode = MOV_MEM_TO_ACC,
    .wbit   = asm8086_wbit(first_byte),
    .dbit   = true,
    .addr   = parse_data(++byte_stream, true),
  };
}

static inline Instruction parse_push_reg(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = PUSH_REG,
    .reg    = asm8086_reg(*byte_stream),
  };
}

static inline Instruction parse_push_rm(std::istreambuf_iterator<char> &byte_stream) {
  const u8 second_byte = *(++byte_stream);

  Instruction instr {
    .opcode = PUSH_RM,
    .mod    = (mod_t) asm8086_mode(second_byte),
    .rm     = asm8086_rm(second_byte),
  };

  if (is_direct_addressing_mode(instr.mod, instr.rm)) {
    instr.addr = parse_data(++byte_stream, true);
  } else {
    instr.disp = parse_displacement(byte_stream, instr.mod);
  }

  return instr;
}

static inline Instruction parse_pop_reg(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = POP_REG,
    .reg    = asm8086_reg(*byte_stream),
  };
}

static inline Instruction parse_pop_rm(std::istreambuf_iterator<char> &byte_stream) {
  const u8 second_byte = *(++byte_stream);

  Instruction instr {
    .opcode = POP_RM,
    .mod    = (mod_t) asm8086_mode(second_byte),
    .rm     = asm8086_rm(second_byte),
  };

  if (is_direct_addressing_mode(instr.mod, instr.rm)) {
    instr.addr = parse_data(++byte_stream, true);
  } else {
    instr.disp = parse_displacement(byte_stream, instr.mod);
  }

  return instr;
}

static inline Instruction parse_xchg_rm(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte  = *byte_stream,
           second_byte = *(++byte_stream);

  Instruction instr {
    .opcode = XCHG_RM,
    .mod    = (mod_t) asm8086_mode(second_byte),
    .reg    = asm8086_reg(second_byte, 3),
    .rm     = asm8086_rm(second_byte),
    .wbit   = asm8086_wbit(first_byte),
  };

  if (is_direct_addressing_mode(instr.mod, instr.rm)) {
    instr.addr = parse_data(++byte_stream, true);
  } else {
    instr.disp = parse_displacement(byte_stream, instr.mod);
  }

  return instr;
}

static inline Instruction parse_xchg_reg_acc(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = XCHG_REG_ACC,
    .mod    = MOD_REG_NO_DISP,
    .reg    = asm8086_reg(*byte_stream),
    .wbit   = true,
  };
}

static inline Instruction parse_in_fix(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = IN_FIX,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),
    .data   = parse_data(++byte_stream, false),
  };
}

static inline Instruction parse_in_var(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = IN_VAR,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),
  };
}

static inline Instruction parse_out_fix(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = OUT_FIX,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),
    .data   = parse_data(++byte_stream, false),
  };
}

static inline Instruction parse_out_var(std::istreambuf_iterator<char> &byte_stream) {
  return {
    .opcode = OUT_VAR,
    .mod    = MOD_REG_NO_DISP,
    .reg    = AX,
    .wbit   = asm8086_wbit(*byte_stream),
  };
}

static const std::unordered_map<op_t, InstrParseFunc> PARSER_REGISTRY {
  // mov
  { MOV_IMM_REG,    &parse_mov_imm_reg },
  { MOV_IMM_MEM,    &parse_mov_imm_mem },
  { MOV_RM,         &parse_mov_rm },
  { MOV_ACC_TO_MEM, &parse_mov_acc_to_mem },
  { MOV_MEM_TO_ACC, &parse_mov_mem_to_acc },

  // push
  { PUSH_REG,       &parse_push_reg },
  { PUSH_RM,        &parse_push_rm },

  // pop
  { POP_REG,        &parse_pop_reg },
  { POP_RM,         &parse_pop_rm },

  // xchg
  { XCHG_RM,        &parse_xchg_rm },
  { XCHG_REG_ACC,   &parse_xchg_reg_acc },

  // in / out
  { IN_FIX,         &parse_in_fix },
  { IN_VAR,         &parse_in_var },
  { OUT_FIX,        &parse_out_fix },
  { OUT_VAR,        &parse_out_var },
};

/**
 * Parse a potentially multi-byte instruction sequence from the byte stream.
 */
static Instruction parse_instruction(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream;

  // The opcode masks are handled in descending order to avoid truncating the byte prematurely. Otherwise, we could pick
  // the wrong opcode value.
  for (u8 mask : OPCODE_MASKS) {
    const op_t op = static_cast<op_t>(mask & first_byte);
    if (!PARSER_REGISTRY.contains(op)) continue;

    // Parse.
    const Instruction instr = PARSER_REGISTRY.at(op)(byte_stream);
    // Move the byte stream to the next byte or EOF.
    ++byte_stream;
    return instr;
  }

  printf("%x\n", first_byte);

  throw std::invalid_argument("[parse_instruction] Invalid opcode byte");
}

/**************************************************
 * Parser string conversion utilities.
 **************************************************/
/**
 * All instruction printing functions will adhere to this signature.
 */
typedef void (*PrintInstruction)(const Instruction &);

static constexpr std::string_view
  // opcodes
  MOV_STR  = "mov",
  PUSH_STR = "push",
  POP_STR  = "pop",
  XCHG_STR = "xchg",
  IN_STR   = "in",
  OUT_STR  = "out",

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
  DI_STR = "di";

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
    case PUSH_RM: // fallthru
    case PUSH_REG:
      return PUSH_STR;

    // pop
    case POP_RM: // fallthru
    case POP_REG:
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

    default: { throw std::invalid_argument("[str_opcode] Invalid opcode"); }
  }
}

/**
 * Convert half-word register code to a string representation.
 */
static std::string_view str_half_register(HalfRegister reg) {
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
static std::string_view str_wide_register(WideRegister reg) {
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
 * Convert a register code to a string representation.
 *
 * @param reg Register code.
 * @param wide Whether the register is half or full word-length.
 */
static inline std::string_view str_register(u8 reg, bool wide) {
  return wide
    ? str_wide_register((WideRegister)reg)
    : str_half_register((HalfRegister)reg);
}

/**
 * Handle mapping in Table 4-10 in the 8086 reference manual. Format calculated address and displacement.
 */
static inline std::string str_reg_mem_field_encoding(const Instruction &instr) {
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
    result = "[";
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
  }

  return result;
}

static inline std::string str_byte_or_word(u16 data) {
  return (data <= 0xff)
    ? "byte "
    : "word ";
}

/**
 * Print the ASM 8086 string representation of a parsed mov instruction to stdout.
 */
static void print_mov(const Instruction &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

  std::string
    src = str_reg_mem_field_encoding(instr),
    dst = (instr.opcode == MOV_IMM_MEM)
      ? str_byte_or_word(instr.data) + std::to_string(instr.data)
      : std::string(str_register(instr.reg, instr.wbit));

  // If D bit is not set, then reg field is the src.
  if (!instr.dbit) std::swap(src, dst);

  std::cout
    << dst
    << ", "
    << src
    << std::endl;
}

static void print_push_pop(const Instruction &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

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

    default: throw std::invalid_argument("[print_push] Opcode not supported");
  }
}

static void print_xchg(const Instruction &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

  std::string
    src = str_reg_mem_field_encoding(instr),
    dst = std::string(str_register(instr.reg, instr.wbit));

  std::cout
    << dst
    << ", "
    // We only want to output an explicit size if this is a memory xchg.
    << ((instr.mod == MOD_REG_NO_DISP)
        ? ""
        : (instr.wbit ? "word " : "byte "))
    << src
    << std::endl;
}

static void print_in_out(const Instruction &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

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

static const std::unordered_map<op_t, PrintInstruction> PRINT_INSTRUCTION_REGISTERY {
  // mov
  { MOV_IMM_REG,     &print_mov },
  { MOV_IMM_MEM,     &print_mov },
  { MOV_RM,          &print_mov },
  { MOV_ACC_TO_MEM,  &print_mov },
  { MOV_MEM_TO_ACC,  &print_mov },

  // push
  { PUSH_REG,   &print_push_pop },
  { PUSH_RM,    &print_push_pop },

  // pop
  { POP_REG,    &print_push_pop },
  { POP_RM,     &print_push_pop },

  // xchg
  { XCHG_RM,        &print_xchg },
  { XCHG_REG_ACC,   &print_xchg },

  // in/out
  { IN_FIX,       &print_in_out },
  { IN_VAR,       &print_in_out },
  { OUT_FIX,      &print_in_out },
  { OUT_VAR,      &print_in_out },
};

static void print_instr(const Instruction &instr) {
  if (!PRINT_INSTRUCTION_REGISTERY.contains(instr.opcode)) throw std::invalid_argument("No print function registered for opcode");
  PRINT_INSTRUCTION_REGISTERY.at(instr.opcode)(instr);
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
  std::istreambuf_iterator<char> byte_stream(input);
  const std::istreambuf_iterator<char> end;

  // Disassemble the binary according to ASM 8086 grammar.
  while (byte_stream != end) {
    const Instruction instr = parse_instruction(byte_stream);

    // TODO: should be able to specify an output file.
    print_instr(instr);
  }

  return 0;
}
