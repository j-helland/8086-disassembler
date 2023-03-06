#include <iostream>
#include <fstream>

#include "cxxopts.h"  // I'm too lazy to write my own arg parsing logic.

#define u8 uint8_t
#define u16 uint16_t

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
  // Immediate-mode to register mov.
  MOV_IMM_REG    = 0xb0,  // 1011 0000

  // Immediate-mode to register/memory mov.
  MOV_IMM_MEM    = 0xc6,  // 1100 0110

  // Register/memory to register mov.
  MOV_RM         = 0x88,  // 1000 1000

  // TODO
  MOV_MEM_TO_ACC = 0xa0,  // 1010 0000
  MOV_ACC_TO_MEM = 0xa2,  // 1010 0010
  MOV_RM_TO_SEG  = 0x8e,  // 1000 1110
  MOV_SEG_TO_RM  = 0x8c,  // 1000 1100

  OP_INVALID,
};

enum OpcodeMasks : u8 {
  // Portion of the byte corresponding to an immediate-mode to register mov.
  MASK_MOV_IMM_REG = 0xf0,

  // Portion of the byte corresponding to a register/memory to register mov.
  MASK_MOV_RM = 0xfc,

  // For when the other masks don't apply.
  MASK_MOV_DEFAULT = 0xfe,
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

  // Register / memory field. This could be either the src or dst depending on the D bit.
  u8    rm;

  /* W bit. Used to specify byte or word-length for various properties.
   * <p>- Dictates whether registers are half or full width.
   * <p>- Dictates whether the data field is byte or word length.
   */
  bool  wbit;

  /* D bit.
   * <p> true: reg field is the dst.
   * <p> false: reg field is the src.
   */
  bool  dbit;

  // Displacement data. Used for calculating an address offset. Could be byte or word length depending on the mod field.
  u16   disp;

  // Data for use in immediate-mode. Could be signed.
  u16   data;
};

/**************************************************
 * Core instruction parsing logic.
 **************************************************/
/**
 * Parse either a byte or word-length chunk of the byte stream. It is assumed according to the ASM 8086 spec that the
 * second byte contains the high bits of the data.
 */
static inline u16 parse_data(std::istreambuf_iterator<char> &byte_stream, bool word) {
  u16
    lo = (u8)*byte_stream,
    hi = word
         ? *(++byte_stream)
         : 0;
  return (hi << 8*sizeof(u8)) | lo;
}

/**
 * Parse the displacement data from the byte stream. The instruction mode dictates whether the data is byte or
 * word-length.
 *
 * @param mod Instruction mode.
 */
static u16 parse_displacement(std::istreambuf_iterator<char> &byte_stream, mod_t mod) {
  switch (mod) {
    case MOD_MEM_BYTE_DISP: // fallthru
    case MOD_MEM_WORD_DISP:
      return parse_data(++byte_stream, mod == MOD_MEM_WORD_DISP);

    default:
      return 0;
  }
}

/**
 * Parse a potentially multi-byte instruction sequence from the byte stream.
 */
static Instruction parse_instruction(std::istreambuf_iterator<char> &byte_stream) {
  const u8 first_byte = *byte_stream;

  Instruction instr {
    .opcode = OP_INVALID,
    .mod = MOD_INVALID,
    .reg = 0,
    .rm = 0,
    .wbit = false,
    .dbit = true,  // default true so immediate mode has correct dst,src.
    .disp = 0,
    .data = 0,
  };

  switch (MASK_MOV_IMM_REG & first_byte) {
    case MOV_IMM_REG: {
      instr.opcode = MOV_IMM_REG;
      instr.wbit = asm8086_wbit(first_byte, 3);
      instr.reg = asm8086_reg(first_byte);
      instr.mod = MOD_REG_NO_DISP;
      instr.data = parse_data(++byte_stream, instr.wbit);

      byte_stream++;
      break;
    }

    default: {
      switch (MASK_MOV_RM & first_byte) {
        case MOV_RM: {
          instr.opcode = MOV_RM;
          instr.wbit = asm8086_wbit(first_byte);
          instr.dbit = asm8086_dbit(first_byte);

          const u8 second_byte = *(++byte_stream);
          instr.mod = (mod_t) asm8086_mode(second_byte);
          instr.reg = asm8086_reg(second_byte, 3);
          instr.rm = asm8086_rm(second_byte);
          instr.disp = parse_displacement(byte_stream, instr.mod);

          byte_stream++;
          break;
        }

        default: {
          switch (MASK_MOV_DEFAULT & first_byte) {
            case MOV_IMM_MEM: {
              instr.opcode = MOV_IMM_MEM;
              instr.wbit = asm8086_wbit(first_byte);

              const u8 second_byte = *(++byte_stream);
              instr.mod = (mod_t) asm8086_mode(second_byte);
              instr.rm = asm8086_rm(second_byte);
              instr.disp = parse_displacement(byte_stream, instr.mod);
              instr.data = parse_data(++byte_stream, instr.wbit);

              byte_stream++;
              break;
            }

            default: {
              printf("opcode %x\n", first_byte);
              throw std::invalid_argument("[parse_opcode] Invalid opcode byte");
            }
          }
        }
      }
    }
  }

  return instr;
}

/**************************************************
 * Parser string conversion utilities.
 **************************************************/
static constexpr std::string_view
  // opcodes
  MOV_STR = "mov",

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
    case MOV_IMM_REG:  // fallthru
    case MOV_IMM_MEM:  // fallthru
    case MOV_RM:
      return MOV_STR;
    default: { throw std::invalid_argument("Invalid opcode"); }
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
        if (instr.mod > 0) result += "bp";
        // TODO: unclear how to handle this
        else result += "DIRECT_ADDRESS";
        break;
      case 7:
        result += "bx";
        break;

      default:
        throw std::invalid_argument("Invalid instruction r/m field");
    }
    if (instr.disp != 0) {
      result += " + " + std::to_string(instr.disp);
    }
    result += "]";
  }

  return result;
}

/**
 * Print the ASM 8086 string representation of a parsed instruction to stdout.
 */
static void print_instr(const Instruction &instr) {
  std::cout
    << str_opcode(instr.opcode)
    << " ";

  std::string
    src = str_reg_mem_field_encoding(instr),
    dst = std::string(str_register(instr.reg, instr.wbit));

  // If D bit is not set, then reg field is the src.
  if (!instr.dbit) std::swap(src, dst);

  std::cout
    << dst
    << ", "
    << src
    << std::endl;
}

/**************************************************
 * Main application flow.
 **************************************************/
static cxxopts::ParseResult parse_opts(int argc, char **argv) {
  cxxopts::Options options("disassembler", "disassemble 8086 binary");
  options.add_options()
    ("f,file", "File path", cxxopts::value<std::string>());
  return options.parse(argc, argv);
}

int main(int argc, char **argv) {
  // Parse commandline args.
  const auto opts = parse_opts(argc, argv);

  // Read binary into buffer.
  const std::string fpath = opts["file"].as<std::string>();
  std::ifstream input(fpath, std::ios::binary);
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
