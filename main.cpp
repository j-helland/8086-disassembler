#include <iostream>
#include <fstream>

#include "cxxopts.h"  // I'm too lazy to write my own arg parsing logic.

#define u8 uint8_t

/**************************************************
 * Opcode byte parsing.
 **************************************************/
/**
 * Codes are right-padded to a full byte to avoid an extra bitshift during parsing.
 */
enum Op : u8 {
  MOV = 0x88,
};

struct OpCode {
  Op op;
  bool dbit;
  bool wbit;
};

static constexpr u8 ASM8086_OP_MASK   = 0xfc;
static constexpr u8 ASM8086_W_BIT     = 0x01;
static constexpr u8 ASM8086_D_BIT     = 0x02;

// macros are stupid
static inline u8   asm8086_op(u8 byte)   { return ASM8086_OP_MASK & byte; }
static inline bool asm8086_wbit(u8 byte) { return !!(ASM8086_W_BIT & byte); }
static inline bool asm8086_dbit(u8 byte) { return !!(ASM8086_D_BIT & byte); }

static OpCode parse_op(u8 byte) {
  Op op;
  switch (asm8086_op(byte)) {
    case MOV: { op = MOV; } break;
    default: { throw std::invalid_argument("Invalid op code"); }
  }

  return {
    .op = op,
    .dbit = asm8086_dbit(byte),
    .wbit = asm8086_wbit(byte),
  };
}

/**************************************************
 * Register byte parsing
 **************************************************/
enum HalfRegister : u8 { AL, CL, DL, BL, AH, CH, DH, BH };
enum WideRegister : u8 { AX, CX, DX, BX, SP, BP, SI, DI };

struct RegInstr {
  u8 mod;
  u8 src;
  u8 dst;
};

static constexpr u8 ASM8086_MODE_OFFSET = 6;
static constexpr u8 ASM8086_SRC_OFFSET  = 3;
static constexpr u8 ASM8086_MODE_MASK   = 0xc0;
static constexpr u8 ASM8086_DST_MASK    = 0x07;
static constexpr u8 ASM8086_SRC_MASK    = 0x38;

static inline u8 asm8086_mode(u8 byte) { return (ASM8086_MODE_MASK & byte) >> ASM8086_MODE_OFFSET; }
static inline u8 asm8086_src(u8 byte)  { return (ASM8086_SRC_MASK & byte) >> ASM8086_SRC_OFFSET; }
static inline u8 asm8086_dst(u8 byte)  { return (ASM8086_DST_MASK & byte); }

static RegInstr parse_registers(u8 byte) {
  const u8 mod = asm8086_mode(byte);

  // TODO
  if (mod < 0b11) {
    throw std::invalid_argument("Memory mode is not supported");
  }

  return {
    .mod = mod,
    .src = asm8086_src(byte),
    .dst = asm8086_dst(byte),
  };
}

/**************************************************
 * Instruction string conversion.
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
static std::string_view str_opcode(const OpCode &opcode) {
  switch (opcode.op) {
    case MOV: return MOV_STR;
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
 * Write formatted instruction to stdout.
 */
static inline void write_instr(const OpCode &opcode, const RegInstr &args) {
  std::cout
    << str_opcode(opcode)
    << " "
    << str_register(args.dst, opcode.wbit)
    << ", "
    << str_register(args.src, opcode.wbit)
    << std::endl;
}

/**************************************************
 * Main parsing logic.
 **************************************************/
static cxxopts::ParseResult parse_opts(int argc, char **argv) {
  cxxopts::Options options("disassembler", "disassemble 8086 binary");
  options.add_options()
    ("f,file", "File path", cxxopts::value<std::string>())
    ("v,verbose", "Verbose mode");
  return options.parse(argc, argv);
}

static std::vector<u8> read_binary(const std::string &path) {
  std::ifstream input(path, std::ios::binary);
  std::vector<u8> buffer(std::istreambuf_iterator<char>(input), {});

  if (buffer.size() % 2 > 0) {
    throw std::length_error("Uneven number of bytes in binary file");
  }
  return buffer;
}

int main(int argc, char **argv) {
  // Parse commandline args.
  const auto opts = parse_opts(argc, argv);

  // Read binary into buffer.
  const std::string fpath = opts["file"].as<std::string>();
  const auto buffer = read_binary(fpath);

  // Disassemble.
  for (auto it = buffer.cbegin(); it != buffer.cend();) {
    const OpCode opcode = parse_op(*it++);
    const RegInstr args = parse_registers(*it++);
    write_instr(opcode, args);
  }

  return 0;
}
