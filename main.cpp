/**************************************************
 * Main application flow.
 **************************************************/

#include <iostream>
#include <fstream>

#include "parse.h"
#include "stringify.h"

struct UserOpts 
{
  std::string fpath;
};

static UserOpts parseOpts(int argc, char** argv) 
{
  if (argc != 2) 
  {
    throw std::invalid_argument("Expected 1 argument");
  }
  return 
  {
    .fpath = std::string(argv[1]),
  };
}

int main(int argc, char** argv) 
{
  // Parse commandline args.
  const UserOpts opts = parseOpts(argc, argv);

  // Read binary into buffer.
  std::ifstream input{ opts.fpath, std::ios::binary };
  PeekingIterator<char> byte_stream{ std::istreambuf_iterator<char>{input} };

  // Disassemble the binary according to ASM 8086 grammar.
  InstructionStream instr_stream;
  while (!byte_stream.is_end()) 
  {
    std::optional<instruction_t> instr = parseInstruction(byte_stream, (u8)*byte_stream);
    if (instr.has_value()) 
    {
      instr_stream.pushBack(std::move(instr.value()));
    }
    else 
    {
      throw std::invalid_argument("No matching instruction");
    }
  }

  // Do a second pass to handle labels in jump instructions.
  instr_stream.processJumps();

  // Final output.
  for (const instruction_t& instr : instr_stream) 
  {
    printInstruction(instr);
  }
}
