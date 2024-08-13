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

static UserOpts parse_opts(int argc, char** argv) 
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
  const UserOpts opts = parse_opts(argc, argv);

  // Read binary into buffer.
  std::ifstream input(opts.fpath, std::ios::binary);
  peeking_iterator<char> byte_stream(std::istreambuf_iterator<char>{input});

  // Disassemble the binary according to ASM 8086 grammar.
  instruction_stream instr_stream;
  while (!byte_stream.is_end()) 
  {
    std::optional<instruction_t> instr = parse_instruction(byte_stream, (u8)*byte_stream);
    if (instr.has_value()) 
    {
      instr_stream.push_back(std::move(instr.value()));
    }
    else 
    {
      throw std::invalid_argument("No matching instruction");
    }
  }

  // Do a second pass to handle labels in jump instructions.
  instr_stream.process_jumps();

  // Write the parsed instruction stream to stdout.
  for (const instruction_t& instr : instr_stream) 
  {
    print_instruction(instr);
  }

  return 0;
}
