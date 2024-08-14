/**************************************************
 * Core instruction parsing logic.
 **************************************************/

#pragma once

#include <optional>

#include "types.h"
#include "instruction.h"
#include "peeking_iterator.h"

/**
 * Parse a potentially multi-byte instruction sequence from the byte _stream.
 */
std::optional<instruction_t> parse_instruction(PeekingIterator<char>& byte_stream, u16 opcode);

namespace utils
{
  bool is_direct_addressing_mode(mod_t mod, u8 rm);
}