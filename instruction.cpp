#include "instruction.h"

void InstructionStream::pushBack(instruction_t &&instr)
{
  if (isConditionalJumpInstr(instr))
  {
    _jumps.push_back(_stream.size());
  }
  _stream.push_back(std::move(instr));
}

void InstructionStream::processJumps()
{
  if (_jumps.empty()) 
  {
    return;
  }

  // Track labels.
  size_t next_label = 0;

  // Sentinel instruction. Handles case where last instruction is a jump, avoiding undefined behavior.
  _stream.push_back({ .bytes_read = _stream.back().bytes_read });
  // Use this lambda to ensure no intermediate state is leaked.
  const auto cleanup = [&_stream = _stream, &_jumps = _jumps]()
  {
    _stream.pop_back();
    _jumps.clear();
  };

  for (size_t jidx : _jumps) 
  {
    // Walk through the instruction stream until the jump target is found. A label will be assigned if one does not
    // exist already.
    instruction_t jump = _stream[jidx];
    const int direction = ((i8)jump.data > 0) ? 1 : -1;
    // Label position in the byte sequence is the start of the following instruction.
    size_t sidx = jidx + 1;
    for(size_t label_offset = abs((i8)jump.data); label_offset != 0; sidx += direction) 
    {
      // This protects from overflow. Jumps should always be aligned with the byte stream. If they aren't, it's likely
      // that the byte sequence was not generated by an ASM 8086 compliant compiler.
      if (_stream[sidx].bytes_read > label_offset) 
      {
        cleanup();
        printf(
          "[offset] %d [sidx] %zu [label_offset] %zu [bytes_read] %d\n",
          (i8)jump.data, sidx, label_offset, _stream[sidx].bytes_read);
        throw std::out_of_range(
          "Jump target offset is not aligned with the byte stream. Binary may not have been compiled correctly.");
      }
      label_offset -= _stream[sidx].bytes_read;
    }

    if (_stream[sidx].is_labeled) 
    {
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

bool InstructionStream::isConditionalJumpInstr(const instruction_t& instr)
{
    switch (instr.opcode) 
    {
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