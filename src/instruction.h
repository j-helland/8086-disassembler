#pragma once

#include <vector>

#include "types.h"

/** Parsed representation of an ASM 8086 instruction.
 */
struct instruction_t 
{
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

  union 
  {
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

  union 
  {
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

  union 
  {
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

class InstructionStream
{
  using iterator_t = std::vector<instruction_t>::iterator;
  using const_iterator_t = std::vector<instruction_t>::const_iterator;

public:  // methods
  /** Push an instruction onto the instruction stack. This will track any passed jump instructions automatically. */
  void pushBack(instruction_t&& instr); 

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
  void processJumps();

  iterator_t begin() { return _stream.begin(); }
  iterator_t end() { return _stream.end(); }
  const_iterator_t cbegin() { return _stream.cbegin(); }
  const_iterator_t cend() { return _stream.cend(); }  

private:  // methods
  bool isConditionalJumpInstr(const instruction_t& instr);

private:  // data
  /** holds the instructions */
  std::vector<instruction_t> _stream;
  /** mark the indices of all jump instructions encountered in the stream */
  std::vector<size_t> _jumps;
};
