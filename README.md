# Overview
Disassembler for the [8086 instruction set](https://edge.edx.org/c4x/BITSPilani/EEE231/asset/8086_family_Users_Manual_1_.pdf), which is a primordial form of x86. Written in C++17.

# Design
The general approach is to process the bytecode file as a byte stream and parse instructions directly from this stream. Given that we need at most two bytes to fully determine an 8086 instruction type, a simple peeking iterator is used to wrap the byte stream.

Due to the inclusion of jump instructions in the 8086 specification, we cannot immediately emit parsed instructions. Rather, this disassembler uses a two-pass approach -- the first pass to parse basic instructions and the second pass to insert labels for jump targets.

Parsed instructions are written to STDOUT with the expectation that this will be used as a commandline tool.

# Installation
Run the following to generate the `build/disassemble-8086` executable.
```bash
> mkdir -p build && cd build
> cmake -DCMAKE_BUILD_TYPE=Release ..
> make
```

While not strictly required, you may want to install [NASM](https://nasm.us) on your system to generate binaries from the provided `asm/*` files.

# Usage
The `disassemble-8086` executable will output disassembly to STDOUT.

```bash
# optional: generate assembled 8086 binary
> nasm asm/p1.asm -o out.bin

> ./build/disassemble-8086 out.bin
mov cx, bx
```

## Testing
Make sure that you have NASM installed. Then run
```bash
# Single test
> python test.py -f asm/p1.out
OK asm/p1.asm

# Full test suite
> python test.py -d asm
OK asm/p19-segment-offset-notation.asm
OK asm/p15-arithmetic.asm
OK asm/p10-sub.asm
OK asm/p3-mem-mov.asm
...
```

## Static Analysis
This code is:
- Warning free under `-Wall` and `-Wextra`.
- Leak free via macOS `leaks`.
- Clean via AddressSanitizer (`-fsanitize=address` for debug builds).

