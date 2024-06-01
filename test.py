import os
import tempfile
import argparse
import subprocess


def ls(d: str) -> str:
    for p in map(lambda f: os.path.join(d, f), os.listdir(d)):
        yield p


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", type=str)
    parser.add_argument("-d", "--directory", type=str)
    parser.add_argument("-e", "--executable", type=str, default="./build/disassemble-8086")
    args = parser.parse_args()

    if not args.file:
        assert os.path.isdir(args.directory)
        files = ls(args.directory)
    elif not args.directory:
        assert os.path.isfile(args.file)
        files = [args.file]
    else:
        raise RuntimeError

    for file_path in files:
        with (
            tempfile.NamedTemporaryFile() as result_file,
            tempfile.NamedTemporaryFile() as disassembly_file,
            tempfile.NamedTemporaryFile() as expected_file,
        ):
            # Run assembly and disassembly.
            subprocess.call(["nasm", file_path, "-o", result_file.name])
            subprocess.call(
                [args.executable, result_file.name],
                stdout=disassembly_file.file)

            # Assemble the disassembly file.
            subprocess.call(["nasm", disassembly_file.name, "-o", expected_file.name])

            result = result_file.readlines()
            expected = expected_file.readlines()
            if result != expected:
                print(f"FAILED {file_path}")
                print(f"Mismatched binaries.\n{'Result':<12}{repr(result)}\n{'Expected':<12}{repr(expected)}")
                disassembly_file.file.seek(0)
                print()
                for instr in disassembly_file.file.readlines():
                    print(instr.decode("utf-8").strip())
                print()
                raise ValueError
            print(f"OK {file_path}")
