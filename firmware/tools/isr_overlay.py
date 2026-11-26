#!/usr/bin/env python3
"""Check how the functions interrupt handlers call keep their locals, in the assembly SDCC wrote.

SDCC lets functions that never run at the same time share RAM for their locals, but it cannot know
that a function an interrupt calls may run in the middle of any other: exactly the files with such
functions must be built with --nooverlay. And a function that runs from more than one of the main
loop, the low-priority and the high-priority interrupts can be entered again while it runs: it
must keep no local or parameter at a fixed address, which REENTRANT guarantees.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys

FUNCTION = re.compile(r"^;\t function (\w+)$", re.MULTILINE)
CALL = re.compile(r"^\t(?:lcall|ljmp|acall|ajmp)\t_(\w+)$", re.MULTILINE)
RETI = re.compile(r"^\treti$", re.MULTILINE)
ALLOCATION = re.compile(r"^;Allocation info for local variables in function '(\w+)'\n;-+\n((?:;[^-].*\n)*)", re.MULTILINE)
IN_REGISTERS_OR_STACK = re.compile(r"Allocated to (?:registers|stack)")
VECTORS = re.compile(r"^__interrupt_vect:\n((?:\t(?:ljmp\t\w+|reti|\.ds\t\d+)\n)*)", re.MULTILINE)
IP_WRITE = re.compile(r"^\t\w+\t_IP\b(.*)$", re.MULTILINE)
IP_VALUE = re.compile(r",#(0x[0-9a-fA-F]+)$")

MAIN = "the main loop"
CONTEXTS = ("the low-priority interrupts", "the high-priority interrupts")


class Function:
    def __init__(self, source: Path, body: str) -> None:
        self.source = source
        self.calls = set(CALL.findall(body))
        self.is_interrupt = bool(RETI.search(body))
        self.fixed: list[str] = []


def read_functions(asm: Path) -> dict[str, Function]:
    text = asm.read_text()
    parts = FUNCTION.split(text)
    functions = {name: Function(asm, body) for name, body in zip(parts[1::2], parts[2::2])}
    for name, block in ALLOCATION.findall(text):
        if name in functions:
            functions[name].fixed = [
                line[1:].split()[0]
                for line in block.splitlines()
                if "Allocated" in line and not IN_REGISTERS_OR_STACK.search(line)
            ]
    return functions


def reachable(functions: dict[str, Function], roots: list[str]) -> set[str]:
    found: set[str] = set()
    todo = list(roots)
    while todo:
        name = todo.pop()
        if name not in found:
            found.add(name)
            todo.extend(functions[name].calls if name in functions else ())
    return found


def interrupt_handlers(texts: list[str]) -> dict[int, str]:
    """The handler of each interrupt number, from the vector table."""
    for text in texts:
        m = VECTORS.search(text)
        if m:
            entries = [line.split() for line in m.group(1).splitlines() if not line.startswith("\t.ds")]
            return {n - 1: entry[1][1:] for n, entry in enumerate(entries) if n and entry[0] == "ljmp"}
    return {}


def priority_bits(texts: list[str]) -> int:
    """The value the program writes to IP, whose bit n raises interrupt n to high priority."""
    writes = [m.group(0) for text in texts for m in IP_WRITE.finditer(text)]
    if not writes:
        return 0
    values = {v.group(1) for w in writes if (v := IP_VALUE.search(w))}
    if len(values) != 1 or len(writes) != len(values):
        raise ValueError(f"cannot tell the interrupt priorities from: {'; '.join(w.strip() for w in writes)}")
    return int(values.pop(), 16)


def contexts(functions: dict[str, Function], texts: list[str]) -> dict[str, set[str]]:
    """For each function, from which of MAIN and CONTEXTS it runs."""
    ip = priority_bits(texts)
    out: dict[str, set[str]] = {}
    roots: list[tuple[str, list[str]]] = [(MAIN, ["main"])]
    for number, handler in interrupt_handlers(texts).items():
        if number >= 8:
            raise ValueError(f"interrupt {number} ({handler}): its priority is not in IP")
        roots.append((CONTEXTS[(ip >> number) & 1], [handler]))
    for context, names in roots:
        for name in reachable(functions, names):
            out.setdefault(name, set()).add(context)
    return out


def problems(asm_files: list[Path], nooverlay: list[Path]) -> list[str]:
    functions: dict[str, Function] = {}
    for asm in asm_files:
        functions.update(read_functions(asm))
    texts = [asm.read_text() for asm in asm_files]
    out = []
    from_interrupts = reachable(
        functions, [callee for f in functions.values() if f.is_interrupt for callee in f.calls]
    )
    needed: set[Path] = set()
    for name in sorted(from_interrupts):
        if name in functions:
            needed.add(functions[name].source)
        else:
            out.append(f"an interrupt calls {name}, which is not in the sources")
    for asm in sorted(needed - set(nooverlay)):
        out.append(f"{asm}: an interrupt calls its functions, but it is built without --nooverlay")
    for asm in sorted(set(nooverlay) - needed):
        out.append(f"{asm}: built with --nooverlay, but no interrupt calls its functions")
    try:
        runs_from = contexts(functions, texts)
    except ValueError as err:
        return [*out, str(err)]
    for name, where in sorted(runs_from.items()):
        if len(where) > 1 and name in functions and functions[name].fixed:
            out.append(
                f"{name} runs from {' and '.join(sorted(where))}, but keeps "
                f"{', '.join(functions[name].fixed)} at a fixed address: make it REENTRANT"
            )
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("asm", type=Path, nargs="+", help="the assembly of every source file")
    parser.add_argument("--nooverlay", type=Path, nargs="*", default=[], help="those built with --nooverlay")
    args = parser.parse_args(argv)
    found = problems(args.asm, args.nooverlay)
    for problem in found:
        print(f"isr_overlay: {problem}", file=sys.stderr)
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
