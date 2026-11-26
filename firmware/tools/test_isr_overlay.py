from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import isr_overlay as m


def function(name: str, *lines: str, locals_: tuple[str, ...] = ()) -> str:
    allocation = (
        f";{'-' * 60}\n;Allocation info for local variables in function '{name}'\n;{'-' * 60}\n"
        + "".join(f";{line}\n" for line in locals_)
        + f";{'-' * 60}\n"
    )
    return allocation + f";\t-----\n;\t function {name}\n;\t-----\n_{name}:\n" + "".join(f"\t{line}\n" for line in lines)


def vectors(*entries: str) -> str:
    return "__interrupt_vect:\n\tljmp\t__sdcc_gsinit_startup\n" + "".join(f"\t{e}\n\t.ds\t5\n" for e in entries)


IN_REGISTERS = "h                         Allocated to registers r7 "
ON_STACK = "b                         Allocated to stack - _bp -4 +2 "
FIXED = "b                         Allocated with name '_shared_PARM_2'"


class IsrOverlayTest(unittest.TestCase):
    def setUp(self) -> None:
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)

    def asm(self, name: str, *parts: str) -> Path:
        path = Path(self.dir.name) / name
        path.write_text("".join(parts))
        return path

    def test_calls_from_interrupts(self) -> None:
        hal = self.asm(
            "hal.asm",
            function("isr", "lcall\t_edge", "reti"),
            function("main_only", "lcall\t_tick", "ret"),
        )
        sched = self.asm("sched.asm", function("edge", "ljmp\t_lock", "ljmp\t00102$"))
        phase = self.asm("phase.asm", function("lock", "ret"))
        app = self.asm("app.asm", function("tick", "lcall\t_lock", "ret"))
        files = [hal, sched, phase, app]
        self.assertEqual(m.problems(files, [sched, phase]), [])
        self.assertEqual(len(m.problems(files, [sched])), 1)
        self.assertEqual(len(m.problems(files, [sched, phase, app])), 1)

    def test_library_call_from_interrupt(self) -> None:
        hal = self.asm("hal.asm", function("isr", "lcall\t__mulint", "reti"))
        self.assertEqual(m.problems([hal], []), ["an interrupt calls _mulint, which is not in the sources"])

    def program(self, ip: str, shared_locals: tuple[str, ...], main_calls: bool = False) -> list[str]:
        main = self.asm(
            "main.asm",
            vectors("ljmp\t_zo", "ljmp\t_tick"),
            function("main", *(["lcall\t_shared"] if main_calls else []), "ret"),
        )
        hal = self.asm(
            "hal.asm",
            function("zo", "lcall\t_shared", "reti"),
            function("tick", "mov\t_IP,#" + ip, "lcall\t_shared", "reti"),
        )
        shared = self.asm("shared.asm", function("shared", "ret", locals_=shared_locals))
        return m.problems([main, hal, shared], [shared])

    def test_same_priority_may_share_fixed_locals(self) -> None:
        self.assertEqual(self.program("0x00", (IN_REGISTERS, FIXED)), [])

    def test_two_priorities_need_no_fixed_locals(self) -> None:
        self.assertEqual(self.program("0x01", (IN_REGISTERS, ON_STACK)), [])
        self.assertEqual(
            self.program("0x01", (IN_REGISTERS, FIXED)),
            [
                "shared runs from the high-priority interrupts and the low-priority interrupts, but keeps b "
                "at a fixed address: make it REENTRANT"
            ],
        )

    def test_main_loop_and_interrupt_need_no_fixed_locals(self) -> None:
        (problem,) = self.program("0x00", (FIXED,), main_calls=True)
        self.assertIn("shared runs from the low-priority interrupts and the main loop", problem)

    def test_unknown_priorities(self) -> None:
        (problem,) = self.program("0x01\n\torl\t_IP,#0x02", ())
        self.assertIn("cannot tell the interrupt priorities", problem)


if __name__ == "__main__":
    unittest.main()
