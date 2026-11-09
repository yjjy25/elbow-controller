"""Compile and exercise both production CAN endpoints and the action controller."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
workspace = root.parents[2]
chassis = workspace / "Mango-Swerve-Firmware/firmware"
compiler = shutil.which("gcc")
if compiler is None:
    raise RuntimeError("MinGW gcc is required")
assert (root / "BSP/work_can_protocol.h").read_bytes() == (
    chassis / "src/service/work_can_protocol.h"
).read_bytes(), "The two protocol headers differ"

can_source = (root / "Core/Src/can.c").read_text()
ioc = (root / "RED_Demo.ioc").read_text()
for expected in ("CAN_MODE_NORMAL", "CAN_BS1_5TQ", "CAN_BS2_2TQ"):
    assert expected in can_source and expected in ioc
assert "CAN2.CalculateBaudRate=1000000" in ioc

with tempfile.TemporaryDirectory(prefix="work_can_", dir=root / "tests") as directory:
    executable = Path(directory) / "test_work_can.exe"
    includes = [
        root / "tests/work_fakes", root / "BSP", chassis / "tests/fakes",
        chassis / "src/service", chassis / "src/platform", chassis / "src/app",
    ]
    sources = [
        root / "tests/test_work_can.c", root / "BSP/vision_motor.c",
        root / "BSP/work_control.c", root / "BSP/can_test.c",
        chassis / "src/service/work_link.c",
    ]
    subprocess.run([
        compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-DTEST_CAN_HAL",
        *["-I" + str(path) for path in includes],
        *[str(path) for path in sources], "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)
