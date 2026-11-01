"""
PlatformIO pre: script: compile C sources as gnu17.

The ESP32-S31 framework (GCC 15) passes -std=gnu23 for C. C23 makes bool/true/false
keywords, which breaks C libraries that define them (ricmoo/QRCode's qrcode.h).
The C3/S3 toolchains compile C as gnu17, so this keeps C behavior the same across
targets. Pair it with `build_unflags = -std=gnu23` to drop the framework flag.
C++ flags are not touched.
"""

Import("env")  # noqa: F821  (provided by SCons)

env.Append(CFLAGS=["-std=gnu17"])  # noqa: F821
