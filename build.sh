#!/bin/sh
# Baut SADestruction.asi (32-Bit-Windows-DLL ohne CRT) mit clang + lld (LLVM 18).
set -e
cd "$(dirname "$0")"
llvm-dlltool -m i386 -k -d build/kernel32.def -l build/kernel32.lib
llvm-dlltool -m i386 -k -d build/user32.def   -l build/user32.lib
clang --target=i686-pc-windows-msvc -O2 -Wall -Wextra -Wno-unused-function -ffreestanding -fno-builtin -nostdlib \
      -c src/sa_runtime.c -o build/sa_runtime.obj
lld-link /dll /entry:DllMain@12 /nodefaultlib /subsystem:windows /out:SADestruction.asi \
         build/sa_runtime.obj build/kernel32.lib build/user32.lib
rm -f SADestruction.lib build/*.obj build/*.lib
echo "OK: SADestruction.asi"
