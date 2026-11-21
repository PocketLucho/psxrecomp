#!/bin/bash
# usage: count.sh <binary> <which> <iters>  -> prints executed SH-4 instruction count
qemu-sh4 -one-insn-per-tb -d nochain,exec -D /dev/stdout "$1" "$2" "$3" 2>/dev/null | grep -c "^Trace"
