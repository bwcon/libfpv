#!/usr/bin/env python3
# Makes the hidden symbols of a WebAssembly object file local, like
# `objcopy --localize-hidden` does for ELF (llvm-objcopy cannot for wasm).
#   tools/wasm-localize-hidden.py <in.o> <out.o>
#
# libfpv's browser build links libfpv, Betaflight, Dear ImGui and fmt into
# one relocatable object (CMakeLists.txt). Natively they stay hidden inside
# libfpv's shared library; in a static WebAssembly link they would meet the
# game's own symbols (a C game's init() or delay(), the game's own ImGui).
# Local symbols are invisible outside the object, so only the default
# visibility API (fpv_*) remains. Weak symbols (inline functions, templates)
# stay global so COMDAT deduplication keeps working. Hidden symbols marked
# `used` (Betaflight's parameter group registry) lose Emscripten's export of
# them too; `used` still keeps them in the module.
#
# Format: https://github.com/WebAssembly/tool-conventions/blob/main/Linking.md
# SPDX-License-Identifier: GPL-3.0-or-later
import sys

BINDING_WEAK, BINDING_LOCAL, VISIBILITY_HIDDEN = 0x1, 0x2, 0x4
UNDEFINED, EXPORTED, EXPLICIT_NAME = 0x10, 0x20, 0x40
KIND_DATA, KIND_SECTION = 1, 3
SYMBOL_TABLE = 8


def uleb(buf, pos):
    result = shift = 0
    while True:
        b = buf[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        shift += 7
        if not b & 0x80:
            return result, pos


def enc(value):
    out = bytearray()
    while True:
        b = value & 0x7F
        value >>= 7
        out.append(b | (0x80 if value else 0))
        if not value:
            return bytes(out)


def localize_symbols(payload):
    count, pos = uleb(payload, 0)
    out = bytearray(enc(count))
    changed = 0
    for _ in range(count):
        kind = payload[pos]
        flags, pos = uleb(payload, pos + 1)
        start = pos
        if kind == KIND_SECTION:
            _, pos = uleb(payload, pos)
        elif kind == KIND_DATA:
            n, pos = uleb(payload, pos)
            pos += n
            if not flags & UNDEFINED:
                for _ in range(3):  # segment, offset, size
                    _, pos = uleb(payload, pos)
        else:  # function, global, tag, table
            _, pos = uleb(payload, pos)
            if not flags & UNDEFINED or flags & EXPLICIT_NAME:
                n, pos = uleb(payload, pos)
                pos += n
        if (kind != KIND_SECTION and flags & VISIBILITY_HIDDEN
                and not flags & (UNDEFINED | BINDING_WEAK | BINDING_LOCAL)):
            flags = (flags | BINDING_LOCAL) & ~EXPORTED
            changed += 1
        out.append(kind)
        out += enc(flags)
        out += payload[start:pos]
    return bytes(out), changed


def main(src, dst):
    data = open(src, 'rb').read()
    if data[:4] != b'\0asm':
        sys.exit(f'{src}: not a WebAssembly file')
    out = bytearray(data[:8])
    pos, total = 8, 0
    while pos < len(data):
        sec_id = data[pos]
        size, body = uleb(data, pos + 1)
        end = body + size
        if sec_id == 0:
            n, p = uleb(data, body)
            name = data[p:p + n]
            if name == b'linking':
                p += n
                version, p = uleb(data, p)
                content = bytearray(enc(version))
                while p < end:
                    sub = data[p]
                    sub_len, q = uleb(data, p + 1)
                    payload = data[q:q + sub_len]
                    if sub == SYMBOL_TABLE:
                        payload, total = localize_symbols(payload)
                    content.append(sub)
                    content += enc(len(payload)) + payload
                    p = q + sub_len
                section = enc(n) + name + bytes(content)
                out.append(0)
                out += enc(len(section)) + section
                pos = end
                continue
        out += data[pos:end]
        pos = end
    open(dst, 'wb').write(out)
    print(f'wasm-localize-hidden: {total} hidden symbols made local')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit('usage: wasm-localize-hidden.py <in.o> <out.o>')
    main(sys.argv[1], sys.argv[2])
