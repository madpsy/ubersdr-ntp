#!/usr/bin/env python3
"""Turn a static file into a C++ raw string literal include.

For the few files the status page loads beside itself -- Leaflet, for the
map -- and which are served from the binary for the same reason the page is:
there is no directory to install, and the page must work on a network with no
route out. See embed-html.py for the page itself.

Usage: embed-file.py SRC DST SYMBOL
"""
import sys

src, dst, symbol = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(src, encoding='utf-8').read()

delim = 'EMB'
while ')' + delim + '"' in text:
    delim += 'X'

with open(dst, 'w', encoding='utf-8') as f:
    f.write('// Generated from %s by tools/embed-file.py -- do not edit.\n' % src.split('/')[-1])
    f.write('constexpr const char* %s = R"%s(%s)%s";\n' % (symbol, delim, text, delim))
print('%s -> %s (%d bytes)' % (src, dst, len(text)))
