#!/usr/bin/env python3
"""Turn src/web/index.html into a C++ raw string literal include.

The page is served from the binary so there is no directory to install and
nothing to get out of step with the JSON it renders -- but it is still a real
.html file on disk, because a 400-line page embedded as a string literal is a
page nobody will edit. This is the one line of build glue that costs.

A <script src="name.js"></script> naming a file beside the page is replaced
by that file's contents, so a drawing with its own few hundred lines can live
in its own file and the binary still serves one document with no second route.

Run by CMake before compiling HttpApi.cpp; also runnable by hand.
"""
import os
import re
import sys

src, dst = sys.argv[1], sys.argv[2]
html = open(src, encoding='utf-8').read()
here = os.path.dirname(os.path.abspath(src))


def inline(m):
    js = open(os.path.join(here, m.group(1)), encoding='utf-8').read()
    # It would end the element early.
    if '</script' in js.lower():
        sys.exit('%s contains </script' % m.group(1))
    return '<script>\n' + js + '</script>'


html = re.sub(r'<script src="([\w.-]+\.js)"></script>', inline, html)

# The delimiter must not appear in the page. )HTML" would end the literal early.
delim = 'HTML'
while ')' + delim + '"' in html:
    delim += 'X'

with open(dst, 'w', encoding='utf-8') as f:
    f.write('// Generated from src/web/index.html by tools/embed-html.py -- do not edit.\n')
    f.write('constexpr const char* kIndexHtml = R"%s(%s)%s";\n' % (delim, html, delim))
print('%s -> %s (%d bytes)' % (src, dst, len(html)))
