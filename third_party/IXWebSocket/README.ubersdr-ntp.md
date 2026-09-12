# Vendored IXWebSocket

Trimmed to what this project builds: `ixwebsocket/*.{h,cpp}`, upstream's
`LICENSE.txt` and `README.md`.

Removed deliberately — upstream's `.github/workflows/` most of all, because a
vendored copy of somebody else's CI runs as *this* repository's CI on every
push, against a build it knows nothing about. Also removed: upstream's own
CMakeLists and pkg-config/cmake templates (this project globs the sources and
sets its own compile definitions — see the note in `../../CMakeLists.txt` about
`IXWEBSOCKET_USE_TLS`, which upstream's `option(USE_TLS)` would otherwise have
set and which is not a build error when missing, only a runtime one), the
sibling libraries (`ixcobra`, `ixredis`, `ixsentry`, `ixsnake`, `ixcrypto`,
`ixbots`), the `ws` command-line tool, the Docker files, the mkdocs site, and
the test suite.

Nothing under `ixwebsocket/` has been modified. Updating is a matter of copying
a fresh `ixwebsocket/` directory in.
