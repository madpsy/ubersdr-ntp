#!/usr/bin/env python3
"""End-to-end smoke test for a built ubersdr-ntp.

What this can and cannot check
------------------------------
It cannot check the DSP: that needs a real receiver, or a fake UberSDR server
speaking Opus, and the decoders here are byte-identical copies of
ubersdr-clock's, which its own build already feeds a synthetic WWV minute and
watches lock. Duplicating that would test the copy, not this program.

What it does check is everything this program adds, and every part of it is a
real failure mode that a clean compile does not rule out:

  * the binary starts and reads a configuration file;
  * the NTP socket binds and answers a genuine mode-3 client request with a
    well-formed mode-4 reply;
  * that reply says stratum 0 / LI=3 while nothing is locked, rather than
    claiming stratum 1 with a garbage offset -- which is the single most
    important thing this server does, because a client that believes an
    unsynchronised radio clock is worse off than one with no radio clock;
  * it does NOT answer mode 6 or mode 7, the control and private modes behind
    every ntpd reflection-amplification advisory;
  * the HTTP service serves the page, /api/time, /api/status and /api/health,
    with /api/health reporting 503 while unsynchronised;
  * the SSE stream connects and emits a tick within a couple of seconds;
  * it refuses to be written to -- a POST is answered 405, not 404;
  * it shuts down on SIGTERM without having to be killed.

Usage: selftest.py /path/to/ubersdr-ntp
"""

import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

NTP_EPOCH_DELTA = 2208988800


def free_port(kind):
    """A port nothing is listening on, chosen by the kernel.

    Fixed port numbers were a mistake worth recording: an earlier version used
    15123/15124, a real daemon happened to be running on exactly those, and the
    test cheerfully talked to IT instead -- reporting stratum 1 where it
    expected 16 and a healthy 503 as a 200. A test that can pass or fail
    depending on what else is running is worse than no test. Binding with port
    0 asks the kernel for one that is free, and the check below refuses to
    proceed if anything answers on it anyway.
    """
    s = socket.socket(socket.AF_INET, kind)
    try:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]
    finally:
        s.close()


NTP_PORT = free_port(socket.SOCK_DGRAM)
HTTP_PORT = free_port(socket.SOCK_STREAM)

ok_count = 0
fail_count = 0


def check(name, condition, detail=''):
    global ok_count, fail_count
    if condition:
        ok_count += 1
        print('  ok    %s' % name)
    else:
        fail_count += 1
        print('  FAIL  %s%s' % (name, (' -- ' + detail) if detail else ''))


def ntp_query(port, mode=3, version=4, timeout=2.0):
    """One NTP request. Returns the parsed reply, or None on timeout."""
    pkt = bytearray(48)
    pkt[0] = (version << 3) | mode
    pkt[2] = 6   # poll
    pkt[3] = 0xEC
    # A recognisable transmit timestamp, so the echoed originate can be checked.
    xmt = time.time() + NTP_EPOCH_DELTA
    struct.pack_into('!II', pkt, 40, int(xmt), int((xmt % 1) * 2**32))

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(bytes(pkt), ('127.0.0.1', port))
        data, _ = s.recvfrom(1024)
    except socket.timeout:
        return None
    finally:
        s.close()

    if len(data) < 48:
        return {'short': len(data)}
    li_vn_mode = data[0]
    return {
        'leap': (li_vn_mode >> 6) & 3,
        'version': (li_vn_mode >> 3) & 7,
        'mode': li_vn_mode & 7,
        'stratum': data[1],
        'poll': data[2],
        'precision': struct.unpack('!b', data[3:4])[0],
        'rootdelay': struct.unpack('!I', data[4:8])[0] / 65536.0,
        'rootdisp': struct.unpack('!I', data[8:12])[0] / 65536.0,
        'refid': data[12:16],
        'originate': data[24:32],
        'sent_xmt': bytes(pkt[40:48]),
        'raw': data,
    }


def http_get(path, timeout=3.0):
    url = 'http://127.0.0.1:%d%s' % (HTTP_PORT, path)
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read().decode('utf-8', 'replace'), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode('utf-8', 'replace'), dict(e.headers)


def http_post(path, timeout=3.0):
    url = 'http://127.0.0.1:%d%s' % (HTTP_PORT, path)
    req = urllib.request.Request(url, data=b'{}', method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status
    except urllib.error.HTTPError as e:
        return e.code
    except Exception:
        return -1


def sse_first_event(timeout=6.0):
    """Connect to /api/events and return the first named event and its data."""
    s = socket.create_connection(('127.0.0.1', HTTP_PORT), timeout=timeout)
    try:
        s.sendall(b'GET /api/events HTTP/1.1\r\nHost: localhost\r\n'
                  b'Accept: text/event-stream\r\n\r\n')
        s.settimeout(timeout)
        buf = b''
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            buf += chunk
            text = buf.decode('utf-8', 'replace')
            # An SSE event ends at a blank line. Look for a complete one that
            # carries both an event name and a data line.
            for block in text.split('\n\n'):
                name, data = None, None
                for line in block.split('\n'):
                    if line.startswith('event: '):
                        name = line[7:].strip()
                    elif line.startswith('data: '):
                        data = line[6:]
                if name and data:
                    return name, data, text
        return None, None, buf.decode('utf-8', 'replace')
    finally:
        s.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    binary = sys.argv[1]
    if not os.access(binary, os.X_OK):
        print('not executable: %s' % binary)
        return 1

    print('ubersdr-ntp selftest: %s' % binary)
    print('  ports: NTP %d, HTTP %d' % (NTP_PORT, HTTP_PORT))

    # Nothing must already be answering there, or every assertion below is
    # about somebody else's process.
    if ntp_query(NTP_PORT, timeout=0.6) is not None:
        print('  ABORT  something is already answering NTP on port %d' % NTP_PORT)
        return 1
    try:
        socket.create_connection(('127.0.0.1', HTTP_PORT), timeout=0.6).close()
        print('  ABORT  something is already listening on TCP port %d' % HTTP_PORT)
        return 1
    except OSError:
        pass

    # --version and --help, before anything is started.
    v = subprocess.run([binary, '--version'], capture_output=True, text=True)
    check('--version', v.returncode == 0 and 'ubersdr-ntp' in v.stdout, v.stdout.strip())
    h = subprocess.run([binary, '--help'], capture_output=True, text=True)
    check('--help', h.returncode == 0 and '--source' in h.stdout)

    # A bad option must be refused rather than ignored.
    b = subprocess.run([binary, '--nonsense'], capture_output=True, text=True)
    check('rejects an unknown option', b.returncode == 2)

    # No arguments at all must say what to do, not start a server with nothing
    # to decode.
    n = subprocess.run([binary], capture_output=True, text=True)
    check('refuses to run with no sources', n.returncode == 2)

    # A configuration that points at a receiver which does not exist: the
    # connection attempts will fail and back off, which is exactly the state
    # this test wants -- running, listening, and honestly unsynchronised.
    cfg = {
        'sources': [{
            'name': 'selftest',
            # 127.0.0.1:1 is guaranteed to refuse, immediately and without DNS.
            'url': 'http://127.0.0.1:1',
            'carrier_hz': 10000000,
        }],
        # The wildcard, so the reply source address can be checked (below).
        'ntp': {'listen': ['0.0.0.0'], 'port': NTP_PORT,
                'answer_when_unsynchronised': True},
        'http': {'enabled': True, 'listen': '127.0.0.1', 'port': HTTP_PORT},
        'log': {'level': 'warn', 'status_interval_seconds': 0},
    }
    fd, path = tempfile.mkstemp(suffix='.json', prefix='ubersdr-ntp-selftest-')
    with os.fdopen(fd, 'w') as f:
        json.dump(cfg, f)

    c = subprocess.run([binary, '--config', path, '--check'],
                       capture_output=True, text=True)
    check('--check accepts the configuration', c.returncode == 0,
          (c.stdout + c.stderr).strip()[-200:])

    proc = subprocess.Popen([binary, '--config', path],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        # Give it a moment to bind. Both sockets are bound before any source
        # connects, so this does not have to wait for the network.
        deadline = time.time() + 10
        bound = False
        while time.time() < deadline:
            if proc.poll() is not None:
                break
            if ntp_query(NTP_PORT, timeout=0.5) is not None:
                bound = True
                break
            time.sleep(0.2)
        if proc.poll() is not None:
            out, errtxt = proc.communicate()
            check('starts and stays running', False,
                  'exited %s: %s' % (proc.returncode, (errtxt or out).strip()[-300:]))
            return 1
        check('NTP socket answers on port %d' % NTP_PORT, bound)

        r = ntp_query(NTP_PORT)
        check('NTP reply is 48 bytes', r is not None and 'short' not in r,
              repr(r))
        if r and 'short' not in r:
            check('NTP reply is mode 4 (server)', r['mode'] == 4, 'mode=%d' % r['mode'])
            check('NTP reply echoes the request version', r['version'] == 4,
                  'version=%d' % r['version'])
            # The point of the whole exercise: with nothing locked it must say
            # so, not claim stratum 1.
            check('unsynchronised reply is stratum 0 on the wire', r['stratum'] == 0,
                  'stratum=%d' % r['stratum'])
            check('unsynchronised reply sets LI=3', r['leap'] == 3, 'leap=%d' % r['leap'])
            check('root delay is zero (we are the reference)', r['rootdelay'] == 0.0,
                  '%f' % r['rootdelay'])
            check('originate timestamp is echoed verbatim',
                  r['originate'] == r['sent_xmt'],
                  '%s != %s' % (r['originate'].hex(), r['sent_xmt'].hex()))
            check('reference id is ASCII', all(ch == 0 or 32 <= ch < 127 for ch in r['refid']),
                  r['refid'].hex())
            check('never-synchronised reply names INIT', r['refid'] == b'INIT', r['refid'].hex())
            check('never-synchronised reference timestamp is zero',
                  r['raw'][16:24] == bytes(8), r['raw'][16:24].hex())
            # Clock resolution, not the radio error budget: that is root
            # dispersion, and a client adds the two together.
            check('precision is the clock resolution', -20 <= r['precision'] <= -10,
                  'precision=%d' % r['precision'])

        # Answered from the address that was asked. The server listens on the
        # wildcard; asking 127.0.0.2 from a socket on 127.0.0.1 is the loopback
        # version of a client reaching a multihomed host on a secondary address,
        # and a connected socket -- as chrony uses -- drops a reply from anywhere
        # else.
        c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        c.settimeout(2.0)
        try:
            c.bind(('127.0.0.1', 0))
            c.connect(('127.0.0.2', NTP_PORT))
            c.send(b'\x23' + bytes(47))
            c.recv(1024)
            check('replies from the address it was asked on', True)
        except socket.timeout:
            check('replies from the address it was asked on', False,
                  'no reply on a socket connected to 127.0.0.2')
        finally:
            c.close()

        # Mode 6 and 7 are the reflection-amplification modes, and mode 1 is a
        # symmetric peer a server-mode reply would be wrong for. Silence is the
        # only correct answer to all three.
        for bad_mode in (1, 6, 7):
            check('ignores mode %d' % bad_mode,
                  ntp_query(NTP_PORT, mode=bad_mode, timeout=1.0) is None)
        # A truncated packet must not be answered either.
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        s.sendto(b'\x1b' * 12, ('127.0.0.1', NTP_PORT))
        try:
            s.recvfrom(1024)
            check('ignores a truncated request', False)
        except socket.timeout:
            check('ignores a truncated request', True)
        finally:
            s.close()

        # --- the HTTP service ------------------------------------------------
        code, body, hdrs = http_get('/')
        check('GET / serves the page', code == 200 and '<title>ubersdr-ntp' in body,
              'HTTP %s' % code)

        code, body, hdrs = http_get('/api/time')
        ok = False
        if code == 200:
            try:
                j = json.loads(body)
                ok = ('unix' in j and 'synchronised' in j and 'roundtrip' in j
                      and 'receive' in j['roundtrip'] and 'transmit' in j['roundtrip'])
            except ValueError:
                pass
        check('GET /api/time returns usable JSON', ok, body[:200])

        code, body, _ = http_get('/api/status')
        ok = False
        if code == 200:
            try:
                j = json.loads(body)
                ok = (len(j.get('sources', [])) == 1
                      and j['sources'][0]['name'] == 'selftest'
                      and j['sources'][0]['dial_hz'] == 9999000)
            except (ValueError, KeyError, IndexError):
                pass
        check('GET /api/status reports the source with a -1 kHz dial', ok, body[:200])

        code, body, _ = http_get('/api/sources')
        check('GET /api/sources returns an array',
              code == 200 and body.lstrip().startswith('['), body[:80])

        code, body, _ = http_get('/api/health')
        check('GET /api/health is 503 while unsynchronised', code == 503, 'HTTP %s' % code)

        code, _, _ = http_get('/api/nothing-here')
        check('GET an unknown path is 404', code == 404, 'HTTP %s' % code)

        check('POST is refused 405 (the service is read-only)',
              http_post('/api/status') == 405)

        name, data, raw = sse_first_event()
        check('SSE stream emits an event', name is not None, raw[:200])
        if data:
            try:
                j = json.loads(data)
                check('SSE event carries a time and source list',
                      'unix' in j or 'sources' in j, data[:160])
            except ValueError:
                check('SSE event data is JSON', False, data[:160])

        # --- shutdown --------------------------------------------------------
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=10)
            check('exits cleanly on SIGTERM', proc.returncode == 0,
                  'returncode=%s' % proc.returncode)
        except subprocess.TimeoutExpired:
            check('exits cleanly on SIGTERM', False, 'still running after 10s')
            proc.kill()
            proc.wait()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        try:
            os.unlink(path)
        except OSError:
            pass

    # --- answer_when_unsynchronised: false ---------------------------------
    #
    # A separate process, because the setting is read at startup. Worth its own
    # run: the first implementation tested the wrong field and answered anyway,
    # and nothing else here would have noticed.
    cfg['ntp']['answer_when_unsynchronised'] = False
    cfg['ntp']['port'] = free_port(socket.SOCK_DGRAM)
    cfg['http']['enabled'] = False
    fd2, path2 = tempfile.mkstemp(suffix='.json', prefix='ubersdr-ntp-selftest-')
    with os.fdopen(fd2, 'w') as f:
        json.dump(cfg, f)
    proc2 = subprocess.Popen([binary, '--config', path2],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        time.sleep(2.0)
        if proc2.poll() is not None:
            check('answer_when_unsynchronised=false starts', False, 'exited early')
        else:
            check('answer_when_unsynchronised=false stays silent',
                  ntp_query(cfg['ntp']['port'], timeout=1.5) is None)
        proc2.send_signal(signal.SIGTERM)
        proc2.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc2.kill()
        proc2.wait()
    finally:
        if proc2.poll() is None:
            proc2.kill()
            proc2.wait()
        try:
            os.unlink(path2)
        except OSError:
            pass

    print('\n%d ok, %d failed' % (ok_count, fail_count))
    return 1 if fail_count else 0


if __name__ == '__main__':
    sys.exit(main())
