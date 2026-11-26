"""Starts and stops the real robot nodes as separate processes (`ros2 run ...`) and keeps
the last lines of their output for the GUI."""
import collections
import os
import signal
import subprocess
import threading
import time


class ManagedProcess:

    def __init__(self, name, cmd, env=None, on_line=None):
        self.name, self.cmd, self.env = name, cmd, env
        self.on_line = on_line            # callback(line) for every output line
        self.proc = None
        self.lines = collections.deque(maxlen=300)
        self.perf = ''                    # last "perf:" line
        self.started_at = None
        self.lock = threading.Lock()

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    @property
    def exit_code(self):
        return None if self.proc is None else self.proc.poll()

    def start(self):
        if self.running:
            return
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0',
                   RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}', **(self.env or {}))
        self._add(f'$ {" ".join(self.cmd)}')
        self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     env=env, start_new_session=True, text=True, bufsize=1)
        self.started_at = time.time()
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def stop(self, timeout=5.0):
        if not self.running:
            return
        p = self.proc
        try:
            os.killpg(p.pid, signal.SIGINT)           # like Ctrl-C: nodes shut down cleanly
            p.wait(timeout)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(2.0)
        except ProcessLookupError:
            pass
        self._add(f'--- stopped (exit {p.poll()}) ---')

    def _read(self, proc):
        for line in proc.stdout:
            self._add(line.rstrip())
        code = proc.wait()
        if code not in (0, -2, 130, None):
            self._add(f'--- exited with code {code} ---')

    def _add(self, line):
        with self.lock:
            self.lines.append(line)
            if 'perf:' in line:
                self.perf = line[line.index('perf:'):]
        if self.on_line is not None:
            try:
                self.on_line(line)
            except Exception:
                pass

    def tail(self, n=60):
        with self.lock:
            return list(self.lines)[-n:]
