"""Minimal HTTP server for the simulator GUI (standard library only).

  GET  /                    the GUI (static/index.html)
  GET  /api/state           snapshot of the whole simulation as JSON (polled ~10x per second)
  GET  /api/logs?key=NAME   last output lines of a node process
  GET  /api/image/NAME.jpg  latest debug image (gesture, bone)
  POST /api/action          JSON {"type": ..., ...}, see Simulator.action
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


def static_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('robot_sim'), 'static')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'static')


class WebServer:

    def __init__(self, sim, port):
        self.sim, self.port = sim, port
        self.static = static_dir()
        sim_ref, static = sim, self.static

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):          # keep the console quiet
                pass

            def send(self, code, body, ctype):
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def json(self, obj, code=200):
                self.send(code, json.dumps(obj).encode(), 'application/json')

            def do_GET(self):
                url = urlparse(self.path)
                try:
                    if url.path in ('/', '/index.html'):
                        with open(os.path.join(static, 'index.html'), 'rb') as f:
                            self.send(200, f.read(), 'text/html; charset=utf-8')
                    elif url.path == '/api/state':
                        self.json(sim_ref.snapshot())
                    elif url.path == '/api/logs':
                        key = parse_qs(url.query).get('key', [''])[0]
                        self.json({'key': key, 'lines': sim_ref.logs(key)})
                    elif url.path.startswith('/api/image/'):
                        name = os.path.splitext(os.path.basename(url.path))[0]
                        jpg = sim_ref.image(name)
                        if jpg is None:
                            self.send(404, b'no image', 'text/plain')
                        else:
                            self.send(200, jpg, 'image/jpeg')
                    else:
                        self.send(404, b'not found', 'text/plain')
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                if urlparse(self.path).path != '/api/action':
                    return self.send(404, b'not found', 'text/plain')
                try:
                    n = int(self.headers.get('Content-Length', 0))
                    body = json.loads(self.rfile.read(n) or b'{}')
                    self.json(sim_ref.action(body) or {'ok': True})
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except Exception as e:             # bad request -> show it in the GUI
                    self.json({'error': f'{type(e).__name__}: {e}'}, 400)

        self.httpd = ThreadingHTTPServer(('0.0.0.0', port), Handler)
        self.httpd.daemon_threads = True

    def start(self):
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def stop(self):
        self.httpd.shutdown()
