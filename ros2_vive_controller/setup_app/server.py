"""Web server of the setup app (stdlib only).

    python3 -m ros2_vive_controller.setup_app [--port 8800] [--host 0.0.0.0] [--demo]

`make setup` runs it next to SteamVR (inside the running vive container, or a one-off one).
GET / serves the page, GET /api/state and /api/points the live state, POST /api/<command> (JSON
body) runs a command, e.g. POST /api/paint/start {"side": "right"}.
"""
import argparse
import json
import os
import shutil
import socket
import sys
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ros2_vive_controller.setup_app.core import CommandError, SetupApp

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
DEFAULT_PORT = 8800


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'ViveSetup/1.0'

        def log_message(self, format, *args):     # quiet: the page polls ten times a second
            pass

        def _send(self, code, body, content_type='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path in ('/', '/index.html'):
                with open(os.path.join(STATIC_DIR, 'index.html'), 'rb') as f:
                    self._send(200, f.read(), 'text/html; charset=utf-8')
            elif path == '/api/state':
                self._send(200, app.state())
            elif path == '/api/points':
                self._send(200, app.points_state())
            else:
                self._send(404, {'ok': False, 'message': 'Not found'})

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            if not path.startswith('/api/'):
                self._send(404, {'ok': False, 'message': 'Not found'})
                return
            name = path[len('/api/'):].strip('/').replace('/', '_')
            try:
                length = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(body, dict):
                    raise ValueError('the body must be a JSON object')
            except ValueError as error:
                self._send(400, {'ok': False, 'message': f'Bad request: {error}'})
                return
            try:
                self._send(200, {'ok': True, 'message': app.call(name, **body)})
            except CommandError as error:
                self._send(400, {'ok': False, 'message': str(error)})

    return Handler


def make_server(app, host='0.0.0.0', port=DEFAULT_PORT):
    server = ThreadingHTTPServer((host, port), make_handler(app))
    server.daemon_threads = True
    return server


def urls(host, port):
    """Addresses to print for the user (laptop and tablet)."""
    if host not in ('', '0.0.0.0'):
        return [f'http://{host}:{port}']
    found = [f'http://localhost:{port}']
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))      # no packet is sent; this picks the outgoing address
            found.append(f'http://{s.getsockname()[0]}:{port}')
    except OSError:
        pass
    found.append(f'http://{socket.gethostname()}:{port}')
    return found


def default_config_dir():
    """The mounted source config in the containers, else the checkout's config/ directory."""
    from ros2_vive_controller import launch_config as lc
    checkout = os.path.join(os.path.dirname(os.path.dirname(STATIC_DIR)), '..', 'config')
    for candidate in (lc.SOURCE_CONFIG_DIR, checkout):
        if os.path.isdir(os.path.join(candidate, 'robots')):
            return os.path.normpath(candidate)
    return lc.config_dir()


def _demo_config(source):
    """A scratch copy of the config, so the demo never overwrites this machine's station file."""
    target = tempfile.mkdtemp(prefix='vive_setup_demo_')
    shutil.copytree(os.path.join(source, 'robots'), os.path.join(target, 'robots'))
    shutil.copy(os.path.join(source, 'vive.params.yaml'), target)
    return target


def main(argv=None):
    parser = argparse.ArgumentParser(description='Vive setup app: devices, robot, origin and workspace fences.')
    parser.add_argument('--host', default='0.0.0.0', help='address to listen on (127.0.0.1: this PC only)')
    parser.add_argument('--port', type=int, default=int(os.environ.get('SETUP_PORT') or DEFAULT_PORT))
    parser.add_argument('--config', default=None, help='config directory (default: the package config)')
    parser.add_argument('--demo', action='store_true', help='simulated controllers, no SteamVR needed')
    args = parser.parse_args(argv)

    config = args.config or default_config_dir()
    on_step = None
    if args.demo:
        from ros2_vive_controller.setup_app import demo
        on_step = demo.install().tick
        config = _demo_config(config)
        print(f'Demo mode: simulated devices, saving to {config}', flush=True)

    from ros2_vive_controller.openvr_class.openvr_class import triad_openvr
    print('Connecting to SteamVR...', flush=True)
    vr = triad_openvr()
    app = SetupApp(vr, config, attached_to=os.environ.get('VIVE_SETUP_ATTACHED_TO', ''), on_step=on_step)
    try:
        server = make_server(app, args.host, args.port)
    except OSError as error:
        print(f'Cannot listen on port {args.port}: {error}. Is the setup app already running? '
              f'Set SETUP_PORT in .env to use another port.', file=sys.stderr)
        return 1
    app.start()
    print('Vive setup app ready. Open one of these in a browser (laptop, tablet or phone):', flush=True)
    for url in urls(args.host, args.port):
        print(f'  {url}', flush=True)
    print('Press Ctrl+C to stop.', flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.stop()
        try:
            import openvr
            openvr.shutdown()
        except Exception:
            pass
    return 0
