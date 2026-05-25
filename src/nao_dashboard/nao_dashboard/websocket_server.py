"""HTTP server for dashboard API + static web UI.

Current implementation uses HTTP polling. The module name is kept for planned
websocket upgrades without changing the backend imports.
"""

from __future__ import annotations

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.parse import parse_qs
from urllib.parse import urlparse


class DashboardHttpServer:
    """Serve dashboard API and static files from a background thread."""

    def __init__(self, *, host: str, port: int, web_dir: str, state_provider):
        self._host = str(host).strip() or '127.0.0.1'
        self._port = int(port)
        self._web_dir = Path(web_dir)
        self._state_provider = state_provider
        self._server = None
        self._thread = None

    def start(self) -> None:
        if self._server is not None:
            return

        handler = self._build_handler(self._web_dir, self._state_provider)
        self._server = ThreadingHTTPServer((self._host, self._port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        self._server = None

    @property
    def base_url(self) -> str:
        return 'http://%s:%d' % (self._host, self._port)

    @staticmethod
    def _build_handler(web_dir: Path, state_provider):
        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path or '/'
                query = parse_qs(parsed.query)

                if path == '/api/state':
                    return self._json_response(state_provider())
                if path == '/api/events':
                    limit = _coerce_int(query.get('limit', ['200'])[0], default=200)
                    state = state_provider()
                    events = state.get('events', [])
                    return self._json_response({'events': events[-max(1, min(2000, limit)): ]})
                if path == '/api/ros_graph':
                    state = state_provider()
                    return self._json_response(state.get('ros_graph', {}))
                if path == '/api/ab_registry':
                    state = state_provider()
                    return self._json_response(state.get('ab_registry', {}))
                if path in ('/', '/index.html'):
                    return self._static_response(web_dir / 'index.html', 'text/html; charset=utf-8')
                if path == '/app.js':
                    return self._static_response(web_dir / 'app.js', 'application/javascript; charset=utf-8')
                if path == '/styles.css':
                    return self._static_response(web_dir / 'styles.css', 'text/css; charset=utf-8')

                self.send_error(HTTPStatus.NOT_FOUND, 'Not found')

            def log_message(self, fmt, *args):
                # Keep ROS logs clean; backend node logs readiness explicitly.
                _ = (fmt, args)

            def _json_response(self, payload: dict):
                body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _static_response(self, path: Path, content_type: str):
                if not path.exists() or not path.is_file():
                    self.send_error(HTTPStatus.NOT_FOUND, 'Not found')
                    return
                body = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return _Handler


def _coerce_int(value, *, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)
