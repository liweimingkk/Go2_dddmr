"""Serve installed web assets with cache disabled for field iteration."""

import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal

from ament_index_python.packages import get_package_share_directory


class NoCacheRequestHandler(SimpleHTTPRequestHandler):
    """Static-file request handler that prevents stale operator interfaces."""

    def end_headers(self) -> None:
        """Add defensive browser headers before completing a response."""
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        super().end_headers()


def parse_args() -> argparse.Namespace:
    """Parse web-server command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--directory", type=Path)
    return parser.parse_args()


def main() -> None:
    """Run the no-cache static web server."""
    args = parse_args()
    directory = args.directory
    if directory is None:
        directory = Path(get_package_share_directory("dddmr_web_viewer")) / "web"
    if not directory.is_dir():
        raise SystemExit(f"Web asset directory does not exist: {directory}")
    if not 1 <= args.port <= 65535:
        raise SystemExit("Port must be in the range 1..65535")

    handler = partial(NoCacheRequestHandler, directory=str(directory))
    server = ThreadingHTTPServer((args.host, args.port), handler)

    def stop_server(_signum, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)
    print(f"DDDMR web viewer: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
