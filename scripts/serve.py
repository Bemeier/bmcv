#!/usr/bin/env python3
"""Serve web/ for development, without caching anything.

`python3 -m http.server` sends Last-Modified and no Cache-Control, which lets a
browser cache an ES module heuristically - it may reuse one for a tenth of the
time since it was last modified without asking. That is invisible and it lies:
you edit a module, reload, and the page runs the old one. An afternoon can go
into debugging a fix that was never loaded.

Nothing here is served to anyone but the developer who started it, so there is
no cost to refusing to cache at all.

    python3 scripts/serve.py [PORT] [DIRECTORY]
"""

import sys
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler


class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    # One line per request instead of three, and only for what was asked for.
    def log_message(self, fmt, *args):
        sys.stderr.write(f"{self.address_string()} {fmt % args}\n")


def main() -> int:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    directory = sys.argv[2] if len(sys.argv) > 2 else "web"

    handler = partial(NoCacheHandler, directory=directory)
    with HTTPServer(("", port), handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
