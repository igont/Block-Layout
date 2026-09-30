"""Локальный просмотр physical_body_v1 через неизменённые модули СУП."""
import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--sup-root", type=Path, default=Path(__file__).resolve().parents[2] / "SpringTest")
    args = parser.parse_args()
    fb_root = Path(__file__).resolve().parents[1]
    sup_assets = args.sup_root / "model-viewer/src/main/resources/META-INF/resources/js/model-viewer"
    if not (sup_assets / "viewer.js").is_file():
        parser.error("Не найден viewer.js в --sup-root")

    class Handler(SimpleHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        def translate_path(self, path):
            route = unquote(urlsplit(path).path)
            if route.startswith("/sup/"):
                root, relative = sup_assets, route[5:]
            elif route.startswith("/tools/sup-physical-viewer"):
                root, relative = fb_root / "tools", route[7:]
            elif route == "/target/banya-work/walls-physical.json":
                root, relative = fb_root, route[1:]
            else:
                return str(fb_root / "__not_served__")
            resolved = (root / relative).resolve()
            return str(resolved if resolved.is_relative_to(root.resolve()) else fb_root / "__not_served__")

        def end_headers(self):
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            super().end_headers()

    class Server(ThreadingHTTPServer):
        request_queue_size = 128

    with Server(("127.0.0.1", args.port), Handler) as server:
        print(f"http://127.0.0.1:{args.port}/tools/sup-physical-viewer.html", flush=True)
        server.serve_forever()


if __name__ == "__main__":
    main()
