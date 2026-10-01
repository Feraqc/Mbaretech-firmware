"""Servidor local de solo lectura para el editor y los headers de Mbaretech2."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, unquote
import argparse
import json
import mimetypes
import re
import threading
import webbrowser

ROOT = Path(__file__).resolve().parents[2]
SUPPORT = {
    'include/fsm/FSMRecipeTypes.h',
    'include/fsm/FSMDefinitions.h',
    'include/buildConfig.h',
    'include/fsm/fsm_recipe_select.h',
}
ASSETS = {'fsm_context_editor_v31.html', 'telemetry_console.html',
          'tools/recipe-editor/core.js', 'tools/recipe-editor/backend.js',
          'tools/recipe-editor/telemetry.js', 'tools/recipe-editor/telemetryConsole.js',
          'tools/recipe-editor/telemetryConsole.css', 'tools/recipe-editor/telemetryStore.js',
          'tools/recipe-editor/telemetryPresentation.js',
          'tools/recipe-editor/telemetryMock.js', 'tools/recipe-editor/telemetryPanels.js',
          'tools/recipe-editor/telemetryWindowBridge.js',
          'tools/recipe-editor/runtimeTuning.js'}


class EditorHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        requested = unquote(urlsplit(self.path).path).lstrip('/')
        if not requested:
            requested = 'fsm_context_editor_v31.html'
        if requested == 'api/recipes':
            folder = ROOT / 'include/fsm/recipes'
            if not folder.is_dir():
                self.send_error(404, 'No se encontro include/fsm/recipes')
                return
            names = sorted(p.name for p in folder.iterdir()
                           if p.is_file() and re.fullmatch(r'fsm_recipe_\w+\.h', p.name))
            self.respond(json.dumps(names).encode('utf-8'), 'application/json')
            return
        recipe = re.fullmatch(r'include/fsm/recipes/fsm_recipe_\w+\.h', requested)
        if requested not in SUPPORT | ASSETS and not recipe:
            self.send_error(404)
            return
        target = (ROOT / requested).resolve()
        if not target.is_relative_to(ROOT) or not target.is_file():
            self.send_error(404)
            return
        content_type = 'text/plain' if target.suffix == '.h' else mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
        try:
            self.respond(target.read_bytes(), content_type)
        except OSError:
            self.send_error(404)

    def respond(self, content, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type + '; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), EditorHandler)
    except OSError as error:
        parser.exit(1, 'No se pudo iniciar el editor: ' + str(error) + '\n')
    url = f'http://127.0.0.1:{server.server_port}/fsm_context_editor_v31.html'
    print('Proyecto: ' + str(ROOT), flush=True)
    print('Editor: ' + url, flush=True)
    print('Ctrl+C para cerrar. El servidor no escribe archivos.', flush=True)
    if not args.no_browser:
        threading.Timer(0.2, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
