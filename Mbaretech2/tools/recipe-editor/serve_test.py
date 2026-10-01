"""Comprobar rutas locales y límites del servidor sin abrir el navegador."""
import threading
import unittest
from urllib.request import urlopen, Request
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer
import json
from serve import EditorHandler, ROOT


class ServerTest(unittest.TestCase):
    def test_project_routes(self):
        server = ThreadingHTTPServer(('127.0.0.1', 0), EditorHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}/'
        try:
            with urlopen(base + 'include/fsm/FSMRecipeTypes.h') as response:
                self.assertEqual(response.read(), (ROOT / 'include/fsm/FSMRecipeTypes.h').read_bytes())
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
            with urlopen(base + 'api/recipes') as response:
                self.assertIn('fsm_recipe_editor_test.h', json.load(response))
            with urlopen(base + 'tools/recipe-editor/telemetry.js') as response:
                self.assertIn(b'WebSocketTelemetryTransport', response.read())
            with urlopen(base + 'telemetry_console.html') as response:
                self.assertIn(b'Mbaretech Telemetry Console', response.read())
            with urlopen(base + 'tools/recipe-editor/telemetryWindowBridge.js') as response:
                self.assertIn(b'BroadcastChannel', response.read())
            with urlopen(base + 'tools/recipe-editor/telemetryConsole.css') as response:
                self.assertTrue(response.headers['Content-Type'].startswith('text/css'))
            with urlopen(base + 'tools/recipe-editor/telemetryMock.js') as response:
                self.assertIn(b'MockTelemetryTransport', response.read())
            with urlopen(base + 'tools/recipe-editor/telemetryPresentation.js') as response:
                self.assertIn(b'formatEventTimeMs', response.read())
            with urlopen(base + 'tools/recipe-editor/runtimeTuning.js') as response:
                self.assertIn(b'RuntimeTuning', response.read())
            for route in ['../platformio.ini', '%2e%2e/platformio.ini', 'src/main.cpp']:
                with self.assertRaises(HTTPError) as error:
                    urlopen(base + route)
                self.assertEqual(error.exception.code, 404)
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(base + 'include/buildConfig.h', data=b'no', method='POST'))
            self.assertEqual(error.exception.code, 501)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
