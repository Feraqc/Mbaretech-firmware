/* Read-only local server for the editor and the firmware headers it imports. */
'use strict';

const fs = require('node:fs/promises');
const http = require('node:http');
const path = require('node:path');
const {spawn} = require('node:child_process');

const ROOT = path.resolve(__dirname, '../..');
const SUPPORT = new Set([
  'include/fsm/FSMRecipeTypes.h',
  'include/fsm/FSMDefinitions.h',
  'include/buildConfig.h',
  'include/fsm/fsm_recipe_select.h',
]);
const ASSETS = new Set([
  'fsm_context_editor_v31.html',
  'telemetry_console.html',
  'tools/recipe-editor/core.js',
  'tools/recipe-editor/backend.js',
  'tools/recipe-editor/telemetry.js',
  'tools/recipe-editor/telemetryConsole.js',
  'tools/recipe-editor/telemetryConsole.css',
  'tools/recipe-editor/telemetryStore.js',
  'tools/recipe-editor/telemetryPresentation.js',
  'tools/recipe-editor/telemetryMock.js',
  'tools/recipe-editor/telemetryPanels.js',
  'tools/recipe-editor/telemetryWindowBridge.js',
  'tools/recipe-editor/runtimeTuning.js',
]);

function contentType(name) {
  if (name.endsWith('.html')) return 'text/html';
  if (name.endsWith('.js')) return 'text/javascript';
  if (name.endsWith('.css')) return 'text/css';
  if (name.endsWith('.json')) return 'application/json';
  return 'text/plain';
}

function respond(response, status, body, type = 'text/plain') {
  response.writeHead(status, {
    'Content-Type': `${type}; charset=utf-8`,
    'Content-Length': Buffer.byteLength(body),
    'Cache-Control': 'no-store',
    'X-Content-Type-Options': 'nosniff',
  });
  response.end(body);
}

async function handleRequest(request, response) {
  // The editor only reads from a small allowlist; writes are never accepted.
  if (request.method !== 'GET') {
    respond(response, 501, 'Metodo no admitido');
    return;
  }
  let requested;
  try {
    requested = decodeURIComponent(new URL(request.url, 'http://127.0.0.1').pathname).replace(/^\//, '');
  } catch {
    respond(response, 404, 'No encontrado');
    return;
  }
  if (!requested) requested = 'fsm_context_editor_v31.html';
  if (requested === 'api/recipes') {
    try {
      const folder = path.join(ROOT, 'include/fsm/recipes');
      const entries = await fs.readdir(folder, {withFileTypes: true});
      const names = entries.filter(entry => entry.isFile() && /^fsm_recipe_\w+\.h$/.test(entry.name))
        .map(entry => entry.name).sort();
      respond(response, 200, JSON.stringify(names), 'application/json');
    } catch {
      respond(response, 404, 'No se encontro include/fsm/recipes');
    }
    return;
  }
  const recipe = /^include\/fsm\/recipes\/fsm_recipe_\w+\.h$/.test(requested);
  if (!SUPPORT.has(requested) && !ASSETS.has(requested) && !recipe) {
    respond(response, 404, 'No encontrado');
    return;
  }
  try {
    const target = await fs.realpath(path.join(ROOT, requested));
    const relative = path.relative(ROOT, target);
    if (relative.startsWith('..') || path.isAbsolute(relative)) throw new Error('Fuera del proyecto');
    const body = await fs.readFile(target);
    respond(response, 200, body, contentType(target));
  } catch {
    respond(response, 404, 'No encontrado');
  }
}

function parseArgs(args) {
  let port = 8765;
  let browser = true;
  for (let index = 0; index < args.length; index++) {
    if (args[index] === '--no-browser') {
      browser = false;
    } else if (args[index] === '--port' && /^\d+$/.test(args[index + 1] || '')) {
      port = Number(args[++index]);
      if (port > 65535) throw new Error('Puerto fuera de rango');
    } else {
      throw new Error(`Opcion no reconocida: ${args[index]}`);
    }
  }
  return {port, browser};
}

function openBrowser(url) {
  // Start-Process uses the registered HTTP handler without cmd.exe's fragile
  // nested quoting. The URL is built locally from the listening port.
  const command = process.platform === 'win32' ? 'powershell.exe'
    : process.platform === 'darwin' ? 'open' : 'xdg-open';
  const args = process.platform === 'win32'
    ? ['-NoProfile', '-NonInteractive', '-Command', `Start-Process -FilePath '${url}'`]
    : [url];
  const child = spawn(command, args, {detached: true, stdio: 'ignore', windowsHide: true});
  child.on('error', error => console.error(`No se pudo abrir el navegador: ${error.message}`));
  child.on('exit', code => {
    if (code !== 0) console.error('No se pudo abrir el navegador; usa la URL mostrada arriba.');
  });
  child.unref();
}

function main() {
  let options;
  try {
    options = parseArgs(process.argv.slice(2));
  } catch (error) {
    console.error(error.message);
    process.exitCode = 1;
    return;
  }
  const server = http.createServer((request, response) => {
    handleRequest(request, response).catch(() => respond(response, 500, 'Error interno'));
  });
  server.on('error', error => {
    console.error(`No se pudo iniciar el editor: ${error.message}`);
    process.exitCode = 1;
  });
  server.listen(options.port, '127.0.0.1', () => {
    const url = `http://127.0.0.1:${server.address().port}/fsm_context_editor_v31.html`;
    console.log(`Proyecto: ${ROOT}`);
    console.log(`Editor: ${url}`);
    console.log('Ctrl+C para cerrar. El servidor no escribe archivos.');
    if (options.browser) openBrowser(url);
  });
}

if (require.main === module) main();
module.exports = {handleRequest, parseArgs};
