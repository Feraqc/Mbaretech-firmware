/* Exercise the Node launcher backend without opening a browser or writing files. */
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const http = require('node:http');
const path = require('node:path');
const {handleRequest, parseArgs} = require('./serve.cjs');

const root = path.resolve(__dirname, '../..');

async function main() {
  assert.deepEqual(parseArgs(['--no-browser', '--port', '0']), {port: 0, browser: false});
  assert.throws(() => parseArgs(['--port', '65536']), /rango/);
  const server = http.createServer((request, response) => handleRequest(request, response));
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const base = `http://127.0.0.1:${server.address().port}`;
  try {
    const header = await fetch(`${base}/include/fsm/FSMRecipeTypes.h`);
    assert.equal(header.status, 200);
    assert.equal(header.headers.get('cache-control'), 'no-store');
    assert.equal(await header.text(), await fs.readFile(path.join(root, 'include/fsm/FSMRecipeTypes.h'), 'utf8'));
    const recipes = await (await fetch(`${base}/api/recipes`)).json();
    assert(recipes.includes('fsm_recipe_editor_test.h'));
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetry.js`)).status, 200);
    assert.equal((await fetch(`${base}/telemetry_console.html`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetryConsole.js`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetryWindowBridge.js`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetryStore.js`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetryPresentation.js`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/runtimeTuning.js`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetryMock.js`)).status, 200);
    assert.equal((await fetch(`${base}/tools/recipe-editor/telemetryPanels.js`)).status, 200);
    const css = await fetch(`${base}/tools/recipe-editor/telemetryConsole.css`);
    assert.equal(css.status, 200);
    assert.match(css.headers.get('content-type'), /^text\/css/);
    assert.equal((await fetch(`${base}/src/main.cpp`)).status, 404);
    assert.equal((await fetch(`${base}/%2e%2e/platformio.ini`)).status, 404);
    assert.equal((await fetch(`${base}/include/buildConfig.h`, {method: 'POST'})).status, 501);
  } finally {
    await new Promise((resolve, reject) => server.close(error => error ? reject(error) : resolve()));
  }
  console.log('Correcto: servidor Node de solo lectura y rutas fijas.');
}

main().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
