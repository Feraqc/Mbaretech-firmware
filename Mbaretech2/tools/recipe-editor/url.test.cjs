/* Las rutas son fijas; los contenidos vienen siempre del servidor del proyecto. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {Project, paths} = require('./backend.js');
const root = path.resolve(__dirname, '../..');
const names = fs.readdirSync(path.join(root,'include/fsm/recipes')).filter(name=>/^fsm_recipe_\w+\.h$/.test(name));
const requested = [];
const fetchFile = async (url, options) => {
  const relative = new URL(url).pathname.slice(1);
  requested.push(relative);
  assert.equal(options.cache,'no-store');
  return {ok:true, async text() {
    return relative === 'api/recipes' ? JSON.stringify(names) : fs.readFileSync(path.join(root,relative),'utf8');
  }};
};
(async()=>{
  const project = await new Project().openFromUrl('http://127.0.0.1:8765/fsm_context_editor_v31.html',fetchFile);
  assert(Object.values(paths).every(file=>requested.includes(file)));
  assert(names.every(name=>requested.includes('include/fsm/recipes/'+name)));
  assert(project.api && project.recipes.length);
  assert(project.recipes.some(recipe=>recipe.file==='fsm_recipe_test.h'));
  assert(!project.errors.some(error=>error.includes('fsm_recipe_test.h')));
  assert(project.unavailable.some(item=>item.includes('fsm_recipe_combat.h')));
  assert(!project.errors.some(error=>error.includes('fsm_recipe_combat.h')));
  assert.equal(project.directory,null);
  await assert.rejects(()=>project.openFromUrl('file:///C:/project/editor.html',fetchFile),/iniciar_editor.cmd/);
  const previous=project.api;
  await assert.rejects(()=>project.openFromUrl('http://localhost/editor.html',async()=>({ok:false,status:404})),/No se encontró/);
  assert.equal(project.api,previous);
  console.log('Correcto: rutas fijas, todas las recetas, sin caché y errores explícitos.');
})().catch(error=>{console.error(error);process.exitCode=1;});
