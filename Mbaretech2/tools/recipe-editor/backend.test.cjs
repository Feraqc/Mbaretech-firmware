/* Pruebas de la adaptación del frontend original y del guardado revisado. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const vm = require('node:vm');
const C = require('./core.js');
const B = require('./backend.js');
const T = require('./telemetry.js');
const root = path.resolve(__dirname, '../..');
const read = file => fs.readFileSync(path.join(root, file), 'utf8');
const files = new Map(Object.values(B.paths).map(file => [file, read(file)]));
const recipePath = 'include/fsm/recipes/fsm_recipe_turn_calibration.h';
files.set(recipePath, read(recipePath));
const project = new B.Project().load(files);
const recipe = project.recipes[0];
const graph = B.toGraph(project.api, recipe);
assert.equal(graph.parameters.length,2);
assert.equal(B.fromGraph(project.api,graph,{file:recipe.file,namespace:recipe.namespace,macro:recipe.macro})
  .states[1].steps[0].transitions[0].timerParameter,1);
const stateTestRecipe=C.importRecipe(project.api,read('include/fsm/recipes/fsm_recipe_state_test.h'),'fsm_recipe_state_test.h');
const stateTestGraph=B.toGraph(project.api,stateTestRecipe);
assert.equal(stateTestGraph.parameters.length,9);
const namedStateTest=B.exportWithStateNames(project,stateTestGraph,
  {file:stateTestRecipe.file,namespace:stateTestRecipe.namespace,macro:stateTestRecipe.macro});
assert.deepEqual(namedStateTest.recipe.states.map(item=>item.id),
  ['IDLE','FORWARD','BACKWARD','FORWARD_LEFT_45']);
assert(namedStateTest.definitions.includes('StateId::FORWARD_LEFT_45'));
assert.equal(stateTestGraph.nodes[1].stateId,'MOTOR_SEQUENCE');
const namedHeader=C.emit(namedStateTest.api,namedStateTest.recipe);
assert.equal(C.importRecipe(namedStateTest.api,namedHeader,stateTestRecipe.file).states[2].id,'BACKWARD');
assert.equal(stateTestGraph.nodes.find(item=>item.stateId==='MOTOR_SEQUENCE').params.leftParameter,2);
assert.equal(stateTestGraph.nodes.find(item=>item.stateId==='MOTOR_SEQUENCE_1').params.rightParameter,4);
assert.equal(stateTestGraph.nodes.find(item=>item.stateId==='MOTOR_SEQUENCE_2').sequence[0].params.leftParameter,6);
const testRecipe = C.importRecipe(project.api, read('include/fsm/recipes/fsm_recipe_test.h'), 'fsm_recipe_test.h');
const testGraph = B.toGraph(project.api, testRecipe);
const testTimers = testGraph.edges.filter(edge => edge.trigger.type === 'timer');
assert.equal(testTimers.length, 3);
assert.equal(new Set(testTimers.map(edge => edge.trigger.macro)).size, testTimers.length);
assert(testTimers.every(edge => edge.trigger.macro !== 'TIMER'));
assert.equal(C.emit(project.api, B.fromGraph(project.api, testGraph,
  {file:testRecipe.file, namespace:testRecipe.namespace, macro:testRecipe.macro})), C.emit(project.api, testRecipe));
// Import uses the catalog's display name without changing the firmware StateId.
const namedApi = {...project.api, metadata:{...project.api.metadata,
  [recipe.states[1].id]:{...project.api.metadata[recipe.states[1].id], name:'Turn calibration'}}};
const namedGraph = B.toGraph(namedApi, recipe);
assert.equal(namedGraph.nodes[1].name, 'Turn calibration');
assert.equal(namedGraph.nodes[1].stateId, recipe.states[1].id);
assert.equal(B.fromGraph(namedApi, namedGraph, {file:recipe.file, namespace:recipe.namespace, macro:recipe.macro}).states[1].id, recipe.states[1].id);
const identity = {file:recipe.file, namespace:recipe.namespace, macro:recipe.macro};
assert.equal(C.emit(project.api, B.fromGraph(project.api, graph, identity)), C.emit(project.api, recipe));
graph.nodes[1].name = 'Etiqueta visual independiente';
const namedDefinitions = project.definitionsWithNames(graph.nodes);
assert(namedDefinitions.includes('{StateId::'+graph.nodes[1].stateId+', "Etiqueta visual independiente",'));
assert.equal(C.loadAPI(files.get(B.paths.types), namedDefinitions).metadata[graph.nodes[1].stateId].name,
  'Etiqueta visual independiente');
assert.equal(new B.Project().load(new Map(files).set(B.paths.definitions,namedDefinitions)).recipes[0].states[1].id,
  graph.nodes[1].stateId);
const duplicateNames = structuredClone(graph);
duplicateNames.nodes[1].name = duplicateNames.nodes[0].name.toLowerCase();
assert.throws(() => B.fromGraph(project.api, duplicateNames, identity), /Nombre de estado duplicado/);
assert.throws(() => project.definitionsWithNames(duplicateNames.nodes), /Nombre de estado duplicado/);
const duplicateMacroName = files.get(B.paths.definitions).replace(
  '{ConditionId::IR2_DETECTED, "IR2_DETECTED",',
  '{ConditionId::IR2_DETECTED, "IR1_DETECTED",');
assert.throws(() => C.loadAPI(files.get(B.paths.types), duplicateMacroName), /ConditionId: nombre duplicado/);
graph.nodes[1].x = 600;
assert.equal(B.fromGraph(project.api, graph, identity).states[1].id, recipe.states[1].id);
assert.equal(B.fromGraph(project.api, graph, identity).states[1].steps[0].left, recipe.states[1].steps[0].left);
graph.nodes[1].sequence[0].params.left_speed_pct = -12;
assert.equal(B.fromGraph(project.api, graph, identity).states[1].steps[0].left, -12);
const invalid = structuredClone(graph);
invalid.nodes[1].initial = true;
assert.throws(() => B.fromGraph(project.api, invalid, identity), /exactamente/);
invalid.nodes[1].initial = false;
invalid.nodes[1].sequence[0].out_transitions[0].next_step = 'MISSING_STEP';
assert.throws(() => B.fromGraph(project.api, invalid, identity), /destino/);
invalid.nodes[1].sequence[0].out_transitions[0].next_step = 'COMPLETE';
invalid.nodes[1].stateId = 'FORWARD';
assert.throws(() => B.fromGraph(project.api, invalid, identity), /no existe/);
assert.throws(() => new B.Project().load(new Map()), /No se encontró/);
const bad = new Map(files);bad.set('include/fsm/recipes/fsm_recipe_bad.h','enum class MacroId { X };');
assert.equal(new B.Project().load(bad).errors.length, 1);
project.configure({ENABLE_SENSOR_TASK:0});
assert.throws(() => project.preview(recipe), /sensores/);
project.configure({ENABLE_SENSOR_TASK:1});
project.configure({ENABLE_WIFI_TELEMETRY:1,ENABLE_RECIPE_FSM:0});
assert.throws(()=>project.preview(recipe),/Telemetría WiFi requiere telemetría y Recipe FSM o combate/);
project.configure({ENABLE_WIFI_TELEMETRY:0,ENABLE_RECIPE_FSM:1});
const changes = project.preview(recipe);
assert(changes.some(item => item.path === B.paths.config));
assert(changes.every(item => item.path === recipePath || Object.values(B.paths).includes(item.path)));

// Verificar la estructura esencial sin congelar cada píxel del inspector.
const html = read('fsm_context_editor_v31.html');
const markup = html.replace(/\r\n/g,'\n').replace(/<script\b[^>]*>[\s\S]*?<\/script>\s*/g,'');
assert.match(markup,/id="inspector"/);
assert.match(markup,/id="availableMacrosFold"/);
assert.match(html,/class="json-view"/);
assert(!/BUILTIN_FSM_DEFINITIONS_SOURCE|FSM_CONTEXT_EDITOR_DATA_V1|enum class MacroId/.test(html));

// DOM mínimo para probar el cableado de botones, sin automatizar un navegador.
class Element {
  constructor() { this.dataset={};this.style={};this.children=[];this.value='';this.innerHTML='';
    const classes=new Set();this.classList={add:(...names)=>names.forEach(name=>classes.add(name)),
      remove:(...names)=>names.forEach(name=>classes.delete(name)),
      toggle:name=>classes.has(name)?classes.delete(name):classes.add(name),contains:name=>classes.has(name)};
    this.attrs={viewBox:'0 0 1200 700'}; }
  click(){this.onclick?.();}
  setAttribute(key,value){this.attrs[key]=String(value);}
  getAttribute(key){return this.attrs[key]??null;}
  getAttribute(key){return this.attrs[key];}
  appendChild(child){this.children.push(child);}
  querySelectorAll(){return [];}
  querySelector(){return new Element();}
  insertAdjacentHTML(position,html){this.innerHTML+=html;}
  addEventListener(){}
  getBoundingClientRect(){return {width:1200,height:700,left:0,top:0};}
}
const elements = new Map();
const appElement=new Element();
const document = {body:new Element(),getElementById(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);},
  querySelector(selector){return selector==='.app'?appElement:null;},
  createElementNS(){return new Element();},addEventListener(){}};
const alerts=[];
let openedConsole=0;
let editorBridgeReceive;
const bridgeMessages=[];
const context={RecipeCore:C,RecipeBackend:B,RecipeTelemetry:T,
  MbaretechRuntimeTuning:require('./runtimeTuning.js'),
  TelemetryWindowBridge:{createBridge:receive=>{
    editorBridgeReceive=receive;return {publish(message){bridgeMessages.push(message);},close(){}};
  }},
  document,console,structuredClone,alert:message=>alerts.push(message),prompt:()=>recipe.name,
  confirm:()=>true,addEventListener(){},navigator:{},setTimeout,clearTimeout,setInterval:()=>0,
  open:()=>{openedConsole++;return {closed:false,location:{href:'about:blank'},focus(){}};}};
context.window=context;
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
// Acceso de prueba a las acciones existentes sin exponerlas en producción.
vm.runInNewContext(script.replace(/\}\)\(\);\s*$/,
  'window.testPlacement={addBaseNode,addSubFSMNode,moveOutTransition,renderStateLibrary,renderMacroLibrary,deleteSelected,model,telemetrySession,telemetryMapping,renderLiveGraphState,copySelectedStateConfig,pasteSelectedStateConfig,undoStateConfigPaste,redoStateConfigPaste,tuning,renderInspector,refreshLiveInspector,transitionExpression,headerExportGraph};})();'),context);
document.getElementById('telemetryToggleBtn').click();
document.getElementById('telemetryToggleBtn').click();
assert.equal(openedConsole,1);
assert(!appElement.classList.contains('telemetry-open'));

function directoryMock(store, prefix='') {
  return {
    async requestPermission(){return 'granted';},
    async getDirectoryHandle(name){return directoryMock(store,prefix+name+'/');},
    async *entries(){for(const [key,value] of store)if(key.startsWith(prefix)&&!key.slice(prefix.length).includes('/'))yield [key.slice(prefix.length),{kind:'file',async getFile(){return {async text(){return value;}}}}];},
    async getFileHandle(name, options={}) {
      const key=prefix+name;
      if(!store.has(key)&&!options.create){const error=new Error('Ausente');error.name='NotFoundError';throw error;}
      return {
        async getFile(){return {async text(){return store.get(key);}};},
        async createWritable(){return {async write(text){store.set(key,text);},async close(){}};}
      };
    }
  };
}
(async()=>{
  await document.getElementById('exportHeaderBtn').onclick();
  assert.match(alerts.pop(),/Carga|Abre/);
  const store=new Map(files);
  store.set("include/fsm/recipes/fsm_recipe_bad.h", "enum class MacroId { X };");
  await context.FSMFirmware.open(directoryMock(store));
  assert.equal(context.FSMFirmware.diagnostics().length,1);
  assert.equal(alerts.length,0);
  document.getElementById('liveTestBtn').click();
  const live=context.testPlacement;
  assert.match(elements.get('inspector').innerHTML,/Selecciona un estado/);
  live.model.selected={type:'node',id:live.model.nodes[1].id};
  live.renderInspector();
  const liveTable=elements.get('inspector').innerHTML;
  assert(liveTable.includes('class="live-state-list"'));
  assert(liveTable.includes('left_turn_duration')&&liveTable.includes('right_turn_duration'));
  assert(liveTable.includes('Running')&&liveTable.includes('Pending'));
  assert(liveTable.includes('Step 1 left motor')&&liveTable.includes('Direct value'));
  assert.equal((liveTable.match(/class="live-state-card"/g)||[]).length,1);
  live.model.selected={type:'node',id:live.model.nodes[0].id};
  live.renderInspector();
  assert(!elements.get('inspector').innerHTML.includes('left_turn_duration'));
  document.getElementById('liveTestBtn').click();
  const graphBefore=JSON.stringify({nodes:live.model.nodes,edges:live.model.edges});
  const liveNodes=live.model.nodes.map(item=>{const el=new Element();el.setAttribute('data-state-id',item.stateId);return el;});
  const liveEdges=live.model.edges.map(item=>{const el=new Element();el.setAttribute('data-edge-id',item.id);return el;});
  elements.get('nodes').querySelectorAll=selector=>selector==='[data-state-id]'?liveNodes:[];
  elements.get('edges').querySelectorAll=selector=>selector==='[data-edge-id]'?liveEdges:[];
  live.telemetrySession.state.connected=true;
  live.telemetrySession.receive(JSON.stringify({type:'hello',protocol:1,machine:'TURN_CALIBRATION'}));
  document.getElementById('startToggleBtn').click();
  const startRequest=bridgeMessages.findLast(message=>message.type==='start_command');
  assert.equal(startRequest.command.type,'start_set');
  assert.equal(startRequest.command.active,true);
  editorBridgeReceive({type:'start_ack',t:12,transaction:startRequest.command.transaction,
    status:'accepted',active:true,source:'remote'});
  assert.equal(document.getElementById('startToggleBtn').attrs['aria-pressed'],'true');
  editorBridgeReceive({type:'hello',t:13,protocol:1,machine:'TURN_CALIBRATION',paramRevision:0});
  editorBridgeReceive({type:'param_schema',t:14,revision:0,index:0,count:9,
    id:'state.IDLE.left',name:'Left motor',unit:'%',default:0,
    min:-100,max:100,step:1,applyPolicy:'next_state_entry',writable:true});
  live.model.selected={type:'node',id:live.model.nodes[1].id};
  document.getElementById('liveTestBtn').click();
  const partialTable=elements.get('inspector').innerHTML;
  assert(partialTable.includes('left_turn_duration'));
  assert(partialTable.includes('right_turn_duration'));
  assert(partialTable.includes('sincronizando'));
  document.getElementById('liveTestBtn').click();
  assert.equal(live.telemetryMapping().valid,true);
  live.telemetrySession.receive(JSON.stringify({type:'fsm_transition',t:0,machine:'TURN_CALIBRATION',from:'IDLE',to:'TURN_SEQUENCE',condition:'TIMER',timerMs:0,elapsedMs:0}));
  assert(liveNodes.find(el=>el.getAttribute('data-state-id')==='TURN_SEQUENCE').classList.contains('live-current-state'));
  assert(liveNodes.find(el=>el.getAttribute('data-state-id')==='IDLE').classList.contains('live-last-state'));
  assert(liveEdges.some(el=>el.classList.contains('live-transition')));
  live.telemetrySession.receive(JSON.stringify({type:'fsm_step',t:350,machine:'TURN_CALIBRATION',state:'TURN_SEQUENCE',step:0,nextStep:1,condition:'TIMER',timerMs:350,elapsedMs:350}));
  assert(liveNodes.find(el=>el.getAttribute('data-state-id')==='TURN_SEQUENCE').classList.contains('live-active-step'));
  live.telemetrySession.receive(JSON.stringify({type:'fsm_status',t:10,machine:'TURN_CALIBRATION',state:'MISSING',step:-1,elapsedMs:3}));
  assert.match(live.telemetryMapping().warning,/Unknown runtime state: MISSING/);
  assert(liveNodes.every(el=>!el.classList.contains('live-current-state')));
  live.telemetrySession.receive(JSON.stringify({type:'fsm_status',t:11,machine:'TURN_CALIBRATION',state:'IDLE',step:-1,elapsedMs:4}));
  assert.equal(live.telemetryMapping().valid,true);
  assert.equal(JSON.stringify({nodes:live.model.nodes,edges:live.model.edges}),graphBefore);
  const previousId=live.model.nodes[0].stateId;
  live.model.nodes[0].stateId='CHANGED';
  assert.match(live.telemetryMapping().warning,/modified after connection/);
  live.model.nodes[0].stateId=previousId;
  live.telemetrySession.receive(JSON.stringify({type:'hello',protocol:1,machine:'TEST'}));
  assert.match(live.telemetryMapping().warning,/Firmware: TEST/);
  live.telemetrySession.disconnect();
  assert(liveNodes.every(el=>!el.classList.contains('live-current-state')));
  await document.getElementById('exportHeaderBtn').onclick();
  const header=document.getElementById('modalText').value;
  assert.equal(header,C.emit(project.api,recipe));
  assert.equal(document.getElementById('downloadBtn').dataset.filename,recipe.file);
  assert.equal(alerts.length,0);
  await document.getElementById('exportJsonBtn').onclick();
  const saved=JSON.parse(document.getElementById('modalText').value);
  assert.equal(saved.graph.nodes.length,recipe.states.length);
  context.FSMFirmware.previewSave();
  await context.FSMFirmware.saveReviewed();
  assert(store.get(B.paths.config).includes('#define '+recipe.macro));
  context.FSMFirmware.previewSave();
  store.set(B.paths.types,store.get(B.paths.types)+'\n// Cambio externo\n');
  await assert.rejects(()=>context.FSMFirmware.saveReviewed(),/Cambio externo/);
  store.set(B.paths.types,files.get(B.paths.types));
  const actions=context.testPlacement;
  const initialCount=actions.model.nodes.length;
  const placed=actions.model.nodes[0];
  actions.model.mode='add-base';
  const duplicateId=actions.addBaseNode(10,20,placed.stateId);
  assert(duplicateId);
  assert.equal(actions.model.nodes.length,initialCount+1);
  assert.equal(actions.model.selected.id,duplicateId);
  assert.equal(actions.model.nodes.at(-1).stateId,placed.stateId+'_1');
  assert.equal(actions.model.mode,'select');
  assert.equal(alerts.length,0);
  assert(!document.getElementById('stateLibrary').innerHTML.includes('disabled'));
  const sequence=actions.model.nodes.find(item=>item.type==='subfsm');
  actions.model.edges.push({id:'T'+actions.model.nextEdge++,from:sequence.id,to:sequence.id,trigger:{type:'timer',duration_ms:700},action:''});
  const firstCloneId=actions.addBaseNode(50,50,sequence.stateId);
  const firstClone=actions.model.nodes.find(item=>item.id===firstCloneId);
  assert.equal(firstClone.stateId,sequence.stateId+'_1');
  assert.equal(firstClone.type,'subfsm');
  assert(actions.model.edges.some(edge=>edge.from===firstCloneId && edge.to===firstCloneId && edge.trigger.duration_ms===700));
  assert.notEqual(firstClone.sequence[0].id,sequence.sequence[0].id);
  assert.equal(firstClone.sequence[0].out_transitions[0].next_step,firstClone.sequence[1].id);
  assert.equal(firstClone.sequence.at(-1).out_transitions[0].next_step,'COMPLETE');
  assert.notEqual(firstClone.sequence[0].out_transitions[0].id,sequence.sequence[0].out_transitions[0].id);
  const secondCloneId=actions.addBaseNode(100,100,sequence.stateId);
  const secondClone=actions.model.nodes.find(item=>item.id===secondCloneId);
  assert.equal(secondClone.stateId,sequence.stateId+'_2');
  const stateIdToReuse=secondClone.stateId;
  actions.deleteSelected();
  const replacementId=actions.addBaseNode(100,100,sequence.stateId);
  assert.equal(actions.model.nodes.find(item=>item.id===replacementId).stateId,stateIdToReuse);
  firstClone.sequence[0].params.left_speed_pct=-17;
  assert.notEqual(sequence.sequence[0].params.left_speed_pct,-17);
  assert.notEqual(secondClone.sequence[0].params.left_speed_pct,-17);
  assert.equal(actions.model.nodes.filter(item=>item.initial).length,1);
  await document.getElementById('exportHeaderBtn').onclick();
  const instanceHeader=document.getElementById('modalText').value;
  assert(instanceHeader.includes('StateId::'+secondClone.stateId));
  assert(document.getElementById('modalTitle').textContent.includes('FSMDefinitions.h'));
  await document.getElementById('exportJsonBtn').onclick();
  const instanceDoc=JSON.parse(document.getElementById('modalText').value);
  const reopened=new B.Project().load(files);
  reopened.restoreGraph(instanceDoc.graph,instanceDoc.identity,instanceDoc.instances);
  assert(reopened.api.states.includes(secondClone.stateId));
  assert.equal(C.emit(reopened.api,B.fromGraph(reopened.api,instanceDoc.graph,instanceDoc.identity)),instanceHeader);
  const planned=context.FSMFirmware.previewSave();
  const catalog=planned.find(item=>item.path===B.paths.definitions);
  assert(catalog && catalog.after.includes('StateId::'+secondClone.stateId));
  assert(catalog.after.includes('"'+secondClone.stateId+'"'));
  await context.FSMFirmware.saveReviewed();
  assert.equal(store.get(B.paths.definitions),catalog.after);
  const persisted=new B.Project().load(store);
  assert(persisted.recipes.some(item=>item.states.some(state=>state.id===secondClone.stateId)));
  assert.equal(persisted.definitionsChanged(),false);
  const build=path.join(root,'.pio/editor-tests');fs.mkdirSync(build,{recursive:true});
  fs.writeFileSync(path.join(build,'bridge.h'),header.replace('../FSMRecipeTypes.h','fsm/FSMRecipeTypes.h'));
  const overlay=path.join(build,'instances/include');
  fs.cpSync(path.join(root,'include'),overlay,{recursive:true});
  fs.writeFileSync(path.join(overlay,'fsm/FSMDefinitions.h'),catalog.after);
  fs.writeFileSync(path.join(build,'instances/recipe.h'),instanceHeader.replace('../FSMRecipeTypes.h','fsm/FSMRecipeTypes.h'));
  fs.writeFileSync(path.join(build,'instances/test.cpp'),'#include "fsm/StateMachine.h"\n#include "recipe.h"\nint main(){return fsm::validateRecipe(fsm_recipe_turn_calibration::MACHINE)?1:0;}\n');
  // Copiar comportamiento conserva identidad y crea pasos/transiciones independientes.
  actions.model.selected={type:'node',id:sequence.id};
  actions.copySelectedStateConfig();
  const oldName=firstClone.name,oldId=firstClone.stateId,oldX=firstClone.x;
  const beforePaste=JSON.stringify({sequence:firstClone.sequence,params:firstClone.params,
    edges:actions.model.edges.filter(edge=>edge.from===firstClone.id)});
  actions.model.selected={type:'node',id:firstClone.id};
  actions.pasteSelectedStateConfig();
  assert.equal(firstClone.name,oldName);assert.equal(firstClone.stateId,oldId);assert.equal(firstClone.x,oldX);
  assert.notEqual(firstClone.sequence[0].id,sequence.sequence[0].id);
  assert.equal(firstClone.sequence[0].out_transitions[0].next_step,firstClone.sequence[1].id);
  actions.undoStateConfigPaste();
  assert.equal(JSON.stringify({sequence:firstClone.sequence,params:firstClone.params,
    edges:actions.model.edges.filter(edge=>edge.from===firstClone.id)}),beforePaste);
  actions.redoStateConfigPaste();
  assert.equal(firstClone.sequence[0].params.left_speed_pct,sequence.sequence[0].params.left_speed_pct);
  // Live Test debe conservar el campo enfocado mientras llegan paquetes MCU.
  actions.telemetrySession.state.connected=true;
  actions.tuning.ingest({type:'hello',machine:actions.model.name,paramRevision:0,bootId:900});
  actions.tuning.ingest({type:'param_schema',revision:0,index:0,count:1,id:'left_turn_duration',
    name:'Left turn duration',unit:'ms',default:350,min:0,max:60000,step:1,writable:true});
  actions.tuning.ingest({type:'param_values',revision:0,index:0,count:1,id:'left_turn_duration',value:350});
  const pendingInput=new Element();pendingInput.dataset.liveParam='left_turn_duration';
  pendingInput.value='365';pendingInput.setCustomValidity=()=>{};
  pendingInput.matches=selector=>selector==='[data-live-param]';
  const inspectorElement=document.getElementById('inspector');
  inspectorElement.querySelectorAll=selector=>selector==='[data-live-param]'?[pendingInput]:[];
  inspectorElement.contains=element=>element===pendingInput;
  document.activeElement=pendingInput;
  document.getElementById('liveTestBtn').click();
  actions.renderInspector();
  actions.telemetrySession.state.sensors.start=true;
  actions.renderInspector();
  assert.match(inspectorElement.innerHTML,/Apaga START/);
  assert.match(inspectorElement.innerHTML,/data-live-param="left_turn_duration"[^>]*disabled/);
  actions.telemetrySession.state.sensors.start=false;
  actions.telemetrySession.state.running=false;
  actions.renderInspector();
  assert.doesNotMatch(inspectorElement.innerHTML,/data-live-param="left_turn_duration"[^>]*disabled/);
  pendingInput.oninput();
  assert.equal(actions.tuning.pending.get('left_turn_duration'),365);
  assert.equal(pendingInput.onblur,undefined);
  const tunedStateId=actions.model.selected.id;
  actions.model.selected={type:'node',id:actions.model.nodes[0].id};
  actions.renderInspector();
  assert.equal(actions.tuning.pending.get('left_turn_duration'),365);
  assert(!inspectorElement.innerHTML.includes('data-live-param="left_turn_duration"'));
  actions.model.selected={type:'node',id:tunedStateId};
  actions.renderInspector();
  assert(inspectorElement.innerHTML.includes('data-live-param="left_turn_duration"'));
  const tunedTimer=actions.model.nodes.flatMap(item=>(item.sequence||[])
    .flatMap(step=>step.out_transitions||[])).find(item=>item.trigger?.timerParameter===1)?.trigger;
  assert(tunedTimer);
  assert.match(actions.transitionExpression(tunedTimer),/365ms pending/);
  assert.equal(actions.model.parameters.find(item=>item.key==='left_turn_duration').default,350);
  const pendingGraph=actions.headerExportGraph();
  assert.equal(pendingGraph.parameters.find(item=>item.key==='left_turn_duration').default,365);
  assert.equal(pendingGraph.nodes.flatMap(item=>(item.sequence||[])
    .flatMap(step=>step.out_transitions||[])).find(item=>item.trigger?.timerParameter===1)
    .trigger.duration_ms,365);
  assert.equal(actions.model.parameters.find(item=>item.key==='left_turn_duration').default,350);
  document.getElementById('liveApply').click();
  const tuningRequest=bridgeMessages.findLast(message=>message.type==='param_command'&&
    message.command.type==='param_set');
  assert.equal(tuningRequest.command.changes[0].id,'left_turn_duration');
  assert.equal(tuningRequest.command.changes[0].value,365);
  editorBridgeReceive({type:'param_ack',transaction:tuningRequest.command.transaction,
    status:'accepted',revision:1,effective:'next_step_entry'});
  assert.equal(actions.tuning.pending.size,0);
  assert.match(actions.transitionExpression(tunedTimer),/365ms live/);
  const unchangedHtml=inspectorElement.innerHTML;
  actions.refreshLiveInspector();
  assert.equal(inspectorElement.innerHTML,unchangedHtml);
  editorBridgeReceive({type:'start_changed',t:901,start:true});
  assert.match(inspectorElement.innerHTML,/Apaga START/);
  assert.match(inspectorElement.innerHTML,/data-live-param="left_turn_duration"[^>]*disabled/);
  actions.telemetrySession.state.sensors.start=false;
  actions.telemetrySession.state.running=false;
  actions.tuning.ingest({type:'hello',machine:actions.model.name,paramRevision:0,bootId:901});
  actions.renderInspector();
  // Se puede preparar Pending desde la receta sin esperar el catálogo completo.
  assert.doesNotMatch(inspectorElement.innerHTML,/data-live-param="left_turn_duration"[^>]*disabled/);
  pendingInput.value='375';pendingInput.oninput();
  assert.equal(actions.tuning.pending.get('left_turn_duration'),375);
  assert.equal(document.getElementById('liveApply').disabled,false);
  const requestsBeforeSync=bridgeMessages.length;
  document.getElementById('liveApply').click();
  assert(bridgeMessages.slice(requestsBeforeSync).some(item=>item.command?.type==='param_schema_request'));
  assert(bridgeMessages.slice(requestsBeforeSync).some(item=>item.command?.type==='param_values_request'));
  editorBridgeReceive({type:'param_schema',t:902,revision:0,index:0,count:1,
    id:'left_turn_duration',name:'Left turn duration',unit:'ms',default:350,
    min:0,max:60000,step:1,writable:true});
  editorBridgeReceive({type:'param_values',t:903,revision:0,index:0,count:1,
    id:'left_turn_duration',value:350});
  const synchronizedSet=bridgeMessages.findLast(item=>item.command?.type==='param_set');
  assert.equal(synchronizedSet.command.changes[0].value,375);
  document.activeElement=null;
  document.getElementById('liveTestBtn').click();
  actions.renderStateLibrary();
  const currentStates=document.getElementById('stateLibrary').innerHTML;
  assert(currentStates.includes(actions.model.nodes[0].name));
  assert(!currentStates.includes('TEST_FORWARD'));
  actions.renderMacroLibrary();
  const usedConditions=new Set(actions.model.edges.flatMap(item=>item.trigger?.terms||[]));
  for(const state of actions.model.nodes)for(const step of state.sequence||[])
    for(const item of step.out_transitions||[])for(const term of item.trigger?.terms||[])usedConditions.add(term);
  const unusedCondition=project.api.conditions.find(id=>!usedConditions.has(id));
  if(unusedCondition)assert(!document.getElementById('macroLibrary').innerHTML.includes(`data-id="${unusedCondition}"`));
  const base=actions.model.nodes[0].stateId;
  document.getElementById('subfsmBaseState').value=base;
  const newSubId=actions.addSubFSMNode(80,90);
  const newSub=actions.model.nodes.find(item=>item.id===newSubId);
  assert.equal(newSub.type,'subfsm');
  assert.equal(newSub.sequence.length,1);
  assert.equal(actions.model.edges.filter(item=>item.from===newSubId).length,0);
  await document.getElementById('exportHeaderBtn').onclick();
  assert(document.getElementById('modalText').value.includes('StateId::'+newSub.stateId));
  const first={id:'priorityA',from:newSubId,to:actions.model.nodes[0].id,
    trigger:{type:'timer',duration_ms:100},action:''};
  const second={id:'priorityB',from:newSubId,to:actions.model.nodes[0].id,
    trigger:{type:'timer',duration_ms:200},action:''};
  actions.model.edges.push(first,second);
  actions.moveOutTransition(second.id,-1);
  assert.equal(actions.model.edges.filter(item=>item.from===newSubId)[0].id,second.id);
  console.log('Correcto: grafo, referencias centrales, validación, frontend original, botones y guardado revisado.');
})().catch(error=>{console.error(error);process.exitCode=1;});
