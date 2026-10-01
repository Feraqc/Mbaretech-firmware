/* La interfaz edita un modelo gráfico; core.js lo convierte usando el API leído. */
'use strict';
const C=RecipeCore, $=id=>document.getElementById(id);
const paths={types:'include/fsm/FSMRecipeTypes.h',defs:'include/fsm/FSMDefinitions.h',config:'include/buildConfig.h',select:'include/fsm/fsm_recipe_select.h'};
let directory=null, texts=new Map(), api=null, recipes=[], unavailable=[], mappings=[], current=null, values={}, active='', removed=new Set(), pending=[];
const say=text=>$('message').textContent=text;
const run=fn=>async()=>{try{await fn();}catch(e){say(e.message);}};
function element(tag,text,parent,cls){const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;if(parent)parent.append(e);return e;}
function button(parent,text,fn){const b=element('button',text,parent);b.onclick=run(fn);return b;}
function select(parent,label,items,value,fn){const l=element('label',label,parent),s=element('select',undefined,l);for(const item of items){const [val,title]=Array.isArray(item)?item:[item,item];const o=element('option',title,s);o.value=val;}s.value=value;s.onchange=()=>{fn(s.value);};return s;}
function input(parent,label,value,fn,type='text'){const l=element('label',label,parent),i=element('input',undefined,l);i.type=type;i.value=value;i.onchange=()=>fn(type==='number'?Number(i.value):i.value);return i;}
function check(parent,label,value,fn){const l=element('label',undefined,parent,'check'),i=element('input',undefined,l);i.type='checkbox';i.checked=value;element('span',label,l);i.onchange=()=>fn(i.checked);}
function unsaved(){
  const original=C.configValues(texts.get(paths.config)||'');
  const originalRecipe=(texts.get(paths.config)||'').match(/^\s*#define\s+(FSM_ACTIVE_RECIPE_\w+)/m)?.[1]||'';
  return removed.size>0||recipes.some(r=>r.dirty)||active!==originalRecipe||Object.keys(values).some(key=>values[key]!==original[key]);
}
function selected(){if(!api||!current)throw Error('Abre un proyecto y selecciona una receta.');return current;}
async function handle(path,create=false){let h=directory;const parts=path.split('/');for(const part of parts.slice(0,-1))h=await h.getDirectoryHandle(part,{create});return h.getFileHandle(parts.at(-1),{create});}
async function read(path){try{return await(await(await handle(path)).getFile()).text();}catch(e){throw Error('No se encontró: '+path+' ('+e.message+')');}}
async function loadDirectory(){
  const next=new Map();for(const path of Object.values(paths))next.set(path,await read(path));
  let folder;try{folder=await directory.getDirectoryHandle('include');folder=await folder.getDirectoryHandle('fsm');folder=await folder.getDirectoryHandle('recipes');}catch(e){throw Error('No se encontró: include/fsm/recipes/');}
  for await(const [name,h] of folder.entries())if(h.kind==='file'&&/^fsm_recipe_\w+\.h$/.test(name))next.set('include/fsm/recipes/'+name,await(await h.getFile()).text());
  initialize(next,true);
}
function initialize(next,hasFolder=false){
  api=null;current=null;recipes=[];render();
  for(const key of ['types','defs','config'])if(!next.has(paths[key]))throw Error('No se encontró: '+paths[key]);
  const loaded=C.loadAPI(next.get(paths.types),next.get(paths.defs));
  api=loaded;texts=next;values=C.configValues(texts.get(paths.config));mappings=C.selections(texts.get(paths.select)||'');removed=new Set();unavailable=[];
  active=texts.get(paths.config).match(/^\s*#define\s+(FSM_ACTIVE_RECIPE_\w+)/m)?.[1]||'';
  for(const [path,source] of texts)if(path.startsWith('include/fsm/recipes/'))try{
    const r=C.importRecipe(api,source,path.split('/').at(-1)),mapping=mappings.find(m=>m.file===r.file);if(mapping)r.macro=mapping.macro;
    r.originalFile=r.file;r.dirty=false;recipes.push(r);
  }catch(e){unavailable.push({file:path.split('/').at(-1),error:e.message});}
  current=recipes.find(r=>r.macro===active)||recipes[0]||null;
  $('resolved').textContent=Object.values(paths).map(p=>(texts.has(p)?'✓ ':'— ')+p).join('\n')+'\n'+(hasFolder?'✓ ':'— ')+'include/fsm/recipes/\nEsquema del firmware: '+api.version;
  say(unavailable.length?'Archivos incompatibles (se conservan sin convertir):\n'+unavailable.map(x=>x.file+': '+x.error).join('\n'):'Proyecto leído.');render();
}
$('open').onclick=run(async()=>{if(!window.showDirectoryPicker)throw Error('Este navegador no ofrece acceso a carpetas. Usa la selección explícita de archivos.');directory=await showDirectoryPicker();await loadDirectory();});
$('reload').onclick=run(async()=>{if(!directory)throw Error('Selecciona nuevamente los archivos en el modo explícito.');if(unsaved()&&!confirm('Se perderán los cambios sin guardar. ¿Recargar?'))return;await loadDirectory();});
$('loadFiles').onclick=run(async()=>{const next=new Map();for(const i of document.querySelectorAll('[data-support]'))if(i.files[0])next.set(i.dataset.support,await i.files[0].text());for(const f of $('recipeFiles').files)next.set('include/fsm/recipes/'+f.name,await f.text());directory=null;initialize(next);});
function unique(r,exclude=null){if(recipes.some(x=>x!==exclude&&(x.file===r.file||x.macro===r.macro))||unavailable.some(x=>x.file===r.file))throw Error('Ya existe una receta o archivo con ese nombre normalizado.');}
$('new').onclick=run(()=>{if(!api)throw Error('Abre primero el proyecto de firmware.');const name=prompt('Nombre de la nueva receta:');if(!name)return;const r=C.freshRecipe(api,name);unique(r);r.dirty=true;recipes.push(r);current=r;active=r.macro;render();});
$('duplicate').onclick=run(()=>{const old=selected(),name=prompt('Nombre de la copia:',old.name+' copia');if(!name)return;const r={...structuredClone(old),...C.normalize(name),name,originalFile:null,dirty:true};unique(r);recipes.push(r);current=r;active=r.macro;render();});
$('rename').onclick=run(()=>{const r=selected(),name=prompt('Nuevo nombre:',r.name);if(!name)return;const identity=C.normalize(name);unique(identity,r);if(r.originalFile&&r.originalFile!==identity.file)removed.add(r.originalFile);const wasActive=active===r.macro;Object.assign(r,identity,{name,dirty:true});if(wasActive)active=r.macro;render();});
$('delete').onclick=run(()=>{const r=selected();if(!confirm('¿Preparar la eliminación de '+r.name+'? Se revisará antes de escribir.'))return;if(r.originalFile)removed.add(r.originalFile);recipes=recipes.filter(x=>x!==r);current=recipes[0]||null;if(active===r.macro)active=current?.macro||'';render();});
function catalog(id,names){const p=$(id);p.replaceChildren();for(const name of names){const box=element('div',undefined,p,'catalog');element('strong',name,box);element('span',api.metadata[name]?.description||'Sin descripción en el catálogo.',box);}}
function changed(){current.dirty=true;}
function render(){
  $('recipes').replaceChildren();$('editor').replaceChildren();$('config').replaceChildren();
  if(!api){for(const id of ['statesCatalog','conditionsCatalog','constants'])$(id).replaceChildren();$('resolved').textContent='No hay proyecto válido cargado.';return;}
  for(const r of recipes){const b=button($('recipes'),(r===current?'● ':'')+r.name+(r.dirty?' *':''),()=>{current=r;render();});b.className='recipe'+(r===current?' selected':'');}
  for(const x of unavailable)element('p',x.file+' — incompatible',$('recipes'));
  catalog('statesCatalog',api.states);catalog('conditionsCatalog',api.conditions);$('constants').replaceChildren();for(const [key,value]of Object.entries(api.constants))element('div',key+' = '+value,$('constants'),'catalog');
  $('recipeTitle').textContent=current?current.name:'Selecciona una receta';
  if(current)renderRecipe();renderConfig();
}
function command(parent,item){const row=element('div',undefined,parent,'row');select(row,'MotionId',api.enums.MotionId,item.motion,v=>{item.motion=v;changed();});input(row,'Motor izquierdo (%)',item.left,v=>{item.left=v;changed();});input(row,'Motor derecho (%)',item.right,v=>{item.right=v;changed();});}
function transitions(parent,items,sub,step){
  element('h4','Transiciones de salida',parent);
  items.forEach((t,index)=>{
    const box=element('div',undefined,parent,'edge'),row=element('div',undefined,box,'row');
    element('span',String(index+1)+'.',row);
    select(row,'Condición',api.enums.TriggerType.filter(x=>sub||x!=='COMPLETION'),t.type,v=>{t.type=v;changed();render();});
    const targets=step?[[String(-1),'END · STEP_COMPLETE'],...subSteps().map((_,i)=>[String(i),'Paso '+i])]:current.states.map(s=>s.id);
    select(row,step?'Siguiente paso':'Siguiente estado',targets,String(t.target),v=>{t.target=step?Number(v):v;changed();});
    if(t.type==='TIMER')input(row,'Duración (ms o constante)',t.timer,v=>{t.timer=v;changed();});
    if(t.type==='SENSOR'){
      t.terms.forEach((term,i)=>{const r=element('div',undefined,box,'row');if(i)select(r,'Operador',api.enums.LogicOp,t.operators[i-1],v=>{t.operators[i-1]=v;changed();});select(r,'ConditionId',api.conditions.map(x=>[x,x+' · '+(api.metadata[x]?.description||'')]),term,v=>{t.terms[i]=v;changed();});button(r,'Quitar condición',()=>{t.terms.splice(i,1);if(t.operators.length)t.operators.splice(Math.max(0,i-1),1);changed();render();});});
      button(box,'+ Condición',()=>{if(t.terms.length)t.operators.push(api.enums.LogicOp[0]);t.terms.push(api.conditions[0]);changed();render();});
    }
    button(row,'↑',()=>{if(index){[items[index-1],items[index]]=[items[index],items[index-1]];changed();render();}}).ariaLabel='Subir prioridad de transición';
    button(row,'Quitar transición',()=>{items.splice(index,1);changed();render();});
  });
  button(parent,'+ Transición',()=>{items.push({type:'TIMER',timer:0,terms:[],operators:[],target:step?-1:current.initialState});changed();render();});
}
let editingState=null;
function subSteps(){return editingState.steps;}
function renderRecipe(){
  const p=$('editor');
  const map=element('details',undefined,p);element('summary','Mapa de transiciones',map);
  for(const state of current.states){
    element('strong',state.id+(state.id===current.initialState?' · inicial':''),map);
    for(const edge of state.transitions)element('p',state.id+' → '+edge.target+' · '+edge.type,map);
    for(const [i,step]of state.steps.entries())for(const edge of step.transitions)element('p','Paso '+i+' → '+(edge.target===-1?'END':edge.target)+' · '+edge.type,map);
    if(!state.transitions.length&&!state.steps.length)element('p','Sin transiciones de salida.',map);
  }
  element('p',current.file+' · namespace '+current.namespace,p);
  select(p,'Estado inicial',current.states.map(s=>s.id),current.initialState,v=>{current.initialState=v;changed();});
  current.states.forEach((state,index)=>{
    editingState=state;const box=element('div',undefined,p,'card'),row=element('div',undefined,box,'row');
    select(row,'StateId',api.states,state.id,v=>{const old=state.id;state.id=v;if(current.initialState===old)current.initialState=v;for(const s of current.states)for(const t of s.transitions)if(t.target===old)t.target=v;changed();render();});
    select(row,'Tipo de estado',api.enums.StateKind,state.kind,v=>{if(v==='MOTOR'&&state.steps.length&&!confirm('¿Quitar los pasos de este estado?')){render();return;}state.kind=v;state.steps=v==='SUBFSM'?(state.steps.length?state.steps:[C.freshStep(api)]):[];changed();render();});
    button(row,'Eliminar estado',()=>{current.states.splice(index,1);changed();render();});
    element('p',api.metadata[state.id]?.description||'Sin descripción en el firmware.',box);command(box,state);
    if(state.kind==='SUBFSM'){
      check(box,'Permitir retención al completar (allowHoldOnCompletion)',state.hold,v=>{state.hold=v;changed();});
      state.steps.forEach((step,i)=>{const s=element('div',undefined,box,'step');element('h3','Paso '+i,s);command(s,step);transitions(s,step.transitions,false,true);button(s,'Eliminar paso',()=>{state.steps.splice(i,1);for(const other of state.steps)for(const t of other.transitions)if(t.target>i)t.target--;else if(t.target===i)t.target=-2;changed();render();});});
      button(box,'+ Paso',()=>{state.steps.push(C.freshStep(api));changed();render();});
    }
    transitions(box,state.transitions,state.kind==='SUBFSM',false);
  });
  button(p,'+ Estado',()=>{const id=api.states.find(x=>!current.states.some(s=>s.id===x));if(!id)throw Error('No quedan StateId disponibles. Añádelos en FSMDefinitions.h y recarga.');current.states.push({id,kind:'MOTOR',...C.freshStep(api),steps:[],hold:false});changed();render();});
  button(p,'Validar receta',()=>{const errors=C.validate(api,current);say(errors.length?'No se puede generar la receta.\n• '+errors.join('\n• '):'Receta válida para el API cargado.');});
}
const programs={ENABLE_FSM:'FSM existente',ENABLE_RECIPE_FSM:'Recipe FSM',ENABLE_GYRO_TEST:'Prueba de gyro',ENABLE_MOTOR_TEST:'Prueba de motores',ENABLE_MOVEMENT_TEST:'Prueba de movimientos',ENABLE_LINE_TEST:'Prueba de línea',ENABLE_TURN_CALIBRATION:'Diagnóstico antiguo no soportado'};
const groups={'Hardware':{ENABLE_MOTORS:'Motores',ENABLE_SENSOR_TASK:'Tarea de sensores',ENABLE_LINE_SENSORS:'Sensores de línea',ENABLE_IR_SENSORS:'Sensores IR',ENABLE_DIP_SWITCHES:'DIP switches',ENABLE_GYRO:'IMU'},'Comunicación':{ENABLE_SERIAL:'Serial',ENABLE_BLE:'BLE',ENABLE_LOGGING:'Logging'},'Depuración':{FSM_CONSOLE_COMPACT:'Consola FSM compacta',FORCE_START_ACTIVE:'Forzar START activo',ENABLE_TASK_TIMING:'Medición de tiempos',ENABLE_DEBUG:'Mensajes de depuración'}};
function renderConfig(){const p=$('config');select(p,'Programa',[['','Solo sensores'],...Object.entries(programs).filter(([k])=>k in values)],Object.keys(programs).find(k=>values[k])||'',v=>{for(const k of Object.keys(programs))if(k in values)values[k]=Number(k===v);if('ENABLE_LEGACY_MOVEMENTS'in values)values.ENABLE_LEGACY_MOVEMENTS=0;renderConfigReplace();});
  select(p,'Receta activa',recipes.map(r=>[r.macro,r.name]),active,v=>{active=v;});
  for(const [group,options]of Object.entries(groups)){element('h3',group,p);const row=element('div',undefined,p,'configGroup');for(const [key,label]of Object.entries(options))if(key in values)check(row,label,values[key],v=>{values[key]=Number(v);});else element('p','No se encontró '+key,p);}
  element('p','DEPURACIÓN: forzar START con motores habilitados permite movimiento al arrancar con datos válidos. ENABLE_MOTORS=0 conserva la salida física desactivada.',p,'warning');
}
function renderConfigReplace(){$('config').replaceChildren();renderConfig();}
function validateConfig(){
  const errors=[];const need=(when,ok,message)=>{if(when&&!ok)errors.push(message);};
  need(values.ENABLE_RECIPE_FSM,values.ENABLE_SENSOR_TASK,'Recipe FSM requiere la tarea de sensores.');need(values.ENABLE_RECIPE_FSM,values.ENABLE_SERIAL||values.ENABLE_LOGGING,'Recipe FSM requiere Serial o Logging.');
  need(values.ENABLE_BLE,values.ENABLE_LOGGING,'BLE requiere Logging.');need(values.ENABLE_LOGGING,values.ENABLE_SERIAL||values.ENABLE_BLE,'Logging requiere un transporte.');need(values.ENABLE_DEBUG,values.ENABLE_SERIAL,'Depuración requiere Serial.');need(values.ENABLE_TASK_TIMING,values.ENABLE_SENSOR_TASK,'Medición de tiempos requiere sensores.');need(values.ENABLE_GYRO_TEST,values.ENABLE_GYRO&&values.ENABLE_SERIAL&&!values.ENABLE_LOGGING,'Prueba de gyro requiere IMU y Serial, sin Logging.');need(values.ENABLE_FSM,values.ENABLE_SENSOR_TASK&&values.ENABLE_LINE_SENSORS&&values.ENABLE_IR_SENSORS,'FSM existente requiere tarea de sensores, línea e IR.');
  need(values.ENABLE_SENSOR_TASK,!(values.ENABLE_MOVEMENT_TEST||values.ENABLE_LINE_TEST||values.ENABLE_TURN_CALIBRATION),'Los diagnósticos de lectura directa requieren desactivar la tarea de sensores.');
  need(values.ENABLE_MOVEMENT_TEST,values.ENABLE_SERIAL&&values.ENABLE_LINE_SENSORS&&values.ENABLE_IR_SENSORS&&values.ENABLE_DIP_SWITCHES,'Movimientos requiere Serial, línea, IR y DIP.');
  need(values.ENABLE_MOTOR_TEST,values.ENABLE_MOTORS&&values.ENABLE_SERIAL,'Prueba de motores requiere motores y Serial.');
  need(values.ENABLE_LINE_TEST,values.ENABLE_LINE_SENSORS&&values.ENABLE_SERIAL,'Prueba de línea requiere línea y Serial.');
  need(values.ENABLE_TURN_CALIBRATION,false,'El diagnóstico antiguo no tiene punto de entrada; usa una receta de giro.');
  if(Object.keys(programs).reduce((sum,k)=>sum+(values[k]||0),0)>1)errors.push('Solo puede haber un programa activo.');
  if(errors.length)throw Error(errors.join('\n'));
}
function prepare(){
  if(!api)throw Error('Abre un proyecto.');validateConfig();
  if(!texts.has(paths.select))throw Error('Selecciona también fsm_recipe_select.h para actualizar la selección.');
  const chosen=recipes.find(r=>r.macro===active);if(!chosen)throw Error('Selecciona una receta activa compatible.');
  const changes=new Map();
  for(const r of recipes)if(r.dirty||r===chosen&&!texts.has('include/fsm/recipes/'+r.file))changes.set('include/fsm/recipes/'+r.file,C.emit(api,r));
  // Validar también la seleccionada aunque no se reescriba.
  C.emit(api,chosen);
  const entries=mappings.filter(m=>!removed.has(m.file)&&!recipes.some(r=>r.file===m.file||r.originalFile===m.file));entries.push(...recipes.map(({file,namespace,macro})=>({file,namespace,macro})));
  changes.set(paths.config,C.patchConfig(texts.get(paths.config),values,active));
  if(JSON.stringify(entries)!==JSON.stringify(mappings))changes.set(paths.select,C.patchSelection(texts.get(paths.select),entries));
  for(const file of removed)if(!recipes.some(r=>r.file===file))changes.set('include/fsm/recipes/'+file,null);
  pending=[...changes].filter(([path,after])=>after!==texts.get(path)).map(([path,after])=>({path,before:texts.get(path),after}));
  if(!pending.length){say('No hay cambios pendientes.');return;}
  $('changes').replaceChildren();$('saveStatus').textContent='';
  for(const item of pending){const d=element('details',undefined,$('changes'));d.open=true;element('summary',(item.after===null?'Eliminar: ':item.before===undefined?'Crear: ':'Modificar: ')+item.path,d);const diff=element('div',undefined,d,'diff');const old=element('div',undefined,diff);element('strong','Antes',old);element('pre',item.before??'(archivo nuevo)',old);const next=element('div',undefined,diff);element('strong','Después',next);element('pre',item.after??'(eliminar archivo)',next);}
  $('confirm').textContent=directory?'Confirmar y guardar':'Descargar archivos revisados';$('preview').showModal();
}
$('saveRecipe').onclick=run(prepare);$('saveConfig').onclick=run(prepare);$('cancel').onclick=()=>$('preview').close();
$('confirm').onclick=async()=>{
  const done=[];$('confirm').disabled=true;
  try{
    if(directory){
      if(await directory.requestPermission({mode:'readwrite'})!=='granted')throw Error('No se concedió permiso de escritura.');
      // Verificar todos los originales antes de escribir para evitar perder cambios externos.
      for(const path of [paths.types,paths.defs])if(await read(path)!==texts.get(path))throw Error('El API del firmware cambió: '+path+'. Recarga antes de guardar.');
      for(const item of pending){let actual;try{actual=await(await(await handle(item.path)).getFile()).text();}catch(e){if(e.name!=='NotFoundError')throw e;}if(actual!==item.before)throw Error('El archivo cambió externamente: '+item.path+'. Recarga el proyecto.');}
      for(const item of pending){if(item.after===null){let h=directory;const parts=item.path.split('/');for(const p of parts.slice(0,-1))h=await h.getDirectoryHandle(p);await h.removeEntry(parts.at(-1));}else{const writer=await(await handle(item.path,true)).createWritable();await writer.write(item.after);await writer.close();}done.push(item.path);}
      await loadDirectory();$('preview').close();say('Guardado:\n'+done.join('\n'));
    }else{
      for(const item of pending){if(item.after===null)continue;const a=document.createElement('a'),url=URL.createObjectURL(new Blob([item.after],{type:'text/plain;charset=utf-8'}));a.href=url;a.download=item.path.split('/').at(-1);a.click();setTimeout(()=>URL.revokeObjectURL(url),30000);}
      $('saveStatus').textContent='Descargas preparadas. Coloca cada archivo en la ruta indicada; elimina manualmente los archivos marcados. El proyecto original no se ha modificado.';
    }
  }catch(e){$('saveStatus').textContent=e.message+(done.length?'\nYa guardados: '+done.join(', '):'');}finally{$('confirm').disabled=false;}
};
window.addEventListener('beforeunload',event=>{if(unsaved()){event.preventDefault();event.returnValue='';}});
