/* Analizador acotado del API actual: no ejecuta C++ ni convierte formatos antiguos. */
(function (root) {
'use strict';
const fail = message => { throw new Error(message); };
const clean = text => text.replace(/"(?:\\.|[^"\\])*"|\/\*[\s\S]*?\*\/|\/\/[^\n]*/g, part => part.startsWith('"') ? part : ' ');
const split = text => {
  let depth = 0, quoted = false, escaped = false, start = 0; const result = [];
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (quoted) { if (escaped) escaped = false; else if (c === '\\') escaped = true; else if (c === '"') quoted = false; continue; }
    if (c === '"') quoted = true;
    else if ('{(['.includes(c)) depth++;
    else if ('})]'.includes(c)) depth--;
    else if (c === ',' && depth === 0) { result.push(text.slice(start, i).trim()); start = i + 1; }
  }
  if (quoted || depth !== 0) fail('Inicializador C++ incompleto.');
  const last = text.slice(start).trim(); if (last) result.push(last); return result;
};
const enumValues = (source, name) => {
  const found = clean(source).match(new RegExp('enum\\s+class\\s+' + name + '(?:\\s*:\\s*\\w+)?\\s*\\{([^}]+)\\}'));
  if (!found) fail('No se encontró enum class ' + name + '.');
  const values = split(found[1]).map(x => x.split('=')[0].trim()).filter(x => x !== 'COUNT');
  if (values.some(value => !/^[A-Za-z_]\w*$/.test(value)) || new Set(values).size !== values.length) fail('Enum incompatible: ' + name);
  return values;
};
const normalize = name => {
  const slug = name.normalize('NFD').replace(/[\u0300-\u036f]/g, '').toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
  if (!slug) fail('El nombre necesita letras o números.');
  return { file: 'fsm_recipe_' + slug + '.h', namespace: 'fsm_recipe_' + slug, macro: 'FSM_ACTIVE_RECIPE_' + slug.toUpperCase() };
};
function loadAPI(types, definitions) {
  if (!/namespace\s+fsm\s*\{/.test(clean(types)) || !/using\s+fsm_defs::StateId\s*;/.test(clean(types)) || !/using\s+fsm_defs::ConditionId\s*;/.test(clean(types)) || !/namespace\s+fsm_defs\s*\{/.test(clean(definitions))) fail('FSMRecipeTypes.h incompatible: namespaces o catálogos no reconocidos.');
  const structs = {}, enums = {}, constants = {};
  for (const m of clean(types).matchAll(/struct\s+(\w+)\s*\{([^}]+)\}/g)) {
    structs[m[1]] = m[2].split(';').map(x => x.trim().replace(/\s*=\s*[^=]+$/,'')).filter(Boolean).map(field => {
      const f = field.match(/^(.+?)\s*(\w+)$/); if (!f) fail('Declaración no compatible: ' + field);
      return { name: f[2], type: f[1].trim() };
    });
  }
  const required = ['MotorCommand','ConditionExpression','TriggerRecipe','StateTransitionRecipe','StepTransitionRecipe','StepRecipe','SubFsmRecipe','StateRecipe','MachineRecipe'];
  for (const name of required) if (!structs[name]) fail('FSMRecipeTypes.h incompatible con esta versión del editor: falta ' + name);
  for (const name of ['MotionId','TriggerType','LogicOp','StateKind']) enums[name] = enumValues(types, name);
  // Las operaciones soportadas son semántica del editor, no catálogos de reemplazo.
  for (const [name, values] of Object.entries({TriggerType:['TIMER','SENSOR','COMPLETION'], LogicOp:['AND','OR'], StateKind:['MOTOR','SUBFSM']}))
    if (enums[name].join(',') !== values.join(',')) fail('FSMRecipeTypes.h incompatible: ' + name);
  const version = clean(definitions).match(/SCHEMA_VERSION\s*=\s*(\d+)/);
  if (!version || Number(version[1]) !== 2 || !/STEP_COMPLETE\s*=\s*-1\s*;/.test(clean(types))) fail('FSMRecipeTypes.h incompatible con esta versión del editor.');
  const states = enumValues(definitions, 'StateId'), conditions = enumValues(definitions, 'ConditionId');
  const metadata = {};
  for (const m of clean(definitions).matchAll(/\{(?:StateId|ConditionId)::(\w+)\s*,\s*("(?:\\.|[^"\\])*")\s*,\s*("(?:\\.|[^"\\])*")\s*\}/g))
    metadata[m[1]] = { name: JSON.parse(m[2]), description: JSON.parse(m[3]) };
  // Firmware uses these names in logs; duplicate names make editor choices
  // and telemetry ambiguous even when the underlying enum IDs are unique.
  for (const [kind, ids] of [['StateId', states], ['ConditionId', conditions]]) {
    const names = new Set();
    for (const id of ids.filter(value => value !== 'COUNT')) {
      const name = metadata[id]?.name?.trim() || id;
      const key = name.toLocaleLowerCase();
      if (names.has(key)) fail(kind + ': nombre duplicado en metadatos: ' + name + '.');
      names.add(key);
    }
  }
  for (const ns of ['motor','timers','runtime']) {
    const body = clean(definitions).match(new RegExp('namespace\\s+' + ns + '\\s*\\{([\\s\\S]*?)\\n\\}'));
    if (body) for (const m of body[1].matchAll(/constexpr\s+\w+\s+(\w+)\s*=\s*([^;]+);/g)) constants['fsm_defs::' + ns + '::' + m[1]] = m[2].trim();
  }
  const api = { structs, enums, states, conditions, metadata, constants, version: Number(version[1]) };
  for (const key of ['MIN_PERCENT','MAX_PERCENT']) number(api, 'fsm_defs::motor::' + key);
  // Comprobar campos y tipos mediante los mismos valores semánticos usados al exportar.
  // No se mantiene una segunda tabla de orden/layout de structs.
  const probe = freshRecipe(api, 'Verificación');
  probe.states[0].kind = 'SUBFSM'; probe.states[0].steps = [freshStep(api)];
  probe.states[0].steps[0].transitions = [{type:'SENSOR', terms:[conditions[0]], operators:[], target:-1, timer:0}];
  probe.states[0].transitions = [{type:'COMPLETION', target:probe.initialState, timer:0, terms:[], operators:[]}];
  emit(api, probe);
  return api;
}
function number(api, value, seen = new Set()) {
  if (typeof value === 'number' && Number.isInteger(value)) return value;
  const text = String(value).trim();
  if (/^-?\d+$/.test(text)) return Number(text);
  const negative = text.startsWith('-'), key = negative ? text.slice(1) : text;
  if (!(key in api.constants) || seen.has(key)) fail('Expresión numérica no compatible: ' + text);
  seen.add(key); return (negative ? -1 : 1) * number(api, api.constants[key], seen);
}
function aggregate(api, type, values) {
  const fields = api.structs[type];
  if (!fields || Object.keys(values).some(name=>!fields.some(field=>field.name===name))) fail('FSMRecipeTypes.h incompatible: campos de ' + type);
  // Los agregados C++ pueden omitir los campos finales con default member initializer.
  const present=fields.slice(0,Object.keys(values).length);
  if(present.some(field=>!(field.name in values)))fail('FSMRecipeTypes.h incompatible: campos de ' + type);
  return '{' + present.map(field => {
    const item = values[field.name];
    if (!item || field.type.replace(/\s+/g,'') !== item[0].replace(/\s+/g,'')) fail('FSMRecipeTypes.h incompatible: ' + type + '.' + field.name);
    return item[1];
  }).join(', ') + '}';
}
function freshStep(api) { return {motion:api.enums.MotionId.includes('STOP') ? 'STOP' : api.enums.MotionId[0], left:0, right:0, transitions:[]}; }
function freshRecipe(api, name) {
  const id = api.states[0]; if (!id) fail('El catálogo de estados está vacío.');
  return {name, ...normalize(name), initialState:id, parameters:[], states:[{id,kind:'MOTOR',...freshStep(api), steps:[],hold:false}]};
}
function validate(api, recipe) {
  const errors = []; const add = text => errors.push(text);
  if (!recipe.name?.trim()) add('El nombre de la receta no puede estar vacío.');
  if (!/^[A-Za-z_]\w*$/.test(recipe.namespace) || !/^fsm_recipe_\w+\.h$/.test(recipe.file) || !/^FSM_ACTIVE_RECIPE_\w+$/.test(recipe.macro)) add('Identidad de archivo, namespace o selector incompatible.');
  if (!recipe.states.length || recipe.states.length > 255) add('La receta necesita entre 1 y 255 estados.');
  const ids = recipe.states.map(s => s.id);
  if (ids.filter(id => id === recipe.initialState).length !== 1) add('Debe existir exactamente un estado inicial.');
  if (new Set(ids).size !== ids.length) add('Cada StateId debe ser único.');
  const parameters=recipe.parameters||[];
  if(parameters.length>64)add('La receta supera el límite de 64 parámetros runtime.');
  const paramById=new Map();const paramKeys=new Set();
  for(const p of parameters){
    if(!Number.isInteger(p.id)||p.id<1||p.id>65535||paramById.has(p.id))add('ParameterId duplicado o inválido.');
    else paramById.set(p.id,p);
    if(!/^[a-z][a-z0-9_]*$/.test(p.key||'')||paramKeys.has(p.key))add('Clave de parámetro duplicada o inválida.');
    paramKeys.add(p.key);
    if(!p.name?.trim())add('Nombre de parámetro vacío.');
    if(p.writable!==undefined&&typeof p.writable!=='boolean')add('Acceso de parámetro inválido.');
    if(!['Percent','Milliseconds'].includes(p.unit)||!['Immediate','NextStateEntry','NextStepEntry','NextMachineStart','StoppedOnly'].includes(p.policy))add('Unidad o política de parámetro inválida.');
    if(!Number.isInteger(p.min)||!Number.isInteger(p.max)||!Number.isInteger(p.default)||!Number.isInteger(p.step)||
       p.min< -2147483648||p.max>2147483647||
       p.min>p.max||p.step<1||p.default<p.min||p.default>p.max||(p.default-p.min)%p.step) add('Rango de parámetro inválido.');
  }
  const ref=(id,unit,label,entryPolicy)=>{if(id===undefined||id===null||id===0)return;
    const p=paramById.get(Number(id));if(!p||p.unit!==unit||
      ![entryPolicy,'NextMachineStart','StoppedOnly'].includes(p.policy)||
      (unit==='Percent'&&(p.min< -100||p.max>100))||
      (unit==='Milliseconds'&&p.min<0))add(label+': referencia de parámetro inválida.');};
  const numeric = (value, low, high, label) => { try { const n = number(api,value); if (n < low || n > high) add(label + ': fuera de rango.'); } catch(e) { add(label + ': ' + e.message); } };
  const command = (item, label, entryPolicy) => {
    if (!api.enums.MotionId.includes(item.motion)) add(label + ': MotionId desconocido.');
    for (const value of [item.left,item.right]) numeric(value,number(api,'fsm_defs::motor::MIN_PERCENT'),number(api,'fsm_defs::motor::MAX_PERCENT'),label + ': motor');
    ref(item.leftParameter,'Percent',label,entryPolicy);ref(item.rightParameter,'Percent',label,entryPolicy);
  };
  const transitions = (items, sub, targets, label, entryPolicy) => {
    if (items.length > 255) add(label + ': demasiadas transiciones.');
    for (const t of items) {
      if (!targets.includes(t.target)) add(label + ': destino inexistente.');
      if (!api.enums.TriggerType.includes(t.type)) add(label + ': TriggerType desconocido.');
      if (t.type === 'COMPLETION' && !sub) add(label + ': COMPLETION requiere salida superior de SubFSM.');
      if (t.type === 'TIMER') numeric(t.timer,0,4294967295,label + ': timer');
      ref(t.timerParameter,'Milliseconds',label,entryPolicy);
      if(t.type!=='TIMER'&&t.timerParameter)add(label+': sólo TIMER admite parámetro.');
      if (t.type === 'SENSOR') {
        if (!t.terms.length || t.terms.length > 255) add(label + ': SENSOR necesita entre 1 y 255 condiciones.');
        if (t.operators.length !== t.terms.length-1) add(label + ': operadores = condiciones - 1.');
        for (const term of t.terms) if (!api.conditions.includes(term)) add(label + ': ConditionId no existe: ' + term);
        for (const op of t.operators) if (!api.enums.LogicOp.includes(op)) add(label + ': LogicOp desconocido.');
      }
    }
  };
  for (const state of recipe.states) {
    if (!api.states.includes(state.id)) add('El estado ' + state.id + ' usa un StateId que no existe en FSMDefinitions.h.');
    if (!api.enums.StateKind.includes(state.kind)) add(state.id + ': StateKind desconocido.');
    command(state,state.id,'NextStateEntry'); transitions(state.transitions,state.kind==='SUBFSM',ids,state.id,'NextStateEntry');
    if (state.kind !== 'SUBFSM') { if (state.steps.length) add(state.id + ': un estado MOTOR no puede contener pasos.'); continue; }
    if (!state.steps.length || state.steps.length > 255) add(state.id + ': SubFSM necesita entre 1 y 255 pasos.');
    state.steps.forEach((step,i) => { command(step,state.id+' paso '+i,'NextStepEntry'); transitions(step.transitions,false,[-1,...state.steps.map((_,j)=>j)],state.id+' paso '+i,'NextStepEntry'); });
    const seen = new Set(), pending = [0]; let complete = false;
    while (pending.length) { const i=pending.pop(); if (seen.has(i)||!state.steps[i]) continue; seen.add(i); for (const t of state.steps[i].transitions) { if(t.target===-1) complete=true; else pending.push(t.target); } }
    if (complete && !state.hold && !state.transitions.some(t=>t.type==='COMPLETION')) add(state.id + ' necesita una transición COMPLETION o permitir retención.');
  }
  return errors;
}
function emit(api, recipe) {
  const errors = validate(api,recipe); if(errors.length) fail('No se puede generar la receta.\n• ' + errors.join('\n• '));
  const lines = ['#pragma once','','#include "../FSMRecipeTypes.h"','','namespace '+recipe.namespace+' {','using namespace fsm;','','// Los tipos y catálogos pertenecen al firmware; aquí solo se define la receta.'];
  let serial=0; const ag=(type,values)=>aggregate(api,type,values);
  const arr=(type,values)=>{if(!values.length)return 'nullptr'; const name='tabla_'+serial++;lines.push('static const '+type+' '+name+'[] = {\n    '+values.join(',\n    ')+'\n};');return name;};
  const motor=item=>ag('MotorCommand',{left_pct:['int8_t',String(item.left)],right_pct:['int8_t',String(item.right)],
    leftParameter:['ParameterId',String(item.leftParameter||0)],rightParameter:['ParameterId',String(item.rightParameter||0)]});
  const trigger=t=>{
    const terms=t.type==='SENSOR'?t.terms:[], operators=t.type==='SENSOR'?t.operators:[];
    const expression=ag('ConditionExpression',{terms:['const ConditionId*',arr('ConditionId',terms.map(x=>'ConditionId::'+x))],termCount:['uint8_t',String(terms.length)],operators:['const LogicOp*',arr('LogicOp',operators.map(x=>'LogicOp::'+x))],operatorCount:['uint8_t',String(operators.length)]});
    return ag('TriggerRecipe',{type:['TriggerType','TriggerType::'+t.type],timerMs:['uint32_t',t.type==='TIMER'?String(t.timer):'0'],expression:['ConditionExpression',expression],
      timerParameter:['ParameterId',String(t.type==='TIMER'?(t.timerParameter||0):0)]});
  };
  const edges=(items,step)=>arr(step?'StepTransitionRecipe':'StateTransitionRecipe',items.map(t=>ag(step?'StepTransitionRecipe':'StateTransitionRecipe',step?{trigger:['TriggerRecipe',trigger(t)],next_step:['int16_t',t.target===-1?'STEP_COMPLETE':String(t.target)]}:{trigger:['TriggerRecipe',trigger(t)],next_state:['StateId','StateId::'+t.target]})));
  const states=recipe.states.map(state=>{
    let sub='nullptr';
    if(state.kind==='SUBFSM') {
      const steps=arr('StepRecipe',state.steps.map(step=>ag('StepRecipe',{motion:['MotionId','MotionId::'+step.motion],motor:['MotorCommand',motor(step)],out_transitions:['const StepTransitionRecipe*',edges(step.transitions,true)],out_count:['uint8_t',String(step.transitions.length)]})));
      const name='secuencia_'+serial++;lines.push('static const SubFsmRecipe '+name+' = '+ag('SubFsmRecipe',{steps:['const StepRecipe*',steps],step_count:['uint8_t',String(state.steps.length)],allowHoldOnCompletion:['bool',String(state.hold)]})+';');sub='&'+name;
    }
    return ag('StateRecipe',{id:['StateId','StateId::'+state.id],kind:['StateKind','StateKind::'+state.kind],motion:['MotionId','MotionId::'+state.motion],motor:['MotorCommand',motor(state)],out_transitions:['const StateTransitionRecipe*',edges(state.transitions,false)],out_count:['uint8_t',String(state.transitions.length)],subfsm:['const SubFsmRecipe*',sub]});
  });
  const table=arr('StateRecipe',states);
  const parameters=arr('ParameterDefinition',(recipe.parameters||[]).map(p=>ag('ParameterDefinition',{
    id:['ParameterId',String(p.id)],key:['const char*',JSON.stringify(p.key)],name:['const char*',JSON.stringify(p.name)],
    type:['ParameterType','ParameterType::Integer'],unit:['ParameterUnit','ParameterUnit::'+p.unit],
    defaultValue:['int32_t',String(p.default)],minimum:['int32_t',String(p.min)],maximum:['int32_t',String(p.max)],
    step:['int32_t',String(p.step)],policy:['ParameterPolicy','ParameterPolicy::'+p.policy],
    access:['ParameterAccess','ParameterAccess::'+(p.writable===false?'ReadOnly':'Writable')]})));
  lines.push('static const MachineRecipe MACHINE = '+ag('MachineRecipe',{initial_state:['StateId','StateId::'+recipe.initialState],states:['const StateRecipe*',table],state_count:['uint8_t',String(states.length)],name:['const char*',JSON.stringify(recipe.name)],
    parameters:['const ParameterDefinition*',parameters],parameterCount:['uint16_t',String((recipe.parameters||[]).length)]})+';','','} // namespace '+recipe.namespace,'');
  return lines.join('\n');
}
/* Lectura de agregados const del API cargado, sin ejecutar expresiones arbitrarias. */
function importRecipe(api, source, file) {
  const text=clean(source);
  if (/\b(?:enum|struct|MacroId)\b|#error/.test(text)) fail('Receta incompatible: solo se admiten tablas del API actual.');
  const ns=text.match(/namespace\s+(\w+)\s*\{/);if(!ns)fail('Receta sin namespace.');
  const symbols={};
  for(const m of text.matchAll(/(?:static\s+)?(?:constexpr|const)\s+(\w+)\s+(\w+)\s*(\[\s*\])?\s*=\s*([\s\S]*?);/g)) symbols[m[2]]={type:m[1],array:!!m[3],value:m[4].trim()};
  const resolving=new Set();
  const resolve=(value,type,array=false)=>{
    value=value.trim();if(value==='nullptr')return null;
    const key=value.replace(/^&/,'');
    if(symbols[key]){const symbol=symbols[key];if(resolving.has(key)||symbol.type!==type||symbol.array!==array)fail('Referencia incompatible: '+key);resolving.add(key);const result=resolve(symbol.value,type,array);resolving.delete(key);return result;}
    if(array){if(!value.startsWith('{')||!value.endsWith('}'))fail('Tabla no compatible.');return split(value.slice(1,-1)).map(x=>resolve(x,type));}
    if(api.structs[type]){if(!value.startsWith('{')||!value.endsWith('}'))fail('Agregado incompatible: '+type);const parts=split(value.slice(1,-1)),fields=api.structs[type];if(parts.length>fields.length)fail('Campos incompatibles: '+type);return Object.fromEntries(fields.slice(0,parts.length).map((f,i)=>[f.name,parts[i]]));}
    if(type==='ConditionId'||type==='LogicOp')return value.replace(/^(?:fsm_defs::|fsm::)?\w+::/,'');
    fail('Tipo no compatible: '+type);
  };
  const scalar=x=>x.replace(/^(?:fsm_defs::|fsm::)?\w+::/,'');
  const list=(value,type,count)=>{const result=resolve(value,type,true)||[];if(result.length!==number(api,count))fail('Conteo no coincide: '+type);return result;};
  const trig=value=>{const t=resolve(value,'TriggerRecipe'),e=resolve(t.expression,'ConditionExpression');return {type:scalar(t.type),timer:t.timerMs,timerParameter:t.timerParameter?number(api,t.timerParameter):0,terms:list(e.terms,'ConditionId',e.termCount),operators:list(e.operators,'LogicOp',e.operatorCount)};};
  const edges=(value,count,step)=>list(value,step?'StepTransitionRecipe':'StateTransitionRecipe',count).map(t=>({...trig(t.trigger),target:step?(t.next_step==='STEP_COMPLETE'?-1:number(api,t.next_step)):scalar(t.next_state)}));
  const command=item=>{const m=resolve(item.motor,'MotorCommand');return {motion:scalar(item.motion),left:m.left_pct,right:m.right_pct,leftParameter:m.leftParameter?number(api,m.leftParameter):0,rightParameter:m.rightParameter?number(api,m.rightParameter):0};};
  const machine=resolve('MACHINE','MachineRecipe');
  const name=JSON.parse(machine.name);
  const parameters=machine.parameters?list(machine.parameters,'ParameterDefinition',machine.parameterCount).map(p=>({
    id:number(api,p.id),key:JSON.parse(p.key),name:JSON.parse(p.name),unit:scalar(p.unit),
    default:number(api,p.defaultValue),min:number(api,p.minimum),max:number(api,p.maximum),
    step:number(api,p.step),policy:scalar(p.policy),writable:p.access?scalar(p.access)!=='ReadOnly':true})):[];
  const recipe={name,file,namespace:ns[1],macro:'FSM_ACTIVE_RECIPE_'+file.replace(/^fsm_recipe_|\.h$/g,'').toUpperCase(),initialState:scalar(machine.initial_state),parameters,states:list(machine.states,'StateRecipe',machine.state_count).map(s=>{
    const sub=resolve(s.subfsm,'SubFsmRecipe');
    if(sub&&!['true','false'].includes(sub.allowHoldOnCompletion))fail('Política de finalización incompatible.');
    return {id:scalar(s.id),kind:scalar(s.kind),...command(s),transitions:edges(s.out_transitions,s.out_count,false),hold:sub?.allowHoldOnCompletion==='true',steps:sub?list(sub.steps,'StepRecipe',sub.step_count).map(step=>({...command(step),transitions:edges(step.out_transitions,step.out_count,true)})):[]};
  })};
  const errors=validate(api,recipe);if(errors.length)fail(errors.join('\n'));return recipe;
}
function selections(source) {
  return [...source.matchAll(/#(?:if|elif)\s+defined\((FSM_ACTIVE_RECIPE_\w+)\)\s*\n#include\s+"recipes\/(fsm_recipe_\w+\.h)"\s*\nnamespace\s+active_fsm_recipe\s*=\s*(\w+)\s*;/g)].map(m=>({macro:m[1],file:m[2],namespace:m[3]}));
}
function selectionSource(entries) {
  if(!entries.length||new Set(entries.map(e=>e.macro)).size!==entries.length)fail('Selección de recetas ambigua.');
  return '#pragma once\n\n// Selección mantenida por el editor; el API del runtime permanece intacto.\n#if ('+entries.map(e=>'defined('+e.macro+')').join(' + ')+') != 1\n#error "Seleccionar exactamente una receta"\n#endif\n\n'+entries.map((e,i)=>(i?'#elif':'#if')+' defined('+e.macro+')\n#include "recipes/'+e.file+'"\nnamespace active_fsm_recipe = '+e.namespace+';').join('\n')+'\n#endif\n';
}
function patchSelection(source, entries) {
  const generated=selectionSource(entries);
  const guard=/^#if[^\r\n]*defined\(FSM_ACTIVE_RECIPE_\w+\)[^\r\n]*!=\s*1[^\r\n]*$/m;
  const chain=/#if\s+defined\(FSM_ACTIVE_RECIPE_\w+\)[\s\S]*?#endif/;
  const oldEntries=selections(source);
  if(!oldEntries.length||!chain.test(source))fail('Selector incompatible: no se reconoció el bloque de selección actual.');
  const block=source.match(chain)[0];
  // No borrar directivas o código ajenos al selector reconocido.
  const residue=clean(block).replace(/#(?:if|elif)\s+defined\(FSM_ACTIVE_RECIPE_\w+\)/g,'').replace(/#include\s+"recipes\/fsm_recipe_\w+\.h"/g,'').replace(/namespace\s+active_fsm_recipe\s*=\s*\w+\s*;/g,'').replace(/#endif/g,'').trim();
  if(residue)fail('El selector contiene código adicional; no se sobrescribirá.');
  const newline=source.includes('\r\n')?'\r\n':'\n';
  const selected=source.replace(chain,generated.match(chain)[0].replace(/\n/g,newline));
  if(guard.test(selected))return selected.replace(guard,generated.match(guard)[0]);
  // Older selectors had the branch chain but no exactly-one check. Preserve
  // their surrounding content and add the missing guard beside the chain.
  const guardText=generated.match(guard)[0]+newline+'#error "Seleccionar exactamente una receta"'+newline+'#endif'+newline+newline;
  return selected.replace(chain,guardText+'$&');
}
function configValues(source) { return Object.fromEntries([...source.matchAll(/^\s*#define\s+(\w+)\s+([01])\b/gm)].map(m=>[m[1],Number(m[2])])); }
function patchConfig(source, values, macro) {
  let out=source;
  for(const [key,value] of Object.entries(values)) {
    if(!/^[A-Z_]+$/.test(key)||![0,1].includes(value))fail('Opción inválida: '+key);
    const re=new RegExp('^(\\s*#define\\s+'+key+'\\s+)[01](\\b[^\\r\\n]*)','gm');
    if([...out.matchAll(re)].length!==1)fail('No se encontró una opción única: '+key);
    out=out.replace(re,(_,a,b)=>a+value+b);
  }
  const matches=[...out.matchAll(/^([ \t]*)#define[ \t]+FSM_ACTIVE_RECIPE_\w+[^\r\n]*/gm)];
  if(matches.length!==1)fail('buildConfig.h necesita una selección activa única.');
  return out.replace(matches[0][0],matches[0][1]+'#define '+macro);
}
const exported={loadAPI,normalize,number,emit,importRecipe,validate,freshRecipe,freshStep,selections,selectionSource,patchSelection,configValues,patchConfig};
if(typeof module!=='undefined')module.exports=exported;else root.RecipeCore=exported;
})(globalThis);
