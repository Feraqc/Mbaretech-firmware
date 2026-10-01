/* Presentación humana; los mensajes y marcas de tiempo del protocolo no cambian. */
(function (root) {
'use strict';
const EVENT_TYPE_LABELS=Object.freeze({
  hello:'Connection',heartbeat:'Heartbeat',stats:'Telemetry Stats',
  sensors:'Sensor Snapshot',sensor_snapshot:'Sensor Snapshot',
  ir_changed:'IR Change',line_changed:'Line Change',start_changed:'START Change',start_ack:'START ACK',
  motor:'Motor Command',motor_command:'Motor Command',
  fsm_status:'FSM Status',fsm_state:'FSM State',fsm_transition:'FSM Transition',
  fsm_step:'FSM Step',fsm_step_transition:'FSM Step',
  task_timing:'Task Timing',imu:'IMU',error:'Error',log:'Log',
  param_schema:'Parameter Schema',param_values:'Parameter Values',
  param_ack:'Parameter Transaction',param_changed:'Parameter Change'
});
const ACRONYMS=new Set(['FSM','IR','START','IMU','BLE']);
const KEY_TYPES=new Set(['fsm_transition','fsm_step','motor','ir_changed',
  'line_changed','start_changed','start_ack','error','param_ack','param_changed']);
const EVENT_FILTERS=Object.freeze([
  {value:'key',label:'Key events',types:KEY_TYPES},
  {value:'',label:'All events',types:null},
  {value:'fsm',label:'FSM transitions',types:new Set(['fsm_transition','fsm_step'])},
  {value:'motor',label:'Motor commands',types:new Set(['motor'])},
  {value:'sensor_changes',label:'Sensor changes',types:new Set(['ir_changed','line_changed'])},
  {value:'sensors',label:'Sensor snapshots',types:new Set(['sensors'])},
  {value:'start',label:'START',types:new Set(['start_changed','start_ack'])},
  {value:'errors',label:'Errors',types:new Set(['error'])},
  {value:'performance',label:'Performance',types:new Set(['task_timing','stats'])},
  {value:'connection',label:'Connection',types:new Set(['hello','heartbeat'])}
]);
function humanize(identifier) {
  return String(identifier??'').replace(/[_\s]+/g,' ').trim().split(' ').map(word=>{
    const upper=word.toUpperCase();
    return upper==='WIFI'?'WiFi':ACRONYMS.has(upper)||/^IR\d+$/.test(upper)?upper:
      word.charAt(0).toUpperCase()+word.slice(1).toLowerCase();
  }).join(' ');
}
function formatTimestampMs(timestampUs){
  return timestampUs!==null&&timestampUs!==undefined&&Number.isFinite(Number(timestampUs))?
    (Number(timestampUs)/1000).toFixed(3)+' ms':'—';
}
function eventTimeMs(event){
  if(Number.isFinite(event?.timeMs))return event.timeMs;
  if(Number.isFinite(event?.us))return event.us/1000;
  return Number.isFinite(event?.t)?event.t:null; // `t` ya está en milisegundos.
}
function formatEventTimeMs(event){
  const value=eventTimeMs(event);return value===null?'—':value.toFixed(3)+' ms';
}
function formatDurationMs(valueUs){
  return valueUs!==null&&valueUs!==undefined&&Number.isFinite(Number(valueUs))?
    (Number(valueUs)/1000).toFixed(3)+' ms':'—';
}
function formatElapsedMs(valueMs){
  return Number.isFinite(Number(valueMs))?Number(valueMs).toFixed(3)+' ms':'—';
}
function getEventTypeLabel(type){return EVENT_TYPE_LABELS[type]||humanize(type);}
function signed(value){const number=Number(value)||0;return `${number>0?'+':number<0?'−':''}${Math.abs(number)}%`;}

class StateNameResolver {
  constructor(){this.editorMachine=null;this.editor=new Map();this.definitions=new Map();this.numericIds=[];}
  setEditorCatalog(catalog){
    this.editorMachine=catalog?.machine||null;this.editor.clear();
    for(const state of catalog?.states||[]){
      if(!state?.id)continue;
      this.editor.set(String(state.id),{name:state.name||state.id,steps:Array.isArray(state.steps)?state.steps:[]});
    }
  }
  setDefinitionsSource(source){
    this.definitions.clear();
    const enumBody=String(source).match(/enum\s+class\s+StateId\s*:[^{]+\{([^}]+)\}/)?.[1]||'';
    this.numericIds=enumBody.split(',').map(item=>item.split('=')[0].trim()).filter(id=>id&&id!=='COUNT');
    const pattern=/\{\s*StateId::([A-Za-z_][\w]*)\s*,\s*"([^"\r\n]*)"/g;
    for(const match of String(source).matchAll(pattern))this.definitions.set(match[1],match[2]);
  }
  idKey(stateId){
    if(Number.isInteger(stateId))return this.numericIds[stateId]||String(stateId);
    return String(stateId??'');
  }
  getStateDisplayName(stateId,machine,telemetryName){
    const key=this.idKey(stateId);
    const editor=this.editorMachine===machine?this.editor.get(key):null;
    const candidate=editor?.name||this.definitions.get(key)||telemetryName||key;
    return humanize(candidate)||'—';
  }
  getStepDisplayName(stateId,step,machine){
    if(step===null||step===undefined)return '—';
    if(step<0)return 'End';
    const entry=this.editorMachine===machine?this.editor.get(this.idKey(stateId)):null;
    const candidate=entry?.steps?.[Number(step)];
    return candidate?humanize(typeof candidate==='string'?candidate:candidate.name||candidate.state):`Step ${step}`;
  }
}
function getEventStateSource(event,resolver){
  const machine=event.machine||event.displayMachine;
  if(event.type==='fsm_transition')return resolver.getStateDisplayName(event.from,machine);
  if(event.type==='fsm_step'||event.type==='fsm_status')return resolver.getStateDisplayName(event.state,machine);
  if(event.type==='motor')return resolver.getStateDisplayName(event.source,machine);
  if(event.type==='ir_changed')return `IR${event.index}`;
  if(event.type==='line_changed')return event.index?'Line Right':'Line Left';
  if(event.type==='start_changed')return 'START';
  if(event.type==='start_ack')return 'START';
  if(event.type==='task_timing')return humanize(event.task);
  if(event.type?.startsWith('param_'))return 'Parameters';
  return event.machine?humanize(event.machine):'—';
}
function formatEventDetails(event,resolver=new StateNameResolver(),tableContext=false){
  const machine=event.machine||event.displayMachine;
  const state=value=>resolver.getStateDisplayName(value,machine);
  const condition=event.condition?humanize(event.condition):null;
  switch(event.type){
    // La tabla ya tiene una columna de origen; el terminal conserva el texto completo.
    case 'fsm_transition':return `${tableContext?'':state(event.from)+' '}→ ${state(event.to)}${condition?' · '+condition:''}`;
    case 'fsm_step':return `${state(event.state)} / ${resolver.getStepDisplayName(event.state,event.step,machine)} → ${resolver.getStepDisplayName(event.state,event.nextStep,machine)}${condition?' · '+condition:''}`;
    case 'fsm_status':return `${tableContext?'':state(event.state)+' · '}${event.step<0?'No SubFSM step':resolver.getStepDisplayName(event.state,event.step,machine)} · ${event.running?'Running':humanize(event.reason||'Stopped')}`;
    case 'motor':return `L ${signed(event.left)} · R ${signed(event.right)}${event.source&&!tableContext?' · '+state(event.source):''}`;
    case 'sensors':return `IR ${(event.ir||[]).map(value=>Number(!!value)).join('')} · Line L${Number(!!event.line?.[0])} R${Number(!!event.line?.[1])} · START ${event.start?'On':'Off'}`;
    case 'ir_changed':return `${tableContext?'':`IR${event.index} → `}${event.detected?'Detected':'Clear'}`;
    case 'line_changed':return `${tableContext?'':`${event.index?'Right':'Left'} → `}${event.detected?'Detected':'Clear'}${event.raw===undefined?'':` · ADC ${event.raw}`}`;
    case 'start_changed':return `${tableContext?'':'START '}${event.start?'active':'stopped'}`;
    case 'start_ack':return `Command #${event.transaction} ${event.status} · START ${event.active?'on':'off'}`;
    case 'task_timing':return `${humanize(event.task)} execution ${event.executionUs} µs · start gap ${event.gapUs} µs`;
    case 'hello':return `${humanize(event.machine)} · protocol v${event.protocol}${event.schema===undefined?'':` · schema v${event.schema}`}`;
    case 'heartbeat':return `Uptime ${formatElapsedMs(event.uptimeMs)}`;
    case 'stats':return `Capture drops ${event.dropped} · Serial ${event.serialDropped??0} · BLE ${event.bleDropped??0} · WiFi ${event.wifiDropped??0}`;
    case 'error':case 'log':return event.message||'—';
    case 'imu':return event.valid?`Yaw ${event.yaw}°`:'IMU invalid';
    case 'param_schema':return event.id?`${event.name} · ${event.id} · ${event.min}–${event.max} ${event.unit||''}`:`${event.count} parameters`;
    case 'param_values':return event.id?`${event.id} = ${event.value}`:`${event.count} values`;
    case 'param_ack':return `Transaction #${event.transaction} ${event.status} · revision ${event.revision}${event.error?' · '+event.error:''}`;
    case 'param_changed':return `${event.id}: ${event.old} → ${event.value} · revision ${event.revision}`;
    default:return humanize(event.type);
  }
}
function matchesEventFilter(event,value){
  const filter=EVENT_FILTERS.find(item=>item.value===value);
  return !filter||!filter.types||filter.types.has(event.type);
}
function tickContext(event,tickPeriodMs){
  if(Number.isInteger(event.tickCount))return {tick:event.tickCount,phaseMs:null};
  const ms=eventTimeMs(event);
  if(!Number.isFinite(tickPeriodMs)||tickPeriodMs<=0||ms===null)return null;
  const tick=Math.floor(ms/tickPeriodMs);
  return {tick,phaseMs:ms-tick*tickPeriodMs};
}
function findResponseChain(events,maxGapMs=50){
  const transitionIndex=events.findLastIndex(event=>event.type==='fsm_transition');
  if(transitionIndex<0)return null;
  const transition=events[transitionIndex];
  const sensor=[...events.slice(0,transitionIndex)].reverse().find(event=>
    ['ir_changed','line_changed'].includes(event.type) &&
    eventTimeMs(transition)-eventTimeMs(event)<=maxGapMs);
  const motor=events.slice(transitionIndex+1).find(event=>
    event.type==='motor'&&eventTimeMs(event)-eventTimeMs(transition)<=maxGapMs);
  return {sensor,transition,motor,
    sensorToFsmMs:sensor?eventTimeMs(transition)-eventTimeMs(sensor):null,
    fsmToMotorMs:motor?eventTimeMs(motor)-eventTimeMs(transition):null,
    totalMs:sensor&&motor?eventTimeMs(motor)-eventTimeMs(sensor):null};
}
const api={EVENT_TYPE_LABELS,EVENT_FILTERS,KEY_TYPES,StateNameResolver,humanize,
  eventTimeMs,formatTimestampMs,formatEventTimeMs,formatDurationMs,formatElapsedMs,
  getEventTypeLabel,getEventStateSource,formatEventDetails,matchesEventFilter,tickContext,
  findResponseChain};
if(typeof module==='object'&&module.exports)module.exports=api;
else root.MbaretechTelemetryPresentation=api;
})(typeof window!=='undefined'?window:globalThis);
