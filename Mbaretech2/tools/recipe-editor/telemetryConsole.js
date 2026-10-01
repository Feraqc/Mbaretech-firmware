/* Controlador de la ventana: transporte → protocolo → store → paneles/bridge. */
(() => {
'use strict';
const $=id=>document.getElementById(id);
const escapeHtml=value=>String(value??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const P=MbaretechTelemetryPanels;
const F=MbaretechTelemetryPresentation;
const resolver=new F.StateNameResolver();
let requestedParameterSchema=false;
let lastHelloIdentity=null;
let startCommandPending=null;
const bridge=TelemetryWindowBridge.createBridge(message=>{
  if(message?.type==='editor_catalog'){
    resolver.setEditorCatalog(message);
    if(session.transport instanceof MbaretechMockTelemetry.MockTelemetryTransport){
      requestedParameterSchema=false;
      session.transport.configureCatalog(message);
    }
    scheduleRender(true);
  }else if(message?.type==='param_command'){
    relayParameterCommand(message);
  }else if(message?.type==='start_command'){
    relayStartCommand(message);
  }
});
const parameterCommands=new Set(['param_schema_request','param_values_request','param_set','param_reset']);
async function relayParameterCommand(message){
  const command=message.command;
  if(!session.state.connected||message.machine!==session.state.helloMachine||
     !command||!parameterCommands.has(command.type)){
    bridge.publish({type:'param_transport_result',requestId:message.requestId,
      ok:false,error:'Conexión o máquina incompatible.'});
    return;
  }
  try{
    await session.transport.send(command);
    bridge.publish({type:'param_transport_result',requestId:message.requestId,ok:true});
  }catch(error){
    bridge.publish({type:'param_transport_result',requestId:message.requestId,
      ok:false,error:String(error.message||error)});
  }
}
async function relayStartCommand(message){
  const command=message.command;
  if(!session.state.connected||message.machine!==session.state.helloMachine||
     command?.type!=='start_set'||!Number.isInteger(command.transaction)||
     typeof command.active!=='boolean'){
    bridge.publish({type:'start_transport_result',requestId:message.requestId,
      ok:false,error:'Conexión, máquina o comando START incompatible.'});
    return;
  }
  try{
    await session.transport.send(command);
    bridge.publish({type:'start_transport_result',requestId:message.requestId,ok:true});
  }catch(error){
    bridge.publish({type:'start_transport_result',requestId:message.requestId,
      ok:false,error:String(error.message||error)});
  }
}
const names={wifi:'WiFi',serial:'Serial',bluetooth:'Bluetooth',mock:'Mock'};
const legendConfig={
  motorOverview:[['motorL','Left motor','#6fb8e4'],['motorR','Right motor','#e4a16c']],
  motorPlot:[['motorL','Left motor','#6fb8e4'],['motorR','Right motor','#e4a16c']],
  linePlot:[['adcL','Left raw','#6fb8e4'],['adcR','Right raw','#e4a16c'],
    ['thresholdL','Left threshold','#8fa3b3'],['thresholdR','Right threshold','#d9a865']],
  timingPlot:[['fsmExec','FSM execution','#a997d8'],['sensorExec','Sensor execution','#78c997'],
    ['fsmGap','FSM start gap','#d9a865'],['sensorGap','Sensor start gap','#69b9d8']],
  snapshotPlot:[['snapshotAge','Snapshot age','#d9a865']],
  overviewSignalPlot:[...Array.from({length:7},(_,i)=>[`ir${i+1}`,`IR${i+1}`,'#79c8df']),
    ['lineL','Line Left','#d9a865'],['lineR','Line Right','#d9a865'],
    ['start','START','#78c997'],['state','FSM State','#a997d8'],['step','SubFSM Step','#d39ac0']],
  signalPlot:[...Array.from({length:7},(_,i)=>[`ir${i+1}`,`IR${i+1}`,'#79c8df']),
    ['lineL','Line Left','#d9a865'],['lineR','Line Right','#d9a865'],
    ['start','START','#78c997'],['state','FSM State','#a997d8'],['step','SubFSM Step','#d39ac0']],
  statePlot:[['state','FSM State','#a997d8'],['step','SubFSM Step','#d39ac0'],
    ['run','Lifecycle','#78c997'],['start','START','#d9a865']]
};
const visibilityKey='mbaretech-telemetry-curves-v1';
let seriesVisibility={};
try{seriesVisibility=JSON.parse(localStorage.getItem(visibilityKey)||'{}')||{};}catch{}
if(seriesVisibility.statePlot?.start===undefined){seriesVisibility.statePlot??={};seriesVisibility.statePlot.start=false;}
function visibleSeries(plotId){
  return new Set((legendConfig[plotId]||[]).filter(([key])=>seriesVisibility[plotId]?.[key]!==false).map(([key])=>key));
}
function saveVisibility(){try{localStorage.setItem(visibilityKey,JSON.stringify(seriesVisibility));}catch{}}
function renderLegend(plotId){
  const element=document.querySelector(`[data-legend="${plotId}"]`);if(!element)return;
  element.innerHTML=(legendConfig[plotId]||[]).map(([key,label,color])=>
    `<button data-series="${key}" class="${seriesVisibility[plotId]?.[key]===false?'off':''}" style="border-left:3px solid ${color}" title="Show or hide ${label}">${label}</button>`).join('')+
    '<span class="legend-actions"><button data-action="all">All</button><button data-action="none">None</button><button data-action="reset">Reset</button></span>';
}
function setupLegends(){
  for(const plotId of Object.keys(legendConfig)){
    const element=document.querySelector(`[data-legend="${plotId}"]`);if(!element)continue;
    element.onclick=event=>{
      const key=event.target.dataset.series,action=event.target.dataset.action;
      if(!key&&!action)return;
      seriesVisibility[plotId]??={};
      if(key)seriesVisibility[plotId][key]=seriesVisibility[plotId][key]===false;
      if(action==='reset')seriesVisibility[plotId]=plotId==='statePlot'?{start:false}:{};
      if(action==='all'||action==='none')for(const [name] of legendConfig[plotId])
        seriesVisibility[plotId][name]=action==='all';
      saveVisibility();renderLegend(plotId);render();
    };
    renderLegend(plotId);
  }
}
const makeTransport=kind=>kind==='serial'?new RecipeTelemetry.SerialTelemetryTransport():
  kind==='bluetooth'?new RecipeTelemetry.BluetoothTelemetryTransport():
  kind==='mock'?new MbaretechMockTelemetry.MockTelemetryTransport():
  new RecipeTelemetry.WebSocketTelemetryTransport();
let session=new RecipeTelemetry.TelemetrySession(makeTransport('wifi'),changed);
const store=new MbaretechTelemetryStore.TelemetryStore(session.state);
const chartHits=new Map();
let selectedView='overview',windowMs=10000,inspectionUs=null;
let selectedEventUs=null,tickPeriodMs=null;
let uiPaused=false,terminalPaused=false,recording=false,replaying=false;
let capture=new MbaretechTelemetryStore.RingBuffer(250000);
let renderPending=false,lastListRender=0,lastConnected=false;

// Los tres transportes reales y Mock pasan por el mismo decoder y la misma tienda.
const receive=session.receive.bind(session);
session.receive=raw=>{
  let message;
  try{message=RecipeTelemetry.decodeTelemetryMessage(raw);}catch{receive(raw);scheduleRender();return;}
  const before={state:session.state.currentState,step:session.state.currentStep,
    running:session.state.running,reason:session.state.stoppedReason};
  receive(raw);
  if(message.type==='hello'){
    const previous=lastHelloIdentity;
    const restarted=previous&&(
      message.machine!==previous.machine||
      (Number.isInteger(message.bootId)&&Number.isInteger(previous.bootId)&&
        message.bootId!==previous.bootId)||
      (!Number.isInteger(message.bootId)&&Number.isFinite(message.us)&&
        Number.isFinite(previous.us)&&message.us<previous.us)||
      (Number.isInteger(message.paramRevision)&&Number.isInteger(previous.revision)&&
        message.paramRevision<previous.revision));
    if(restarted)requestedParameterSchema=false;
    lastHelloIdentity={machine:message.machine,bootId:message.bootId,
      us:message.us,revision:message.paramRevision};
  }
  if(message.type==='start_ack'&&startCommandPending?.transaction===message.transaction){
    clearTimeout(startCommandPending.timeout);
    startCommandPending=null;
  }
  if(message.type==='start_ack'||message.type==='start_changed'||message.type==='sensors')
    updateStartButton();
  if(message.type==='hello'&&message.paramSchema===1&&!requestedParameterSchema){
    requestedParameterSchema=true;
    // Solicitar metadatos una vez por conexión; también sirve a editores abiertos después.
    Promise.resolve().then(()=>session.transport.send({type:'param_schema_request'}))
      .then(()=>session.transport.send({type:'param_values_request'})).catch(()=>{});
  }
  if(message.type==='hello'&&Number.isFinite(message.tickPeriodMs)&&message.tickPeriodMs>0)
    tickPeriodMs=message.tickPeriodMs;
  store.ingest(message,raw);
  if(recording&&!replaying)capture.push(message);
  const graph=TelemetryWindowBridge.graphMessage(message,before.state,before.step,before.running,before.reason);
  if(graph)bridge.publish(graph);
  scheduleRender();
};

function publishStatus(){
  const s=store.state;
  bridge.publish({type:'console_status',connected:s.connected,transport:s.transport,
    machine:s.helloMachine,protocol:s.protocol,bootId:s.bootId,
    paramRevision:s.parameters.revision,currentState:s.currentState,
    currentStep:s.currentStep,running:s.running,reason:s.stoppedReason,
    lastTransition:s.lastTransition,start:s.sensors.start,
    ir:s.sensors.ir,line:s.sensors.line,at:Date.now()});
}
function changed(reason){
  const connected=session.state.connected;
  if(connected&&!lastConnected){
    if(store.connectAt)store.reconnects++;
    store.connectAt=Date.now();
  }
  lastConnected=connected;
  if(!connected&&startCommandPending){clearTimeout(startCommandPending.timeout);startCommandPending=null;}
  updateStartButton();
  if(!connected){requestedParameterSchema=false;lastHelloIdentity=null;}
  if(reason==='connection'||reason==='hello')publishStatus();
  if(reason==='hello')bridge.publish({type:'catalog_request',machine:session.state.machine});
  scheduleRender(true);
}
function updateStartButton(){
  const button=$('startToggle');
  button.disabled=!session.state.connected||Boolean(startCommandPending);
  const active=Boolean(session.state.sensors.start);
  button.textContent=`START: ${active?'ON':'OFF'}${startCommandPending?' …':''}`;
  button.setAttribute('aria-pressed',String(Boolean(active)));
}
setInterval(()=>{publishStatus();if(!uiPaused)renderHeader();},1000);
setInterval(()=>bridge.publish({type:'catalog_request',machine:store.state.machine}),5000);
window.addEventListener('beforeunload',()=>{
  bridge.publish({type:'console_status',connected:false,transport:store.state.transport});
  session.disconnect();bridge.close();
});

function scheduleRender(force=false){
  if(uiPaused&&!force)return;
  if(renderPending)return;
  renderPending=true;
  setTimeout(()=>requestAnimationFrame(()=>{renderPending=false;render();}),50);
}
function text(id,value){const element=$(id);if(element)element.textContent=String(value);}
function fact(label,value){return `<div class="fact"><b>${escapeHtml(label)}</b><span>${escapeHtml(value)}</span></div>`;}
function renderHeader(){
  const s=store.state,age=s.lastPacketAt?Date.now()-s.lastPacketAt:null;
  text('connection',s.connected?`● ${s.transport.toUpperCase()} LIVE`:`○ ${s.status.toUpperCase()}`);
  $('connection').className='badge '+(s.connected?'live':s.status==='error'?'error':'');
  text('machine',`Machine ${s.machine||'—'}`);
  text('protocol',`Protocol ${s.protocol?'v'+s.protocol:'—'} / ${s.schema?'schema v'+s.schema:'—'}`);
  text('uptime',`Uptime ${F.formatElapsedMs(s.uptimeMs)}`);
  text('rate',`${s.stats.rate} ev/s · ${store.byteRate()} B/s`);
  text('gaps',`${s.stats.sequenceGaps} gaps`);
  text('drops',`${s.stats.dropped} device drops · ${store.events.overwrites} browser`);
  text('packetAge',`Last packet ${age===null?'—':age+' ms'}`);
  text('paramRevision',`Params r${s.parameters.revision}`);
}
function render(){
  renderHeader();
  const s=store.state,endUs=inspectionUs??store.latestUs;
  const events=store.events.valuesSince(Math.max(0,endUs-windowMs*1000),event=>event.us<=endUs);
  if(selectedView==='overview')renderOverview(s,events,endUs);
  else if(selectedView==='signals')renderSignals(events,endUs);
  else if(selectedView==='fsm')renderFsm(s);
  else if(selectedView==='performance')renderPerformance(s,events,endUs);
  else if(selectedView==='events')renderEvents();
  else if(selectedView==='terminal')renderTerminal();
  else if(selectedView==='capture')renderCapture();
}
function stat(label,value,note='',tone=''){
  return `<div class="stat"><div class="stat-label">${escapeHtml(label)}</div><div class="stat-value ${tone}">${escapeHtml(value)}</div><div class="stat-note">${escapeHtml(note)}</div></div>`;
}
function renderOverview(s,events,endUs){
  const snapshotAge=s.sampledAtMs?Math.max(0,s.uptimeMs-s.sampledAtMs):null;
  const stateName=resolver.getStateDisplayName(s.currentState,s.machine);
  const stepName=resolver.getStepDisplayName(s.currentState,s.currentStep,s.machine);
  const motorSource=resolver.getStateDisplayName(s.motorSource,s.machine);
  $('stats').innerHTML=stat('START',s.sensors.start?'ACTIVE':'STOPPED',s.running?'FSM RUNNING':s.stoppedReason||'stopped',s.sensors.start?'good':'warn')+
    stat('FSM STATE',stateName,s.machine||'no machine')+
    stat('SUBFSM STEP',stepName,'current step')+
    stat('MOTOR LEFT',`${s.motors.left}%`,motorSource)+
    stat('MOTOR RIGHT',`${s.motors.right}%`,motorSource)+
    stat('SNAPSHOT AGE',snapshotAge===null?'—':F.formatElapsedMs(snapshotAge),`max ${F.formatElapsedMs(store.maxSnapshotAgeMs)}`,snapshotAge>10?'warn':'')+
    stat('RX RATE',`${s.stats.rate}/s`,`${store.byteRate()} bytes/s`)+
    stat('TELEMETRY DROPS',s.stats.dropped,`${store.events.overwrites} browser overwrites`,s.stats.dropped?'warn':'');
  const ir=s.sensors.ir;
  $('irMap').innerHTML=`<span></span><span class="${ir[2]?'on':''}">IR3</span><span class="${ir[3]?'on':''}">IR4</span><span class="${ir[4]?'on':''}">IR5</span><span></span>
    <span class="${ir[1]?'on':''}">IR2</span><span class="center">FRONT</span><span class="${ir[5]?'on':''}">IR6</span>
    <span class="${ir[0]?'on':''}">IR1</span><span class="center">ROBOT</span><span class="${ir[6]?'on':''}">IR7</span>`;
  $('robotCenter').innerHTML=`ROBOT<br>START ${s.sensors.start?'●':'○'}<br>L ${s.motors.left>0?'+':''}${s.motors.left}%　R ${s.motors.right>0?'+':''}${s.motors.right}%`;
  $('lineIndicators').innerHTML=`<span class="${s.sensors.line[0]?'on':''}">LINE L ${s.sensors.line[0]?'●':'○'}</span><span class="${s.sensors.line[1]?'on':''}">LINE R ${s.sensors.line[1]?'●':'○'}</span>`;
  $('motorCommands').innerHTML=[0,1].map((i)=>{
    const value=i?s.motors.right:s.motors.left;
    return `<div class="motor-cell"><small>${i?'RIGHT':'LEFT'}</small><br><strong>${value>0?'+':''}${value}%</strong><br><span>${value<0?'← BACKWARD':value>0?'FORWARD →':'STOPPED'}</span><div class="motor-bar"><i style="left:${value<0?50+value/2:50}%;width:${Math.abs(value)/2}%"></i></div><small>${escapeHtml(motorSource)}</small></div>`;
  }).join('');
  chartHits.set('motorOverview',P.drawSeries($('motorOverview'),events,endUs,windowMs,motorSeries(),
    {min:-100,max:100,visible:visibleSeries('motorOverview')}));
  chartHits.set('overviewSignalPlot',P.drawLogic($('overviewSignalPlot'),events,endUs,windowMs,
    {visible:visibleSeries('overviewSignalPlot'),resolver}));
  if(Date.now()-lastListRender>180){
    $('recent').innerHTML=store.events.latest(16,e=>isHighValue(e)).map(compactEventRow).join('');
    lastListRender=Date.now();
  }
}
function lineSeries(){return [
  {key:'adcL',label:'LINE L ADC',color:'#6fb8e4',read:e=>e.type==='sensors'?e.lineRaw?.[0]:undefined},
  {key:'adcR',label:'LINE R ADC',color:'#e4a16c',read:e=>e.type==='sensors'?e.lineRaw?.[1]:undefined},
  {key:'thresholdL',label:'L THRESHOLD',color:'#8fa3b3',read:e=>e.type==='sensors'?e.lineThreshold:undefined},
  {key:'thresholdR',label:'R THRESHOLD',color:'#d9a865',read:e=>e.type==='sensors'?e.lineThreshold:undefined}];}
function motorSeries(){return [
  {key:'motorL',label:'MOTOR L',color:'#6fb8e4',read:e=>e.type==='motor'?e.left:undefined},
  {key:'motorR',label:'MOTOR R',color:'#e4a16c',read:e=>e.type==='motor'?e.right:undefined}];}
function renderSignals(events,endUs){
  const s=store.state;
  chartHits.set('signalPlot',P.drawLogic($('signalPlot'),events,endUs,windowMs,
    {visible:visibleSeries('signalPlot'),resolver}));
  chartHits.set('linePlot',P.drawSeries($('linePlot'),events,endUs,windowMs,lineSeries(),
    {min:0,max:4095,visible:visibleSeries('linePlot')}));
  chartHits.set('motorPlot',P.drawSeries($('motorPlot'),events,endUs,windowMs,motorSeries(),
    {min:-100,max:100,visible:visibleSeries('motorPlot')}));
  chartHits.set('statePlot',P.drawStateTimeline($('statePlot'),events,endUs,windowMs,
    {visible:visibleSeries('statePlot'),resolver}));
  $('lineCalibration').innerHTML=[0,1].map(index=>{
    const raw=s.sensors.rawLine[index],threshold=s.lineThreshold??0;
    const width=Math.max(0,Math.min(100,raw/4095*100));
    const thresholdPosition=Math.max(0,Math.min(100,threshold/4095*100));
    return `<div class="line-side"><h3>${index?'RIGHT':'LEFT'} LINE</h3><div>Raw ADC <strong>${escapeHtml(raw)}</strong></div><div>Threshold ${escapeHtml(threshold||'—')}</div><div class="adc-bar" style="--threshold:${thresholdPosition}%"><i style="width:${width}%"></i></div><div>Filtered ${s.sensors.line[index]?'DETECTED':'CLEAR'}</div><div>Filter progress unavailable in current protocol</div></div>`;
  }).join('');
}
function renderFsm(s){
  const last=s.lastTransition;
  const chain=F.findResponseChain(store.events.latest(300).reverse());
  const latency=value=>value===null||value===undefined?'—':`${value.toFixed(3)} ms`;
  $('fsmDetails').innerHTML=fact('Machine',s.machine)+
    fact('Current state',resolver.getStateDisplayName(s.currentState,s.machine))+
    fact('SubFSM step',resolver.getStepDisplayName(s.currentState,s.currentStep,s.machine))+
    fact('State elapsed',F.formatElapsedMs(s.stateElapsedMs))+
    fact('Step elapsed',F.formatElapsedMs(s.stepElapsedMs))+
    fact('Lifecycle',s.running?'RUNNING':s.stoppedReason||'STOPPED')+
    fact('Previous state',resolver.getStateDisplayName(last?.from??last?.state,s.machine))+
    fact('Next state / step',last?.to?resolver.getStateDisplayName(last.to,s.machine):
      resolver.getStepDisplayName(last?.state,last?.nextStep,s.machine))+
    fact('Condition',F.humanize(last?.condition))+
    fact('Occurred',last?F.formatEventTimeMs(last):'—')+
    fact('Sensor → FSM',chain?latency(chain.sensorToFsmMs):'—')+
    fact('FSM → motor',chain?latency(chain.fsmToMotorMs):'—')+
    fact('Sensor → motor',chain?latency(chain.totalMs):'—')+
    fact('Timing method','Nearest edges within 50 ms; correlation only');
  $('fsmDetails').innerHTML+=fact('Parameter revision',s.parameters.revision)+
    fact('Last parameter change',s.parameters.lastChange?
      `${s.parameters.lastChange.id}: ${s.parameters.lastChange.old} → ${s.parameters.lastChange.value} · ${F.formatEventTimeMs(s.parameters.lastChange)}`:'—')+
    fact('Last transaction',s.parameters.lastAck?
      `#${s.parameters.lastAck.transaction} ${s.parameters.lastAck.status}`:'—');
  $('transitions').innerHTML=store.events.latest(100,e=>e.type==='fsm_transition'||e.type==='fsm_step').map(compactEventRow).join('');
  $('candidates').innerHTML=fact('Priority / candidate evaluation',
    'Firmware does not publish evaluated candidate order; exact fired conditions are shown above.')+
    fact('Last selected condition',F.humanize(s.lastCondition)||'—');
}
function renderPerformance(s,events,endUs){
  $('timing').innerHTML=Object.entries(s.timing).map(([name,value])=>{
    const max=store.tasks.get(name);
    return fact(`${name} current`,`${value.executionUs} µs`) + fact(`${name} max`,`${max?.maxExecutionUs??0} µs`)+
      fact(`${name} start gap`,`${value.gapUs} µs`)+fact(`${name} max gap`,`${max?.maxGapUs??0} µs`);
  }).join('')||fact('Task timing','Enable task timing in firmware');
  chartHits.set('timingPlot',P.drawSeries($('timingPlot'),events,endUs,windowMs,[
    {key:'fsmExec',label:'FSM EXEC µs',color:'#a997d8',read:e=>e.type==='task_timing'&&e.task==='fsm'?e.executionUs:undefined},
    {key:'sensorExec',label:'SENSOR EXEC µs',color:'#78c997',read:e=>e.type==='task_timing'&&e.task==='sensor'?e.executionUs:undefined},
    {key:'fsmGap',label:'FSM GAP µs',color:'#d9a865',read:e=>e.type==='task_timing'&&e.task==='fsm'?e.gapUs:undefined},
    {key:'sensorGap',label:'SENSOR GAP µs',color:'#69b9d8',read:e=>e.type==='task_timing'&&e.task==='sensor'?e.gapUs:undefined}],
    {min:0,max:1500,visible:visibleSeries('timingPlot')}));
  $('health').innerHTML=fact('Capture drops',s.stats.dropped)+fact('Serial drops',s.stats.serialDropped)+
    fact('BLE drops',s.stats.bleDropped)+fact('WiFi drops',s.stats.wifiDropped)+
    fact('Browser ring overwrites',store.events.overwrites)+fact('Sequence gaps',s.stats.sequenceGaps)+
    fact('Decoder errors',s.stats.errors)+fact('Snapshot age max',F.formatElapsedMs(store.maxSnapshotAgeMs));
  chartHits.set('snapshotPlot',P.drawSeries($('snapshotPlot'),events,endUs,windowMs,[
    {key:'snapshotAge',label:'SNAPSHOT AGE ms',color:'#d9a865',read:e=>
      e.type==='sensors'&&Number.isFinite(e.sampledAtMs)?Math.max(0,e.t-e.sampledAtMs):undefined}],
    {min:0,max:20,visible:visibleSeries('snapshotPlot')}));
  $('network').innerHTML=fact('Transport',s.transport)+fact('Connection',s.status)+fact('Endpoint',s.endpoint||'—')+
    fact('Connected for',store.connectAt&&s.connected?F.formatElapsedMs(Date.now()-store.connectAt):'—')+
    fact('RX messages/s',s.stats.rate)+fact('RX bytes/s',store.byteRate())+
    fact('Last packet',s.lastPacketAt?F.formatElapsedMs(Date.now()-s.lastPacketAt):'—')+
    fact('Reconnections',store.reconnects)+fact('WiFi RSSI',s.network.wifiRssi===null?'—':`${s.network.wifiRssi} dBm`)+
    fact('RTT','Not published by device')+fact('Queue utilization','Not published by device')+
    fact('Overrun count','Not published by device');
}
function isHighValue(event){return F.KEY_TYPES.has(event.type);}
function rowClass(event){return isHighValue(event)?'actual':'sample';}
function compactEventRow(event){
  return `<div class="event-row ${rowClass(event)}" data-us="${Number(event.us)||0}"><span class="time">${F.formatEventTimeMs(event)}</span><span class="kind">${escapeHtml(F.getEventTypeLabel(event.type))}</span><span>${escapeHtml(F.formatEventDetails(event,resolver))}</span></div>`;
}
function eventRow(event,delta){
  const tick=F.tickContext(event,tickPeriodMs);
  const tickLabel=tick?String(tick.tick):'—';
  const title=`seq=${event.seq??'—'} · us=${event.us??'—'} · type=${event.type} · tick phase=${tick?.phaseMs===null||tick?.phaseMs===undefined?'—':tick.phaseMs.toFixed(3)+' ms'}`;
  return `<div class="event-row ${rowClass(event)} ${event.us===selectedEventUs?'selected':''}" data-us="${Number(event.us)||0}" title="${escapeHtml(title)}"><span class="time">${F.formatEventTimeMs(event)}</span><span>${escapeHtml(delta)}</span><span>${escapeHtml(tickLabel)}</span><span class="kind">${escapeHtml(F.getEventTypeLabel(event.type))}</span><span>${escapeHtml(F.getEventStateSource(event,resolver))}</span><span>${escapeHtml(F.formatEventDetails(event,resolver,true))}</span></div>`;
}
function renderEvents(){
  if(Date.now()-lastListRender<150)return;lastListRender=Date.now();
  const type=$('eventType').value,state=$('eventState').value.trim().toUpperCase();
  const sensor=$('eventSensor').value.trim().toUpperCase(),query=$('eventSearch').value.trim().toUpperCase();
  const matches=event=>{
    if(!F.matchesEventFilter(event,type))return false;
    if(state&&![event.state,event.from,event.to,event.source,
      F.getEventStateSource(event,resolver)].some(value=>String(value||'').toUpperCase().includes(state)))return false;
    const description=F.formatEventDetails(event,resolver).toUpperCase();
    if(sensor&&!description.includes(sensor))return false;
    return !query||description.includes(query)||F.getEventTypeLabel(event.type).toUpperCase().includes(query)||
      String(event.us).includes(query)||F.formatEventTimeMs(event).toUpperCase().includes(query);
  };
  const chronological=store.events.latest(400,matches).reverse();
  const mode=$('deltaMode').value;
  let rows=chronological.map((event,index)=>{
    const reference=mode==='selected'?selectedEventUs:
      index?chronological[index-1].timeUs:null;
    const delta=reference===null?'—':`${(F.eventTimeMs(event)-reference/1000)>=0?'+':'−'}${Math.abs(F.eventTimeMs(event)-reference/1000).toFixed(3)} ms`;
    return {event,delta};
  });
  if($('eventOrder').value==='newest')rows=rows.reverse();
  $('eventRows').innerHTML=rows.map(({event,delta})=>eventRow(event,delta)).join('');
}
function renderTerminal(){
  if(terminalPaused||Date.now()-lastListRender<150)return;lastListRender=Date.now();
  const type=$('terminalType').value,raw=$('terminalMode').value==='raw';
  $('terminalHeader').innerHTML=raw?'<span>RAW CANONICAL PROTOCOL</span>':
    '<span>TIME</span><span>TYPE</span><span>DETAILS</span>';
  $('terminalHeader').classList.toggle('raw-header',raw);
  const lines=store.terminal.latest(400,item=>!type||item.event.type===type).reverse().map(item=>{
    const event=item.event;
    return raw?`<div class="terminal-raw">${escapeHtml(item.raw)}</div>`:
      `<div class="terminal-row ${rowClass(event)}"><span class="time">${F.formatEventTimeMs(event)}</span><span class="kind">${escapeHtml(F.getEventTypeLabel(event.type))}</span><span>${escapeHtml(F.formatEventDetails(event,resolver))}</span></div>`;
  });
  $('terminalLines').innerHTML=lines.join('');
  if($('autoScroll').checked)$('terminalLines').parentElement.scrollTop=$('terminalLines').parentElement.scrollHeight;
}
function renderCapture(){
  $('captureState').innerHTML=fact('Recording',recording?'ACTIVE':'STOPPED')+
    fact('Captured events',capture.length)+fact('Capture overwrites',capture.overwrites)+
    fact('Rolling history',`${store.events.length} / ${store.events.capacity}`);
}
function clearDisplay(){
  store.clear();session.state.history.length=0;session.state.timeline.length=0;
  session.state.console.length=0;inspectionUs=null;selectedEventUs=null;
  text('selectedMarker','Select a row as marker A');render();
}
function connect(){
  try{
    const kind=$('transport').value;
    if(session.state.transport!==names[kind])session.setTransport(makeTransport(kind),names[kind]);
    const endpoint=$('endpoint').value.trim();
    if(kind==='wifi')try{localStorage.setItem('mbaretech-telemetry-endpoint',endpoint);}catch{}
    Promise.resolve(session.connect(kind==='wifi'?endpoint:undefined)).catch(error=>text('warning',error.message));
    text('warning','');
  }catch(error){text('warning',error.message);}
  render();
}

// Paneles reordenables; tamaño, colapso y orden sobreviven al refresh.
const layoutKey='mbaretech-telemetry-layout-v2';
const defaultLayout={};
document.querySelectorAll('.view.dashboard-grid').forEach(view=>{
  defaultLayout[view.id]=[...view.querySelectorAll('[data-panel]')].map(panel=>({
    id:panel.dataset.panel,span:panel.dataset.span||''}));
});
function saveLayout(){
  const layout={};
  document.querySelectorAll('.view.dashboard-grid').forEach(view=>{
    layout[view.id]=[...view.querySelectorAll('[data-panel]')].map(panel=>({
      id:panel.dataset.panel,span:panel.dataset.span||'',collapsed:panel.classList.contains('collapsed')}));
  });
  try{localStorage.setItem(layoutKey,JSON.stringify(layout));}catch{}
}
function restoreLayout(){
  let layout;try{layout=JSON.parse(localStorage.getItem(layoutKey)||'{}');}catch{layout={};}
  for(const [viewId,panels] of Object.entries(layout)){
    const view=$(viewId);if(!view||!Array.isArray(panels))continue;
    for(const item of panels){
      const panel=[...view.querySelectorAll('[data-panel]')].find(candidate=>candidate.dataset.panel===item.id);
      if(!panel)continue;
      panel.dataset.span=item.span||'';panel.classList.toggle('collapsed',!!item.collapsed);view.appendChild(panel);
    }
  }
}
function setupPanels(){
  restoreLayout();
  for(const panel of document.querySelectorAll('[data-panel]')){
    const title=panel.querySelector('.panel-title');
    title.draggable=true;
    const actions=document.createElement('div');actions.className='panel-actions';
    actions.innerHTML='<button title="Resize panel" data-action="size">◫</button><button title="Collapse panel" data-action="collapse">−</button><button title="Maximize panel" data-action="maximize">□</button>';
    title.appendChild(actions);
    actions.onclick=event=>{
      const action=event.target.dataset.action;if(!action)return;event.stopPropagation();
      if(action==='size')panel.dataset.span=panel.dataset.span==='wide'?'':'wide';
      if(action==='collapse')panel.classList.toggle('collapsed');
      if(action==='maximize')panel.classList.toggle('maximized');
      saveLayout();render();
    };
    title.ondragstart=event=>{event.dataTransfer.setData('text/plain',panel.dataset.panel);panel.classList.add('dragging');};
    title.ondragend=()=>panel.classList.remove('dragging');
    panel.ondragover=event=>{event.preventDefault();panel.classList.add('drag-over');};
    panel.ondragleave=()=>panel.classList.remove('drag-over');
    panel.ondrop=event=>{
      event.preventDefault();panel.classList.remove('drag-over');
      const id=event.dataTransfer.getData('text/plain');
      const source=[...panel.parentElement.querySelectorAll('[data-panel]')].find(item=>item.dataset.panel===id);
      if(source&&source!==panel){panel.before(source);saveLayout();render();}
    };
  }
}

function selectView(view){
  selectedView=view;lastListRender=0;
  document.querySelectorAll('[data-view]').forEach(button=>button.classList.toggle('active',button.dataset.view===view));
  document.querySelectorAll('.view').forEach(element=>element.classList.toggle('active',element.id===view));
  render();
}
document.querySelectorAll('[data-view]').forEach(button=>button.onclick=()=>selectView(button.dataset.view));
$('transport').onchange=()=>{$('endpointLabel').hidden=$('transport').value!=='wifi';updateStartButton();};
try{
  $('endpoint').value=localStorage.getItem('mbaretech-telemetry-endpoint')||$('endpoint').value;
  windowMs=Number(localStorage.getItem('mbaretech-telemetry-window'))||10000;
  $('window').value=String(windowMs);
}catch{}
$('window').onchange=()=>{windowMs=Number($('window').value);inspectionUs=null;
  try{localStorage.setItem('mbaretech-telemetry-window',String(windowMs));}catch{}render();};
$('connect').onclick=connect;
$('disconnect').onclick=()=>session.disconnect();
$('startToggle').onclick=async()=>{
  if(!session.state.connected||startCommandPending)return;
  const transaction=(Date.now()%2000000000)+1;
  const active=!Boolean(session.state.sensors.start);
  const timeout=setTimeout(()=>{
    if(startCommandPending?.transaction!==transaction)return;
    startCommandPending=null;updateStartButton();
    text('warning','No START ACK from device. Check connection and telemetry drops.');
  },3000);
  startCommandPending={transaction,timeout};updateStartButton();
  try{await session.transport.send({type:'start_set',transaction,active});}
  catch(error){
    if(startCommandPending?.transaction===transaction){clearTimeout(timeout);startCommandPending=null;}
    text('warning',error.message);updateStartButton();
  }
};
updateStartButton();
$('pauseUi').onclick=()=>{uiPaused=!uiPaused;text('pauseUi',uiPaused?'Resume UI':'Pause UI');
  if(!uiPaused){inspectionUs=null;text('warning','');render();}};
$('record').onclick=()=>{capture=new MbaretechTelemetryStore.RingBuffer(250000);recording=true;
  text('record','Recording ●');render();};
$('stop').onclick=()=>{recording=false;text('record','Record');render();};
$('clear').onclick=clearDisplay;
$('clearCapture').onclick=()=>{capture.clear();render();};
$('resetLayout').onclick=()=>{try{localStorage.removeItem(layoutKey);}catch{}
  for(const [viewId,panels] of Object.entries(defaultLayout)){
    const view=$(viewId);
    for(const item of panels){
      const panel=[...view.querySelectorAll('[data-panel]')].find(candidate=>candidate.dataset.panel===item.id);
      panel.dataset.span=item.span;panel.classList.remove('collapsed','maximized');view.appendChild(panel);
    }
  }
  render();};
$('settings').onclick=()=>{document.body.classList.toggle('comfortable');
  text('warning',document.body.classList.contains('comfortable')?'Comfortable density enabled. Click Settings again for compact.':'Compact density enabled.');
  try{localStorage.setItem('mbaretech-telemetry-density',document.body.classList.contains('comfortable')?'comfortable':'compact');}catch{}};
try{document.body.classList.toggle('comfortable',localStorage.getItem('mbaretech-telemetry-density')==='comfortable');}catch{}
$('eventType').innerHTML=F.EVENT_FILTERS.map(filter=>
  `<option value="${filter.value}">${filter.label}</option>`).join('');
$('terminalType').innerHTML='<option value="">All events</option>'+Object.keys(F.EVENT_TYPE_LABELS)
  .filter(type=>['hello','heartbeat','stats','sensors','ir_changed','line_changed','start_changed','start_ack','motor',
    'fsm_status','fsm_transition','fsm_step','task_timing','imu','error','log'].includes(type))
  .map(type=>`<option value="${type}">${F.getEventTypeLabel(type)}</option>`).join('');
for(const id of ['eventType','terminalType'])$(id).onchange=()=>{lastListRender=0;render();};
for(const id of ['eventState','eventSensor','eventSearch','eventOrder','terminalMode','deltaMode'])
  $(id).addEventListener(['eventOrder','terminalMode','deltaMode'].includes(id)?'change':'input',
    ()=>{lastListRender=0;render();});
$('terminalPause').onclick=()=>{terminalPaused=!terminalPaused;
  text('terminalPause',terminalPaused?'Resume terminal':'Pause terminal');if(!terminalPaused){lastListRender=0;render();}};
$('copyTerminal').onclick=()=>navigator.clipboard.writeText($('terminalLines').innerText)
  .catch(error=>text('warning',error.message));
$('clearTerminal').onclick=()=>{store.terminal.clear();session.state.console.length=0;lastListRender=0;render();};
$('export').onclick=()=>{
  const lines=[];for(let i=0;i<capture.length;i++)lines.push(JSON.stringify(capture.at(i)));
  const blob=new Blob([lines.join('\n')+'\n'],{type:'application/x-ndjson'});
  const url=URL.createObjectURL(blob),link=document.createElement('a');
  link.href=url;link.download='mbaretech-telemetry.ndjson';link.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
};
$('replay').onclick=async()=>{
  const file=$('replayFile').files[0];if(!file)return;
  if(store.state.connected){text('warning','Disconnect the robot before replay.');return;}
  replaying=true;clearDisplay();
  try{for(const line of (await file.text()).split(/\r?\n/))if(line.trim())session.receive(line);
    text('warning','Replay loaded locally. No commands sent.');}
  catch(error){text('warning',error.message);}
  finally{replaying=false;render();}
};
for(const id of ['overviewSignalPlot','signalPlot','linePlot','motorPlot','motorOverview','statePlot','timingPlot','snapshotPlot']){
  const canvas=$(id);if(!canvas)continue;
  canvas.onmousemove=event=>{
    const rect=canvas.getBoundingClientRect();
    const hits=chartHits.get(id)||[],x=event.clientX-rect.left,y=event.clientY-rect.top;
    const hit=P.nearest(hits,x,y,id==='statePlot');
    const values=id==='statePlot'?[hit].filter(Boolean):P.valuesAtX(hits,x);
    const detail=values.length?`${F.formatEventTimeMs(hit?.event||values[0].event)} · `+
      values.map(item=>`${item.label}: ${item.value}${item.durationUs===undefined?'':` (${F.formatDurationMs(item.durationUs)})`}`)
        .join(' · ')+(hit?.event.condition?` · ${F.humanize(hit.event.condition)}`:''):'';
    canvas.title=detail;
    if(id==='statePlot')text('stateCursor',detail||'Hover over a segment for duration and reason');
    if(id==='signalPlot')text('signalCursor',detail||'Hover for producer timestamp');
  };
  canvas.onwheel=event=>{event.preventDefault();
    const choices=[500,2000,5000,10000,30000],index=choices.indexOf(windowMs);
    windowMs=choices[Math.max(0,Math.min(choices.length-1,index+(event.deltaY>0?1:-1)))];
    $('window').value=String(windowMs);render();};
}
document.addEventListener('click',event=>{
  const row=event.target.closest('.event-row');if(!row)return;
  const us=Number(row.dataset.us);if(!us)return;
  selectedEventUs=us;inspectionUs=us+windowMs*500;uiPaused=true;
  text('pauseUi','Resume UI');
  text('selectedMarker',`A = ${F.formatTimestampMs(us)} · Δ selected available`);
  lastListRender=0;render();
  text('warning',`Plots centered on ${F.formatTimestampMs(us)}. Open Signals to inspect; Resume UI returns to live.`);
});
setupPanels();setupLegends();render();publishStatus();
bridge.publish({type:'catalog_request'});
fetch('include/fsm/FSMDefinitions.h').then(response=>response.ok?response.text():'')
  .then(source=>{if(source){resolver.setDefinitionsSource(source);scheduleRender(true);}})
  .catch(()=>{}); // La consola también funciona sin servidor de headers.
})();
