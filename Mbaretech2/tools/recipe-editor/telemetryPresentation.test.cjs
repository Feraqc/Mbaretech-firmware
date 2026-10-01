/* Formato en ms, resolución semántica y visibilidad de curvas. */
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const F=require('./telemetryPresentation.js');
const P=require('./telemetryPanels.js');
const {TelemetryStore}=require('./telemetryStore.js');

assert.equal(F.formatTimestampMs(25080853),'25080.853 ms');
assert.equal(F.formatDurationMs(151),'0.151 ms');
assert.equal(F.formatEventTimeMs({us:25080853,t:25080}),'25080.853 ms');
assert.equal(F.formatEventTimeMs({t:25080}),'25080.000 ms'); // No dividir `t` dos veces.
assert.equal(F.getEventTypeLabel('fsm_transition'),'FSM Transition');
assert.equal(F.getEventTypeLabel('custom_event'),'Custom Event');
assert.equal(F.matchesEventFilter({type:'fsm_status'},'key'),false);
assert.equal(F.matchesEventFilter({type:'fsm_transition'},'key'),true);

const resolver=new F.StateNameResolver();
const definitions=fs.readFileSync(path.resolve(__dirname,'../../include/fsm/FSMDefinitions.h'),'utf8');
resolver.setDefinitionsSource(definitions);
assert.equal(resolver.getStateDisplayName('MOTOR_SEQUENCE_1','state_test'),'Backward');
assert.equal(resolver.getStateDisplayName(7,'state_test'),'Backward');
resolver.setEditorCatalog({machine:'state_test',states:[
  {id:'MOTOR_SEQUENCE_1',name:'Reverse',steps:['Backward','Pause']}]});
assert.equal(resolver.getStateDisplayName('MOTOR_SEQUENCE_1','state_test'),'Reverse');
assert.equal(resolver.getStateDisplayName('MOTOR_SEQUENCE_1','other'),'Backward');
assert.equal(resolver.getStepDisplayName('MOTOR_SEQUENCE_1',0,'state_test'),'Backward');
assert.equal(resolver.getStepDisplayName('MOTOR_SEQUENCE_1',3,'state_test'),'Step 3');
const transition={type:'fsm_transition',us:25081004,t:25081,machine:'state_test',
  from:'MOTOR_SEQUENCE',to:'MOTOR_SEQUENCE_1',condition:'TIMER'};
assert.match(F.formatEventDetails(transition,resolver),/Reverse · Timer/);
assert.equal(F.formatEventDetails(transition,resolver,true),'→ Reverse · Timer');
assert.equal(F.formatEventDetails({type:'ir_changed',index:4,detected:true},resolver,true),'Detected');
assert.equal(F.formatEventDetails({type:'motor',left:100,right:-40,source:'MOTOR_SEQUENCE_1',machine:'state_test'},resolver,true),'L +100% · R −40%');
assert.equal(F.tickContext(transition,null),null);
assert.equal(F.tickContext(transition,1).tick,25081);
assert(Math.abs(F.tickContext(transition,1).phaseMs-0.004)<1e-6);
const chain=F.findResponseChain([
  {type:'ir_changed',us:25080853,t:25080,index:4,detected:true},
  transition,{type:'motor',us:25081173,t:25081,left:100,right:100}]);
assert(Math.abs(chain.sensorToFsmMs-0.151)<1e-6);
assert(Math.abs(chain.fsmToMotorMs-0.169)<1e-6);
assert(Math.abs(chain.totalMs-0.320)<1e-6);

const state={machine:'state_test'};
const store=new TelemetryStore(state,4);
const raw='{"type":"motor","seq":2,"t":25,"us":25081,"left":10,"right":20}';
const event=JSON.parse(raw);
store.ingest(event,raw);
assert.equal(event.timeUs,25081);
assert.equal(event.timeMs,25.081);
assert.equal(JSON.stringify(event),raw); // Alias no enumerables; captura Raw intacta.

function canvas(){
  const context={setTransform(){},clearRect(){},fillRect(){},fillText(){},
    beginPath(){},moveTo(){},lineTo(){},stroke(){},setLineDash(){}};
  return {clientWidth:800,clientHeight:200,width:0,height:0,getContext:()=>context};
}
const samples=[{type:'motor',us:1000,left:10,right:20},{type:'motor',us:2000,left:30,right:40}];
const series=[{key:'motorL',label:'Left',read:e=>e.left},{key:'motorR',label:'Right',read:e=>e.right}];
const all=P.drawSeries(canvas(),samples,2000,500,series,{min:-100,max:100});
const right=P.drawSeries(canvas(),samples,2000,500,series,{min:-100,max:100,visible:new Set(['motorR'])});
assert.equal(all.length,4);
assert.equal(right.length,2);
assert(right.every(point=>point.label==='Right'));
assert.equal(samples.length,2); // La visibilidad no altera adquisición/historial.
const digital=[{type:'ir_changed',us:1000,index:4,detected:true},
  {type:'start_changed',us:1500,start:true},
  {type:'ir_changed',us:2000,index:4,detected:false}];
const onlyIr=P.drawLogic(canvas(),digital,2000,500,{visible:new Set(['ir4']),resolver});
assert.equal(onlyIr.length,2);
assert(onlyIr.every(point=>point.label==='IR4'));
const onlyStart=P.drawStateTimeline(canvas(),digital,2000,500,{visible:new Set(['start']),resolver});
assert.equal(onlyStart.length,1);
assert.equal(onlyStart[0].label,'START');
console.log('Correcto: ms, metadata, Δ/tick y curvas ocultas sin pérdida de datos.');
