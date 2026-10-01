/* Store, Mock y paneles comparten mensajes normalizados. */
const assert=require('node:assert/strict');
const T=require('./telemetry.js');
const {RingBuffer,TelemetryStore}=require('./telemetryStore.js');
const P=require('./telemetryPanels.js');
global.RecipeTelemetry=T;
const {MockTelemetryTransport}=require('./telemetryMock.js');

const ring=new RingBuffer(3);
for(let i=0;i<5;i++)ring.push({us:i});
assert.deepEqual(ring.latest(3).map(event=>event.us),[4,3,2]);
assert.equal(ring.overwrites,2);
assert.deepEqual(ring.valuesSince(3).map(event=>event.us),[3,4]);

const timers={task:null,setInterval(fn){this.task=fn;return 1;},clearInterval(){this.task=null;}};
const transport=new MockTelemetryTransport(timers);
const session=new T.TelemetrySession(transport);
const store=new TelemetryStore(session.state,6000);
const received=[];
transport.onMessage(raw=>{
  const event=T.decodeTelemetryMessage(raw);
  session.receive(raw);store.ingest(event,raw);received.push(event);
});
session.connect();
assert.equal(session.state.connected,true);
transport.send({type:'start_set',transaction:71,active:false});
assert(received.some(event=>event.type==='start_ack'&&event.transaction===71&&event.active===false));
assert(received.some(event=>event.type==='start_changed'&&event.start===false));
const transitionsBeforeStop=received.filter(event=>event.type==='fsm_transition').length;
for(let i=0;i<45;i++)timers.task();
assert.equal(received.filter(event=>event.type==='fsm_transition').length,transitionsBeforeStop);
assert.equal(session.state.sensors.start,false);
transport.send({type:'start_set',transaction:72,active:true});
assert.equal(session.state.sensors.start,true);
transport.configureCatalog({machine:'EDITOR_TEST',states:[{id:'IDLE'},{id:'DRIVE',motor:{
  left_speed_pct:30,right_speed_pct:30,leftParameter:1},timers:[{
  duration_ms:400,timerParameter:2,macro:'DRIVE_TIMER'}]}],parameters:[
  {id:'state.DRIVE.left',parameterId:1,name:'Left motor',group:'DRIVE',unit:'%',value:30,
    min:-100,max:100,step:1,applyPolicy:'next_state_entry',writable:true},
  {id:'drive_duration',parameterId:2,name:'Drive duration',group:'DRIVE',unit:'ms',value:400,
    min:0,max:60000,step:1,applyPolicy:'next_state_entry',writable:true}
]});
transport.send({type:'param_schema_request'});
transport.send({type:'param_values_request'});
transport.send({type:'param_set',machine:'EDITOR_TEST',transaction:3,baseRevision:0,
  changes:[{id:'state.DRIVE.left',value:42}]});
assert(received.some(event=>event.type==='param_ack'&&event.transaction===3&&event.status==='rejected'&&
  event.error==='start_must_be_off'));
transport.send({type:'start_set',transaction:73,active:false});
transport.send({type:'param_set',machine:'EDITOR_TEST',transaction:4,baseRevision:0,
  changes:[{id:'state.DRIVE.left',value:42}]});
assert(received.some(event=>event.type==='param_schema'&&event.id==='state.DRIVE.left'));
assert(received.some(event=>event.type==='param_ack'&&event.status==='accepted'&&event.revision===1));
assert(received.some(event=>event.type==='param_changed'&&event.old===30&&event.value===42));
transport.send({type:'param_set',machine:'EDITOR_TEST',transaction:5,baseRevision:0,
  changes:[{id:'state.DRIVE.left',value:60}]});
assert(received.some(event=>event.type==='param_ack'&&event.status==='rejected'&&event.error==='revision_conflict'));
transport.send({type:'param_reset',machine:'EDITOR_TEST',transaction:6,baseRevision:1});
assert(received.some(event=>event.type==='param_changed'&&event.old===42&&event.value===30));
transport.send({type:'param_set',machine:'EDITOR_TEST',transaction:7,baseRevision:2,
  changes:[{id:'state.DRIVE.left',value:42},{id:'drive_duration',value:1200}]});
transport.send({type:'start_set',transaction:74,active:true});
for(let i=0;i<510;i++)timers.task();
assert(received.some(event=>event.type==='motor'&&event.source==='DRIVE'&&event.left===42&&event.right===30));
assert(received.some(event=>event.type==='fsm_transition'&&event.from==='DRIVE'&&
  event.timerMs===1200&&event.elapsedMs===1200));
assert.equal(session.state.machine,'EDITOR_TEST');
assert(received.some(event=>event.type==='fsm_transition'));
assert(received.some(event=>event.type==='fsm_step'));
assert(received.some(event=>event.type==='line_changed'));
assert(received.some(event=>event.type==='motor'));
assert(received.some(event=>event.type==='task_timing'));
assert(session.state.stats.sequenceGaps>=1);
assert(store.events.length<=store.events.capacity);
assert(store.byteRate()>=0);
assert.match(P.formatEvent(received.find(event=>event.type==='fsm_transition')),/FSM/);
assert(store.tasks.get('fsm').maxExecutionUs>0);
session.disconnect();
assert.equal(timers.task,null);
// Una reconexión Mock simula reboot y restaura defaults/revisión.
const previousBoot=transport.bootId;
session.connect();
assert.equal(transport.bootId,previousBoot+1);
assert.equal(transport.revision,0);
assert.equal(transport.parameters[0].value,30);
session.disconnect();
console.log('Correcto: Mock, ring buffer, marcas de tiempo, eventos FSM y métricas.');
