/* Protocolo, reconexión y almacén en vivo sin navegador ni robot físico. */
const assert=require('node:assert/strict');
const T=require('./telemetry.js');
const Bridge=require('./telemetryWindowBridge.js');

// Only graph-relevant changes cross the window boundary.
assert.equal(Bridge.graphMessage({type:'sensors'},null,null),null);
assert.equal(Bridge.graphMessage({type:'fsm_status',state:'SEARCH',step:-1},'SEARCH',-1),null);
assert.equal(Bridge.graphMessage({type:'fsm_status',state:'ATTACK',step:-1},'SEARCH',-1).state,'ATTACK');
assert.equal(Bridge.graphMessage({type:'fsm_transition',from:'SEARCH',to:'ATTACK'},'SEARCH',-1).to,'ATTACK');
class FakeChannel {
  static latest;
  constructor(name){assert.equal(name,Bridge.CHANNEL);FakeChannel.latest=this;}
  postMessage(value){this.sent=value;}
  close(){this.closed=true;}
}
let bridged;
const ipc=Bridge.createBridge(message=>{bridged=message;},FakeChannel);
ipc.publish({type:'console_status',connected:true});
assert.equal(FakeChannel.latest.sent.connected,true);
FakeChannel.latest.onmessage({data:{type:'hello',machine:'TEST'}});
assert.equal(bridged.machine,'TEST');
ipc.close();assert.equal(FakeChannel.latest.closed,true);

class FakeSocket {
  static instances=[];
  constructor(url){this.url=url;this.closed=false;FakeSocket.instances.push(this);}
  close(){this.closed=true;this.onclose?.();}
  open(){this.readyState=1;this.onopen?.();}
  send(value){this.sent=value;}
  receive(value){this.onmessage?.({data:JSON.stringify(value)});}
}
const timers={next:0,jobs:new Map(),setTimeout(fn,ms){this.jobs.set(++this.next,{fn,ms});return this.next;},
  clearTimeout(id){this.jobs.delete(id);},run(){const [[id,job]]=[...this.jobs];this.jobs.delete(id);job.fn();}};
const transport=new T.WebSocketTelemetryTransport(FakeSocket,timers);
const changes=[];
const session=new T.TelemetrySession(transport,reason=>changes.push(reason));
assert.throws(()=>transport.send({type:'param_values_request'}),/desconectado/);
assert.throws(()=>session.connect('http://192.168.4.1/ws'),/ws:\/\//);
session.connect('ws://192.168.4.1/ws');
assert.equal(session.state.status,'connecting');
const first=FakeSocket.instances.at(-1);
first.open();assert(session.state.connected);
transport.send({type:'param_values_request'});
assert.equal(first.sent,'{"type":"param_values_request"}');
transport.send({type:'start_set',transaction:7,active:false});
assert.equal(first.sent,'{"type":"start_set","transaction":7,"active":false}');
first.receive({type:'hello',protocol:1,machine:'TURN_CALIBRATION'});
assert.equal(session.state.helloMachine,'TURN_CALIBRATION');
first.receive({type:'sensors',t:100,start:true,ir:[0,0,0,1,0,0,0],line:[0,1],lineRaw:[1800,1760]});
first.receive({type:'motor',t:101,left:-40,right:40});
first.receive({type:'fsm_status',t:102,machine:'TURN_CALIBRATION',state:'TURN_SEQUENCE',step:1,elapsedMs:250});
first.receive({type:'fsm_step',t:350,machine:'TURN_CALIBRATION',state:'TURN_SEQUENCE',step:1,nextStep:2,condition:'TIMER',timerMs:350,elapsedMs:350});
assert.equal(session.state.currentState,'TURN_SEQUENCE');
assert.equal(session.state.currentStep,2);
assert.equal(session.state.lastTimerMs,350);
assert.deepEqual(session.state.motors,{left:-40,right:40});
assert.equal(session.state.sensors.ir[3],true);
assert.equal(session.state.sensors.rawLine[0],1800);
assert.equal(session.state.sensors.start,true);
session.pause(true);
first.receive({type:'fsm_transition',t:500,machine:'TURN_CALIBRATION',from:'TURN_SEQUENCE',to:'DONE',condition:'LINE_LEFT_DETECTED',elapsedMs:500});
assert.equal(session.state.currentState,'DONE'); // Pausa la pantalla, no la ingestión.
session.pause(false);
assert.equal(session.state.lastTransition.to,'DONE');
first.receive({type:'stats',t:501,dropped:3});
first.receive({type:'unknown',t:502});
assert.equal(session.state.stats.dropped,4);
assert.equal(session.state.stats.errors,1);
assert(changes.includes('hello')&&changes.includes('event'));
session.clear();assert.equal(session.state.console.length,0);

first.close();assert.equal(session.state.status,'reconnecting');
assert.equal(timers.jobs.size,1);
timers.run();
const second=FakeSocket.instances.at(-1);
assert.notEqual(second,first);
second.open();
second.receive({type:'hello',protocol:1,machine:'MOTOR_TEST'});
assert.equal(session.state.helloMachine,'MOTOR_TEST');
session.disconnect();
assert(second.closed&&!session.state.connected);
assert.equal(timers.jobs.size,0);
assert.throws(()=>T.decodeTelemetryMessage('{broken'),/JSON/);
assert.throws(()=>T.decodeTelemetryMessage(JSON.stringify({type:'hello',protocol:2,machine:'A'})),/Protocolo/);
assert.throws(()=>T.decodeTelemetryMessage(JSON.stringify({type:'motor',t:1,left:101,right:0})),/motores/);
// Canonical frames retain producer timestamps and report skipped sequence IDs.
session.receive(JSON.stringify({type:'hello',protocol:1,schema:2,machine:'MOTOR_TEST',seq:10,t:1,us:1000}));
session.receive(JSON.stringify({type:'start_changed',seq:12,t:2,us:2000,start:true}));
assert.equal(session.state.stats.sequenceGaps,1);
assert.equal(session.state.timeline.at(-1).us,2000);
assert.equal(session.state.sensors.start,true);
session.receive(JSON.stringify({type:'heartbeat',seq:13,t:3,us:3000,uptimeMs:3}));
assert.equal(session.state.uptimeMs,3);
session.state.currentState='MOTOR_SEQUENCE';
session.receive(JSON.stringify({type:'hello',protocol:1,schema:2,machine:'MOTOR_TEST',seq:14,t:4,us:4000}));
assert.equal(session.state.currentState,'MOTOR_SEQUENCE'); // HELLO periódico no borra el debugger.
const beforeErrors=session.state.stats.errors;
session.receive(JSON.stringify({type:'motor',t:4,left:1,right:2}));
assert.equal(session.state.stats.errors,beforeErrors+1);

// A slow/failed observer keeps bounded history and cannot alter another session.
const independent=new T.TelemetrySession(new T.TelemetryTransport());
for(let i=15;i<2115;i++)session.receive(JSON.stringify({type:'heartbeat',seq:i,t:i,us:i*1000,uptimeMs:i}));
assert.equal(session.state.history.length,2000);
assert.equal(session.state.console.length,400);
assert.equal(independent.state.stats.messages,0);

async function checkSerialAndBle(){
  const serialLines=[];
  const chunks=[new TextEncoder().encode('{"type":"heartbeat"}\n'),new TextEncoder().encode('{"type":"stats"}\n')];
  const serial={requestPort:async()=>({
    readable:{getReader(){return {read:async()=>chunks.length?{value:chunks.shift(),done:false}:{done:true},releaseLock(){},cancel:async()=>{}};}},
    open:async()=>{},close:async()=>{}
  })};
  const serialTransport=new T.SerialTelemetryTransport(serial);
  serialTransport.onData(line=>serialLines.push(line));
  await serialTransport.connect();
  assert.equal(serialLines.length,2);

  const bleLines=[];
  const characteristic={addEventListener(_,handler){this.handler=handler;},startNotifications:async()=>{}};
  const device={addEventListener(){},gatt:{connect:async()=>({getPrimaryService:async()=>({getCharacteristic:async()=>characteristic})}),disconnect(){}}};
  const bluetooth={requestDevice:async()=>device};
  const bleTransport=new T.BluetoothTelemetryTransport(bluetooth);
  bleTransport.onData(line=>bleLines.push(line));
  await bleTransport.connect();
  const bytes=new TextEncoder().encode('{"type":"heartbeat"}\n');
  characteristic.handler({target:{value:new DataView(bytes.buffer)}});
  assert.equal(bleLines.length,1);
  bleTransport.disconnect();
  console.log('Correcto: WebSocket/Serial/Bluetooth, secuencia, historial acotado y aislamiento.');
}
checkSerialAndBle().catch(error=>{console.error(error);process.exitCode=1;});
