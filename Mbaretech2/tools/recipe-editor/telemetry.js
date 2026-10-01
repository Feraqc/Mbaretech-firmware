/* Transporte, decodificación y estado normalizado; independiente del canvas. */
(function (root) {
'use strict';

function createLiveState() {
  return {
    status:'disconnected', connected:false, endpoint:'', machine:null, helloMachine:null,
    foreignMachine:null, protocol:null, schema:null, bootId:null, transport:'WiFi',
    currentState:null, currentStep:null, stateElapsedMs:0, stepElapsedMs:0,lastCondition:null,lastTimerMs:null,running:false,stoppedReason:null,
    lastTransition:null, motors:{left:0,right:0},motorSource:null,
    sensors:{start:false,ir:Array(7).fill(false),line:[false,false],rawLine:[-1,-1]},
    stats:{messages:0,dropped:0,errors:0,rate:0,sequenceGaps:0,serialDropped:0,bleDropped:0,wifiDropped:0},
    uptimeMs:0,lastPacketAt:0,lastSequence:null,valid:false,sampledAtMs:0,lineThreshold:null,
    network:{wifiReady:false,wifiRssi:null},
    timing:{},yaw:null,history:[],timeline:[],console:[],paused:false,autoScroll:true,
    parameters:{schema:new Map(),values:new Map(),revision:0,lastAck:null,lastChange:null}
  };
}

function decodeTelemetryMessage(raw) {
  if (typeof raw !== 'string' || raw.length > 4096) throw new Error('Mensaje de telemetría demasiado largo.');
  const data=JSON.parse(raw);
  if (!data || typeof data !== 'object' || Array.isArray(data) || typeof data.type !== 'string')
    throw new Error('Mensaje de telemetría inválido.');
  const id=x=>typeof x==='string' && /^[A-Za-z_][A-Za-z_0-9]*$/.test(x);
  const integer=x=>Number.isInteger(x) && x>=0 && x<=4294967295;
  const wheel=x=>Number.isInteger(x) && x>=-100 && x<=100;
  const bits=(x,n)=>Array.isArray(x) && x.length===n && x.every(v=>v===0||v===1||typeof v==='boolean');
  if(data.seq!==undefined && !integer(data.seq)) throw new Error('Secuencia inválida.');
  if(data.us!==undefined && (!Number.isSafeInteger(data.us)||data.us<0)) throw new Error('Timestamp inválido.');
  switch(data.type) {
    case 'hello':
      if(data.protocol!==1 || (data.schema!==undefined && data.schema!==2) || !id(data.machine)) throw new Error('Protocolo o máquina no compatible.');
      if(data.paramRevision!==undefined&&!integer(data.paramRevision))throw new Error('Revisión de parámetros inválida.');
      if(data.bootId!==undefined&&!integer(data.bootId))throw new Error('Identidad de arranque inválida.');
      break;
    case 'param_schema':
      if(!integer(data.t)||!integer(data.revision)||!integer(data.count)||!integer(data.index)||
         (data.parameterId!==undefined&&!integer(data.parameterId))||
         (data.count>0&&(!data.id||typeof data.id!=='string'||!Number.isInteger(data.default)||
           !Number.isInteger(data.min)||!Number.isInteger(data.max)||!Number.isInteger(data.step)||
           typeof data.applyPolicy!=='string'||typeof data.writable!=='boolean')))
        throw new Error('Esquema de parámetros inválido.');
      break;
    case 'param_values':
      if(!integer(data.t)||!integer(data.revision)||!integer(data.count)||!integer(data.index)||
         (data.count>0&&(typeof data.id!=='string'||!Number.isInteger(data.value))))
        throw new Error('Valores de parámetros inválidos.');
      break;
    case 'param_ack':
      if(!integer(data.t)||!integer(data.transaction)||!integer(data.revision)||
         !['accepted','rejected'].includes(data.status)||typeof data.error!=='string')
        throw new Error('ACK de parámetros inválido.');
      break;
    case 'param_changed':
      if(!integer(data.t)||!integer(data.transaction)||!integer(data.revision)||
         typeof data.id!=='string'||!Number.isInteger(data.old)||!Number.isInteger(data.value))
        throw new Error('Cambio de parámetro inválido.');
      break;
    case 'sensors':
      if(!integer(data.t)||typeof data.start!=='boolean'||!bits(data.ir,7)||!bits(data.line,2)||
         !Array.isArray(data.lineRaw)||data.lineRaw.length!==2||!data.lineRaw.every(Number.isInteger))
        throw new Error('Sensores inválidos.');
      break;
    case 'motor':
      if(!integer(data.t)||!wheel(data.left)||!wheel(data.right)) throw new Error('Comando de motores inválido.');
      break;
    case 'fsm_status':
      if(!integer(data.t)||!id(data.machine)||!(data.state===null||id(data.state))||
         !Number.isInteger(data.step)||!integer(data.elapsedMs)) throw new Error('Estado FSM inválido.');
      break;
    case 'fsm_transition':
      if(!integer(data.t)||!id(data.machine)||!id(data.from)||!id(data.to)||
         typeof data.condition!=='string'||!integer(data.elapsedMs)) throw new Error('Transición FSM inválida.');
      break;
    case 'fsm_step':
      if(!integer(data.t)||!id(data.machine)||!id(data.state)||!Number.isInteger(data.step)||
         !Number.isInteger(data.nextStep)||typeof data.condition!=='string'||
         !integer(data.elapsedMs)) throw new Error('Paso SubFSM inválido.');
      break;
    case 'stats':
      if(!integer(data.t)||!integer(data.dropped)) throw new Error('Estadísticas inválidas.');
      break;
    case 'heartbeat':
      if(!integer(data.t)||!integer(data.uptimeMs)) throw new Error('Heartbeat inválido.');
      break;
    case 'start_changed':
      if(!integer(data.t)||typeof data.start!=='boolean') throw new Error('START inválido.');
      break;
    case 'start_ack':
      if(!integer(data.t)||!integer(data.transaction)||data.status!=='accepted'||
         typeof data.active!=='boolean'||data.source!=='remote')
        throw new Error('ACK START inválido.');
      break;
    case 'ir_changed': case 'line_changed':
      if(!integer(data.t)||!Number.isInteger(data.index)||typeof data.detected!=='boolean') throw new Error('Cambio de sensor inválido.');
      break;
    case 'task_timing':
      if(!integer(data.t)||!id(data.task)||!integer(data.executionUs)||!integer(data.gapUs)) throw new Error('Timing inválido.');
      break;
    case 'error': case 'log':
      if(!integer(data.t)||typeof data.message!=='string') throw new Error('Log inválido.');
      break;
    case 'imu':
      if(!integer(data.t)||!Number.isInteger(data.yaw)||typeof data.valid!=='boolean') throw new Error('IMU inválida.');
      break;
    default: throw new Error('Tipo de telemetría desconocido: '+data.type);
  }
  return data;
}

class TelemetryTransport {
  connect() { throw new Error('Transporte no implementado.'); }
  disconnect() { throw new Error('Transporte no implementado.'); }
  send() { throw new Error('Este transporte no admite comandos de parámetros.'); }
  onMessage(handler) { this.messageHandler=handler; }
  onData(handler) { this.onMessage(handler); }
  onStatus(handler) { this.statusHandler=handler; }
}

class WebSocketTelemetryTransport extends TelemetryTransport {
  constructor(Socket=root.WebSocket, timers=root) {
    super(); this.Socket=Socket; this.timers=timers; this.socket=null;
    this.wanted=false; this.retry=null; this.attempt=0; this.endpoint=''; this.generation=0;
  }
  connect(endpoint) {
    const url=new URL(endpoint);
    if(!['ws:','wss:'].includes(url.protocol)||!url.hostname) throw new Error('Usa una dirección ws:// o wss://.');
    if(!this.Socket) throw new Error('Este navegador no admite WebSocket.');
    this.disconnect(); this.wanted=true; this.endpoint=url.href; this.attempt=0;
    this.open();
  }
  open() {
    if(!this.wanted)return;
    const generation=++this.generation;
    this.statusHandler?.('connecting');
    let socket;
    try { socket=new this.Socket(this.endpoint); }
    catch(error) { this.statusHandler?.('error'); this.scheduleReconnect(); return; }
    this.socket=socket;
    socket.onopen=()=>{ if(generation!==this.generation)return;this.attempt=0;this.statusHandler?.('connected'); };
    socket.onmessage=event=>{ if(generation===this.generation)this.messageHandler?.(event.data); };
    socket.onerror=()=>{ if(generation===this.generation){this.statusHandler?.('error');socket.close();} };
    socket.onclose=()=>{
      if(generation!==this.generation)return;
      this.socket=null;this.statusHandler?.(this.wanted?'reconnecting':'disconnected');
      if(this.wanted)this.scheduleReconnect();
    };
  }
  scheduleReconnect() {
    if(!this.wanted||this.retry!==null)return;
    const delay=Math.min(8000,1000*2**Math.min(this.attempt++,3));
    this.retry=this.timers.setTimeout(()=>{this.retry=null;this.open();},delay);
  }
  disconnect() {
    this.wanted=false;this.generation++;
    if(this.retry!==null){this.timers.clearTimeout(this.retry);this.retry=null;}
    if(this.socket){this.socket.close();this.socket=null;}
    this.statusHandler?.('disconnected');
  }
  send(command){
    if(!this.socket||this.socket.readyState!==1)throw new Error('WiFi desconectado.');
    this.socket.send(JSON.stringify(command));
  }
}

// Web Serial and Web Bluetooth only carry newline-delimited canonical frames.
// Neither transport exposes writes to the editor's observer-only session.
class SerialTelemetryTransport extends TelemetryTransport {
  constructor(serial=root.navigator?.serial){super();this.serial=serial;this.port=null;this.reader=null;this.active=false;}
  async connect(){
    if(!this.serial)throw new Error('Web Serial no disponible en este navegador.');
    this.statusHandler?.('connecting');
    try {this.port=await this.serial.requestPort();await this.port.open({baudRate:115200});}
    catch(error){this.statusHandler?.('error');throw error;}
    this.active=true;this.statusHandler?.('connected');
    const decoder=new TextDecoder();let pending='';
    try {
      while(this.active && this.port.readable){
        const reader=this.port.readable.getReader();this.reader=reader;
        try {
          for(;;){const {value,done}=await reader.read();if(done){this.active=false;break;}if(!this.active)break;
            pending+=decoder.decode(value,{stream:true});
            if(pending.length>8192){pending='';continue;}
            let end;while((end=pending.indexOf('\n'))>=0){
              const line=pending.slice(0,end).trim();pending=pending.slice(end+1);
              if(line.startsWith('{'))this.messageHandler?.(line);
            }
          }
        } finally {reader.releaseLock();this.reader=null;}
      }
    } catch {if(this.active)this.statusHandler?.('error');}
    finally {await this.disconnect();}
  }
  async disconnect(){this.active=false;try{await this.reader?.cancel();}catch{}
    try{await this.port?.close();}catch{}this.port=null;this.statusHandler?.('disconnected');}
  async send(command){
    if(!this.port?.writable)throw new Error('Serial desconectado.');
    const writer=this.port.writable.getWriter();
    try{await writer.write(new TextEncoder().encode(JSON.stringify(command)+'\n'));}
    finally{writer.releaseLock();}
  }
}

class BluetoothTelemetryTransport extends TelemetryTransport {
  constructor(bluetooth=root.navigator?.bluetooth){super();this.bluetooth=bluetooth;this.device=null;this.buffer='';this.rx=null;}
  async connect(){
    if(!this.bluetooth)throw new Error('Web Bluetooth no disponible en este navegador.');
    this.statusHandler?.('connecting');
    try {
    const service='6e400001-b5a3-f393-e0a9-e50e24dcca9e';
    this.device=await this.bluetooth.requestDevice({filters:[{services:[service]}]});
    this.device.addEventListener('gattserverdisconnected',()=>this.statusHandler?.('disconnected'));
    const server=await this.device.gatt.connect();
    this.rx=await (await server.getPrimaryService(service))
      .getCharacteristic('6e400002-b5a3-f393-e0a9-e50e24dcca9e');
    const characteristic=await (await server.getPrimaryService(service))
      .getCharacteristic('6e400003-b5a3-f393-e0a9-e50e24dcca9e');
    characteristic.addEventListener('characteristicvaluechanged',event=>{
      this.buffer+=new TextDecoder().decode(event.target.value);
      if(this.buffer.length>8192){this.buffer='';return;}
      let end;while((end=this.buffer.indexOf('\n'))>=0){
        const line=this.buffer.slice(0,end).trim();this.buffer=this.buffer.slice(end+1);
        if(line.startsWith('{'))this.messageHandler?.(line);
      }
    });
    await characteristic.startNotifications();this.statusHandler?.('connected');
    } catch(error){this.statusHandler?.('error');throw error;}
  }
  disconnect(){this.device?.gatt?.disconnect();this.device=null;this.rx=null;this.statusHandler?.('disconnected');}
  async send(command){
    if(!this.rx)throw new Error('Bluetooth desconectado.');
    const bytes=new TextEncoder().encode(JSON.stringify(command)+'\n');
    // El firmware recompone una línea NDJSON a partir de fragmentos ATT.
    for(let offset=0;offset<bytes.length;offset+=20)
      await this.rx.writeValue(bytes.slice(offset,offset+20));
  }
}

class TelemetrySession {
  constructor(transport, onChange=()=>{}) {
    this.transport=transport;this.state=createLiveState();this.onChange=onChange;
    this.rateStart=Date.now();this.rateCount=0;
    this.bindTransport(transport);
  }
  bindTransport(transport) {
    transport.onMessage(raw=>this.receive(raw));
    transport.onStatus(status=>{
      this.state.status=status;this.state.connected=status==='connected';
      if(!this.state.connected){this.state.currentState=null;this.state.currentStep=null;this.state.lastTransition=null;}
      this.onChange('connection');
    });
  }
  setTransport(transport,name) {
    this.transport.onMessage(()=>{});this.transport.onStatus(()=>{});
    this.transport.disconnect();this.transport=transport;this.state.transport=name;
    this.bindTransport(transport);this.onChange('connection');
  }
  connect(endpoint) {
    this.state.endpoint=endpoint;
    this.state.machine=null;this.state.helloMachine=null;this.state.foreignMachine=null;
    this.state.protocol=null;this.state.currentState=null;
    this.state.currentStep=null;this.state.lastTransition=null;
    this.state.lastSequence=null;
    this.state.parameters={schema:new Map(),values:new Map(),revision:0,lastAck:null,lastChange:null};
    return this.transport.connect(endpoint);
  }
  disconnect(){this.transport.disconnect();}
  receive(raw) {
    let message;
    try {message=decodeTelemetryMessage(raw);}
    catch(error) {
      this.state.stats.errors++;this.state.stats.dropped++;
      this.pushConsole(null,'ERROR '+error.message);this.onChange('error');return;
    }
    const s=this.state;
    if(s.schema===2 && message.type!=='hello' && (message.seq===undefined || message.us===undefined)){
      s.stats.errors++;this.pushConsole(null,'ERROR Missing sequence or microsecond timestamp');
      this.onChange('error');return;
    }
    const previousState=s.currentState, previousStep=s.currentStep;
    const previousForeign=s.foreignMachine;
    s.stats.messages++;this.rateCount++;
    const now=Date.now(),elapsed=now-this.rateStart;
    s.lastPacketAt=now;
    const newBoot=message.type==='hello'&&Number.isInteger(message.bootId)&&
      s.bootId!==null&&s.bootId!==message.bootId;
    if(message.type==='hello'&&(s.helloMachine!==message.machine||newBoot))s.lastSequence=null;
    if(message.seq!==undefined){
      if(s.lastSequence!==null){const gap=(message.seq-s.lastSequence-1)>>>0;
        if(gap<0x80000000)s.stats.sequenceGaps+=gap;}
      s.lastSequence=message.seq;
    }
    if(elapsed>=1000){s.stats.rate=Math.round(this.rateCount*1000/elapsed);this.rateCount=0;this.rateStart=now;}
    switch(message.type) {
      case 'hello':
        const changedMachine=s.helloMachine!==message.machine;
        s.machine=message.machine;s.helloMachine=message.machine;
        if(Number.isInteger(message.bootId))s.bootId=message.bootId;
        s.foreignMachine=null;s.protocol=message.protocol;s.schema=message.schema??null;
        if(changedMachine||newBoot){
          s.currentState=null;s.currentStep=null;s.lastTransition=null;
          s.parameters.schema.clear();s.parameters.values.clear();
          s.parameters.lastAck=null;s.parameters.lastChange=null;
        }
        if(Number.isInteger(message.paramRevision))s.parameters.revision=message.paramRevision;
        break;
      case 'param_schema':
        if(message.id)s.parameters.schema.set(message.id,message);
        s.parameters.revision=message.revision;break;
      case 'param_values':
        if(message.id)s.parameters.values.set(message.id,message.value);
        s.parameters.revision=message.revision;break;
      case 'param_ack':
        s.parameters.lastAck=message;s.parameters.revision=message.revision;break;
      case 'param_changed':
        s.parameters.lastChange=message;s.parameters.values.set(message.id,message.value);
        s.parameters.revision=message.revision;break;
      case 'sensors':
        s.sensors={start:message.start,ir:message.ir.map(Boolean),line:message.line.map(Boolean),rawLine:message.lineRaw.slice()};
        s.valid=message.valid??false;s.sampledAtMs=message.sampledAtMs??message.t;
        s.lineThreshold=message.lineThreshold??null;break;
      case 'motor':s.motors={left:message.left,right:message.right};s.motorSource=message.source??null;break;
      case 'fsm_status':
        s.machine=message.machine;
        if(s.helloMachine && message.machine!==s.helloMachine)s.foreignMachine=message.machine;
        s.currentState=message.state;
        s.currentStep=message.step<0?null:message.step;s.stateElapsedMs=message.elapsedMs;
        s.stepElapsedMs=message.stepElapsedMs??0;
        s.running=message.running??true;s.stoppedReason=message.reason??null;break;
      case 'fsm_transition':
        s.machine=message.machine;
        if(s.helloMachine && message.machine!==s.helloMachine)s.foreignMachine=message.machine;
        s.currentState=message.to;s.currentStep=null;
        s.lastCondition=message.condition;s.lastTimerMs=message.timerMs??null;
        s.lastTransition=message;s.stateElapsedMs=0;break;
      case 'fsm_step':
        s.machine=message.machine;
        if(s.helloMachine && message.machine!==s.helloMachine)s.foreignMachine=message.machine;
        s.currentState=message.state;
        s.currentStep=message.nextStep<0?null:message.nextStep;
        s.lastCondition=message.condition;s.lastTimerMs=message.timerMs??null;
        s.lastTransition=message;break;
      case 'stats':
        s.stats.dropped=message.dropped;s.stats.serialDropped=message.serialDropped??0;
        s.stats.bleDropped=message.bleDropped??0;s.stats.wifiDropped=message.wifiDropped??0;break;
      case 'heartbeat':s.uptimeMs=message.uptimeMs;
        s.network={wifiReady:message.wifiReady??false,wifiRssi:message.wifiRssi??null};break;
      case 'start_changed':s.sensors.start=message.start;break;
      case 'start_ack':s.startAck=message;s.sensors.start=message.active;break;
      case 'ir_changed':if(message.index>=1&&message.index<=7)s.sensors.ir[message.index-1]=message.detected;break;
      case 'line_changed':if(message.index>=0&&message.index<=1){s.sensors.line[message.index]=message.detected;s.sensors.rawLine[message.index]=message.raw;}break;
      case 'task_timing':s.timing[message.task]={executionUs:message.executionUs,gapUs:message.gapUs};break;
      case 'imu':s.yaw=message.valid?message.yaw:null;break;
    }
    s.history.push(message);
    if(s.history.length>2000)s.history.splice(0,s.history.length-2000);
    if(['fsm_transition','fsm_step','start_changed','start_ack','ir_changed','line_changed','motor','error','param_changed','param_ack'].includes(message.type)){
      s.timeline.push(message);if(s.timeline.length>500)s.timeline.splice(0,s.timeline.length-500);
    }
    this.pushConsole(message.t,raw);
    const changedState=s.foreignMachine!==previousForeign || message.type==='fsm_status' &&
      (s.currentState!==previousState || s.currentStep!==previousStep);
    this.onChange(message.type==='hello'?'hello':
      ['fsm_transition','fsm_step'].includes(message.type)||changedState?'event':'sample');
  }
  pushConsole(t,text) {
    const entries=this.state.console;
    entries.push({t,text});
    if(entries.length>400)entries.splice(0,entries.length-400);
  }
  clear(){this.state.console.length=0;this.onChange('clear');}
  pause(value){this.state.paused=value;this.onChange('pause');}
}

const exported={createLiveState,decodeTelemetryMessage,TelemetryTransport,WebSocketTelemetryTransport,
  SerialTelemetryTransport,BluetoothTelemetryTransport,TelemetrySession};
if(typeof module==='object'&&module.exports)module.exports=exported;
else root.RecipeTelemetry=exported;
})(typeof window!=='undefined'?window:globalThis);
