/* Fuente de simulación: usa exactamente el mismo decoder y store que el robot. */
(function (root) {
'use strict';
class MockTelemetryTransport extends root.RecipeTelemetry.TelemetryTransport {
  constructor(clock=root) {
    super();this.clock=clock;this.timer=null;this.seq=0;this.us=0;
    this.stateIndex=0;this.stateTick=0;this.tickCount=0;this.ir=Array(7).fill(0);
    this.line=[0,0];this.start=false;this.machine='COMBAT';
    this.catalog=null;this.catalogSignature='';this.parameters=[];this.revision=0;
    this.bootId=0;this.activeParameters=new Map();
  }
  configureCatalog(catalog){
    if(!catalog?.machine||!Array.isArray(catalog.parameters))return;
    const signature=JSON.stringify([catalog.machine,catalog.states,catalog.parameters]);
    if(signature===this.catalogSignature)return;
    this.catalogSignature=signature;
    this.catalog=catalog;this.machine=catalog.machine;this.stateIndex=0;this.stateTick=0;
    this.parameters=catalog.parameters.map(item=>({...item,default:item.value,value:item.value}));
    this.revision=0;this.bootId++;
    // Un nuevo HELLO obliga a la consola a pedir el esquema de la receta activa.
    if(this.timer!==null)this.emit('hello',this.hello());
  }
  hello(){return {protocol:1,schema:2,machine:this.machine,paramSchema:1,
    paramRevision:this.revision,bootId:this.bootId};}
  send(command){
    if(this.timer===null)throw new Error('Mock desconectado.');
    if(command.type==='start_set'){
      if(!Number.isInteger(command.transaction)||typeof command.active!=='boolean')
        throw new Error('Comando START inválido.');
      this.setStart(command.active,true);
      this.emit('start_ack',{transaction:command.transaction,status:'accepted',
        active:this.start,source:'remote'});
      return;
    }
    if(command.type==='param_schema_request'){
      if(!this.parameters.length)this.emit('param_schema',{revision:this.revision,index:0,count:0});
      this.parameters.forEach((parameter,index)=>this.emit('param_schema',{
        revision:this.revision,index,count:this.parameters.length,id:parameter.id,
        parameterId:parameter.parameterId,
        name:parameter.name,group:parameter.group,unit:parameter.unit,valueType:'int32',
        default:parameter.default,min:parameter.min,max:parameter.max,step:parameter.step,
        applyPolicy:parameter.applyPolicy,writable:parameter.writable}));
      return;
    }
    if(command.type==='param_values_request'){
      if(!this.parameters.length)this.emit('param_values',{revision:this.revision,index:0,count:0});
      this.parameters.forEach((parameter,index)=>this.emit('param_values',{
        revision:this.revision,index,count:this.parameters.length,
        id:parameter.id,value:parameter.value}));
      return;
    }
    if(!['param_set','param_reset'].includes(command.type))throw new Error('Comando desconocido.');
    const changes=command.type==='param_reset'
      ?this.parameters.map(parameter=>({id:parameter.id,value:parameter.default}))
      :command.changes;
    let error='';
    if(command.machine!==this.machine)error='machine_mismatch';
    else if(this.start)error='start_must_be_off';
    else if((command.baseRevision??command.revision)!==this.revision)error='revision_conflict';
    else if(!Array.isArray(changes)||(!changes.length&&command.type==='param_set')||changes.length>this.parameters.length)
      error='invalid_changes';
    const seen=new Set();
    if(!error)for(const change of changes){
      const parameter=this.parameters.find(item=>item.id===change.id);
      if(!parameter||(command.type==='param_set'&&!parameter.writable)||seen.has(change.id)||!Number.isInteger(change.value)||
        change.value<parameter.min||change.value>parameter.max||
        (change.value-parameter.min)%parameter.step!==0){error='invalid_parameter';break;}
      seen.add(change.id);
    }
    if(!error)this.revision++;
    this.emit('param_ack',{transaction:command.transaction||0,revision:this.revision,
      status:error?'rejected':'accepted',effective:'next_state_entry',error});
    if(error)return;
    // Igual que el MCU: una transacción completa precede a los marcadores y valores.
    for(const change of changes){
      const parameter=this.parameters.find(item=>item.id===change.id);
      const old=parameter.value;parameter.value=change.value;
      if(old!==change.value)this.emit('param_changed',{
        transaction:command.transaction||0,revision:this.revision,
        id:change.id,old,value:change.value});
    }
    this.send({type:'param_values_request'});
  }
  connect() {
    this.disconnect();
    this.seq=0;this.us=0;this.tickCount=0;this.stateIndex=0;this.stateTick=0;
    this.bootId++;this.revision=0;
    for(const parameter of this.parameters)parameter.value=parameter.default;
    this.ir.fill(0);this.line=[0,0];this.start=false;
    this.activeParameters.clear();
    this.statusHandler?.('connected');
    this.emit('hello',this.hello());
    this.emit('start_changed',{start:false});
    this.timer=this.clock.setInterval(()=>this.tick(),20);
  }
  disconnect() {
    if(this.timer!==null)this.clock.clearInterval(this.timer);
    this.timer=null;this.statusHandler?.('disconnected');
  }
  setStart(active,forceRestart=false) {
    if(this.timer===null)throw new Error('Mock desconectado.');
    const next=Boolean(active);
    if(next===this.start&&!forceRestart)return;
    const changed=next!==this.start;
    this.start=next;
    // Una ejecución usa los valores aprobados antes de START; cambios posteriores esperan el próximo inicio.
    if(next)this.activeParameters=new Map(this.parameters.map(item=>[item.parameterId??item.id,item.value]));
    // Cada orden remota invalida el progreso y reinicia por el estado inicial.
    if(forceRestart||!next){this.stateIndex=0;this.stateTick=0;}
    // La misma señal canónica alimenta la consola, el historial y el editor.
    if(changed)this.emit('start_changed',{start:next});
    if(!next||forceRestart)this.emit('motor',{left:0,right:0,source:'START'});
    this.emit('fsm_status',{machine:this.machine,state:
      this.catalog?.states?.[this.stateIndex]?.id||['SEARCH','ATTACK','ESCAPE'][this.stateIndex%3],
      step:-1,elapsedMs:0,stepElapsedMs:0,running:next,
      reason:next?'RUNNING':'START_INACTIVE'});
  }
  emit(type,payload={}) {
    const message={type,seq:this.seq++>>>0,t:Math.floor(this.us/1000),us:this.us,...payload};
    this.messageHandler?.(JSON.stringify(message));
    this.us+=250; // El orden submilisegundo también se conserva en Mock.
    return message;
  }
  parameterValue(reference,literal,fallback){
    const parameter=this.parameters.find(item=>
      item.parameterId===reference||item.id===reference);
    return parameter&&this.activeParameters.has(parameter.parameterId??parameter.id)
      ?this.activeParameters.get(parameter.parameterId??parameter.id):literal??fallback;
  }
  tick() {
    this.tickCount++;
    this.us+=18000;
    if(this.start)this.stateTick++;
    const states=this.catalog?.states?.map(state=>state.id).filter(Boolean);
    const activeStates=states?.length?states:['SEARCH','ATTACK','ESCAPE'];
    if(this.tickCount%6===0){
      const index=(Math.floor(this.tickCount/6)%7);
      this.ir[index]=1-this.ir[index];
      this.emit('ir_changed',{index:index+1,detected:!!this.ir[index]});
    }
    if(this.tickCount%18===0){
      const index=Math.floor(this.tickCount/18)%2;
      this.line[index]=1-this.line[index];
      this.emit('line_changed',{index,detected:!!this.line[index],raw:this.line[index]?120:185});
    }
    const current=this.catalog?.states?.[this.stateIndex];
    const timer=current?.timers?.[0];
    const duration=timer?this.parameterValue(timer.timerParameter,timer.duration_ms,800):800;
    if(this.start&&this.stateTick*20>=duration){
      const from=activeStates[this.stateIndex%activeStates.length];
      this.stateIndex=(this.stateIndex+1)%activeStates.length;
      const to=activeStates[this.stateIndex];
      this.emit('fsm_transition',{machine:this.machine,from,to,
        condition:from==='SEARCH'?'IR4_DETECTED':from==='ATTACK'?'LINE_LEFT_DETECTED':'TIMER',
        timerMs:timer?duration:from==='ESCAPE'?800:0,elapsedMs:this.stateTick*20});
      this.stateTick=0;
      const fallback=to==='ATTACK'?[100,100]:to==='ESCAPE'?[-80,80]:[-40,40];
      const state=this.catalog?.states?.find(item=>item.id===to);
      const motor=state?.steps?.[0]?.motor??state?.motor;
      const command=motor?[
        this.parameterValue(motor.leftParameter,motor.left_speed_pct,fallback[0]),
        this.parameterValue(motor.rightParameter,motor.right_speed_pct,fallback[1])]:fallback;
      this.emit('motor',{left:command[0],right:command[1],source:to});
    }
    if(this.start&&this.stateIndex===1&&this.tickCount%12===0){
      const step=(Math.floor(this.tickCount/12)%2);
      this.emit('fsm_step',{machine:this.machine,state:'ATTACK',step,nextStep:1-step,
        condition:'TIMER',timerMs:240,elapsedMs:240});
    }
    if(this.tickCount%5===0){
      this.emit('sensors',{start:this.start,valid:true,sampledAtMs:Math.floor(this.us/1000)-1,
        lineThreshold:145,ir:this.ir.slice(),line:this.line.slice(),
        lineRaw:this.line.map(v=>v?120+this.tickCount%12:180+this.tickCount%9)});
      this.emit('fsm_status',{machine:this.machine,state:activeStates[this.stateIndex],
        step:this.stateIndex===1?Math.floor(this.tickCount/12)%2:-1,
        elapsedMs:this.stateTick*20,stepElapsedMs:(this.tickCount%12)*20,
        running:this.start,reason:this.start?'RUNNING':'START_INACTIVE'});
      this.emit('task_timing',{task:'sensor',executionUs:80+this.tickCount%23,gapUs:1000+this.tickCount%30});
      this.emit('task_timing',{task:'fsm',executionUs:65+this.tickCount%31,gapUs:1000+this.tickCount%42});
    }
    if(this.tickCount%50===0){
      this.emit('heartbeat',{uptimeMs:Math.floor(this.us/1000),wifiReady:true,wifiRssi:-48});
      this.emit('stats',{dropped:Math.floor(this.tickCount/500),serialDropped:0,bleDropped:0,wifiDropped:0});
    }
    // Tráfico de diagnóstico para probar ingesta rápida sin redibujar por paquete.
    for(let sample=0;sample<8;sample++)this.emit('log',{message:`mock trace ${this.tickCount}.${sample}`});
    if(this.tickCount%500===0)this.seq++; // Demuestra la detección de un salto de secuencia.
  }
}
if(typeof module==='object'&&module.exports)module.exports={MockTelemetryTransport};
else root.MbaretechMockTelemetry={MockTelemetryTransport};
})(typeof window!=='undefined'?window:globalThis);
