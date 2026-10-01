/* Estado de Live Test: valores MCU y edición pendiente nunca alteran la receta. */
(function(root){
'use strict';
class RuntimeTuning {
  constructor(){
    this.machine=null;this.schema=new Map();this.values=new Map();this.pending=new Map();
    this.revision=0;this.lastAck=null;this.inFlight=null;this.status='Esperando conexión.';
    this.schemaExpected=0;this.valuesExpected=0;
    this.schemaReceived=false;this.valuesReceived=false;
    this.schemaIndexes=new Set();this.valueIndexes=new Set();
    this.valuesSnapshotRevision=-1;
    this.bootId=null;this.lastHelloUs=null;this.lastHelloRevision=null;
    this.nextTransaction=(Date.now()%2000000000)+1;
  }
  ingest(message){
    if(message.type==='hello'){
      const bootChanged=Number.isInteger(message.bootId)&&this.bootId!==null&&
        message.bootId!==this.bootId;
      const clockRestarted=!Number.isInteger(message.bootId)&&
        Number.isFinite(message.us)&&this.lastHelloUs!==null&&message.us<this.lastHelloUs;
      const revisionRestarted=!Number.isInteger(message.bootId)&&
        Number.isInteger(message.paramRevision)&&this.lastHelloRevision!==null&&
        message.paramRevision<this.lastHelloRevision;
      const fresh=this.machine!==message.machine||bootChanged||clockRestarted||revisionRestarted;
      if(fresh){
        this.schema.clear();this.values.clear();this.pending.clear();this.inFlight=null;
        this.schemaExpected=0;this.valuesExpected=0;
        this.schemaReceived=false;this.valuesReceived=false;
        this.schemaIndexes.clear();this.valueIndexes.clear();
        this.valuesSnapshotRevision=-1;
        this.lastAck=null;
        this.status=this.machine===null?'Esperando esquema y valores del MCU.':
          'MCU reiniciado o receta cambiada; sincronizando defaults y valores.';
      }
      this.machine=message.machine;
      if(Number.isInteger(message.bootId))this.bootId=message.bootId;
      else if(fresh)this.bootId=null;
      if(Number.isFinite(message.us))this.lastHelloUs=message.us;
      if(Number.isInteger(message.paramRevision))this.lastHelloRevision=message.paramRevision;
      if(Number.isInteger(message.paramRevision))
        this.revision=fresh?message.paramRevision:Math.max(this.revision,message.paramRevision);
      return fresh;
    }else if(message.type==='param_schema'){
      this.schemaReceived=true;
      // Reintentos pueden llegar mientras la primera serie está incompleta.
      // Fusionar por ID evita ocultar filas ya recibidas al ver otro index 0.
      if(message.count!==this.schemaExpected){this.schemaIndexes.clear();this.schemaExpected=message.count;}
      if(message.id)this.schema.set(message.id,message);
      this.schemaIndexes.add(message.index);
      this.revision=Math.max(this.revision,message.revision);
    }else if(message.type==='param_values'){
      // Respuestas antiguas pueden llegar después de un ACK aceptado.
      if(message.revision<this.revision)return false;
      this.valuesReceived=true;
      if(message.count!==this.valuesExpected||message.revision!==this.valuesSnapshotRevision){
        this.valueIndexes.clear();this.valuesExpected=message.count;
        this.valuesSnapshotRevision=message.revision;
      }
      if(message.id){
        this.values.set(message.id,message.value);
        if(!this.inFlight&&this.pending.get(message.id)===message.value){
          this.pending.delete(message.id);
          this.status='Valor confirmado por el MCU tras consultar la revisión.';
        }
      }
      this.valueIndexes.add(message.index);
      this.revision=Math.max(this.revision,message.revision);
    }else if(message.type==='param_changed'){
      if(message.revision<this.revision)return false;
      this.values.set(message.id,message.value);this.revision=message.revision;
    }else if(message.type==='param_ack'){
      this.lastAck=message;this.revision=Math.max(this.revision,message.revision);
      if(this.inFlight?.transaction===message.transaction){
        if(message.status==='accepted'){
          for(const [id,value] of this.inFlight.changes){
            this.values.set(id,value);
            if(this.pending.get(id)===value)this.pending.delete(id);
          }
          this.status=`Aplicado en RAM · revisión ${message.revision} · ${message.effective}`;
        }else this.status=`Rechazado: ${message.error||'sin motivo'}`;
        this.inFlight=null;
      }
    }
  }
  stage(id,raw,recipeMeta=null){
    // El editor permite preparar un valor desde la receta antes de recibir todo
    // el catálogo; el envío exige después la autorización del MCU.
    const meta=this.schema.get(id)||recipeMeta,value=Number(raw);
    if(!meta||!meta.writable||!Number.isInteger(value)||value<meta.min||value>meta.max||
       (value-meta.min)%meta.step!==0)throw new Error('Valor fuera del rango del parámetro.');
    if(value===this.values.get(id))this.pending.delete(id);
    else this.pending.set(id,value);
  }
  discard(){this.pending.clear();this.status='Cambios pendientes descartados.';}
  prepareSet(){
    if(this.inFlight)throw new Error('Esperando ACK de la transacción anterior.');
    if(!this.pending.size||this.pending.size>12)throw new Error('Se requieren 1–12 cambios.');
    for(const [id,value] of this.pending){
      const meta=this.schema.get(id);
      if(!meta||!this.values.has(id)||!meta.writable||value<meta.min||value>meta.max||
         (value-meta.min)%meta.step!==0)
        throw new Error(`El MCU no confirmó el parámetro ${id}.`);
    }
    const transaction=this.nextTransaction++;
    const changes=[...this.pending].map(([id,value])=>({id,value}));
    this.inFlight={transaction,changes:new Map(this.pending)};
    this.status=`Enviando transacción #${transaction}…`;
    return {type:'param_set',machine:this.machine,baseRevision:this.revision,transaction,changes};
  }
  prepareReset(){
    if(this.inFlight)throw new Error('Esperando ACK de la transacción anterior.');
    const transaction=this.nextTransaction++;
    this.inFlight={transaction,changes:new Map()};
    this.status=`Restaurando defaults compilados · #${transaction}…`;
    return {type:'param_reset',machine:this.machine,baseRevision:this.revision,transaction};
  }
  sendFailed(reason){this.inFlight=null;this.status=`No enviado: ${reason}`;}
  relevant(stateId){
    const prefix=`state.${stateId}.`;
    return [...this.schema.values()].filter(item=>item.id.startsWith(prefix));
  }
  schemaComplete(){return this.schemaReceived&&this.schemaIndexes.size>=this.schemaExpected;}
  valuesComplete(){return this.valuesReceived&&this.valueIndexes.size>=this.valuesExpected;}
}
const api={RuntimeTuning};
if(typeof module==='object'&&module.exports)module.exports=api;
else root.MbaretechRuntimeTuning=api;
})(typeof window!=='undefined'?window:globalThis);
