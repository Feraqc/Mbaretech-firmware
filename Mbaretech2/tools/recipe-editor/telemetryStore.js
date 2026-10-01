/* Historial acotado y métricas compartidas por todos los paneles. */
(function (root) {
'use strict';

class RingBuffer {
  constructor(capacity) {
    this.items=new Array(capacity);this.capacity=capacity;this.start=0;this.length=0;this.overwrites=0;
  }
  push(value) {
    const index=(this.start+this.length)%this.capacity;
    this.items[index]=value;
    if(this.length<this.capacity)this.length++;
    else {this.start=(this.start+1)%this.capacity;this.overwrites++;}
  }
  at(index) {
    if(index<0)index+=this.length;
    return index<0||index>=this.length?undefined:this.items[(this.start+index)%this.capacity];
  }
  valuesSince(minUs,predicate=()=>true) {
    const result=[];
    for(let i=0;i<this.length;i++){
      const item=this.at(i);
      if(item.us>=minUs&&predicate(item))result.push(item);
    }
    return result;
  }
  latest(count,predicate=()=>true) {
    const result=[];
    for(let i=this.length-1;i>=0&&result.length<count;i--){
      const item=this.at(i);if(predicate(item))result.push(item);
    }
    return result;
  }
  clear(){this.start=0;this.length=0;this.overwrites=0;}
}

class TelemetryStore {
  constructor(state,capacity=65536) {
    this.state=state;
    this.events=new RingBuffer(capacity);
    this.terminal=new RingBuffer(1200);
    this.bytes=0;this.bytesWindow=new RingBuffer(2048);
    this.connectAt=0;this.reconnects=0;
    this.maxSnapshotAgeMs=0;
    this.tasks=new Map();
    this.latestUs=0;
    this.listeners=new Set();
  }
  subscribe(listener){this.listeners.add(listener);return()=>this.listeners.delete(listener);}
  ingest(message,raw) {
    // Alias de análisis en ms; no se serializan en la captura canónica.
    const timeUs=Number.isFinite(message.us)?message.us:message.t*1000;
    if(!Object.prototype.hasOwnProperty.call(message,'timeUs'))
      Object.defineProperties(message,{
        timeUs:{value:timeUs},timeMs:{value:timeUs/1000},
        displayMachine:{value:message.machine||this.state.machine||null}
      });
    this.events.push(message);
    this.latestUs=Math.max(this.latestUs,message.us??0);
    this.terminal.push({event:message,raw});
    const bytes=raw.length;
    this.bytes+=bytes;
    this.bytesWindow.push({at:Date.now(),bytes});
    if(message.type==='sensors' && Number.isFinite(message.sampledAtMs))
      this.maxSnapshotAgeMs=Math.max(this.maxSnapshotAgeMs,Math.max(0,message.t-message.sampledAtMs));
    if(message.type==='task_timing') {
      const previous=this.tasks.get(message.task)||{maxExecutionUs:0,maxGapUs:0,samples:0};
      previous.maxExecutionUs=Math.max(previous.maxExecutionUs,message.executionUs);
      previous.maxGapUs=Math.max(previous.maxGapUs,message.gapUs);
      previous.samples++;
      this.tasks.set(message.task,previous);
    }
    for(const listener of this.listeners)listener(message);
  }
  window(ms) {
    return this.events.valuesSince(Math.max(0,this.latestUs-ms*1000));
  }
  byteRate(now=Date.now()) {
    let total=0;
    for(let i=this.bytesWindow.length-1;i>=0;i--){
      const point=this.bytesWindow.at(i);if(now-point.at>1000)break;total+=point.bytes;
    }
    return total;
  }
  clear() {
    this.events.clear();this.terminal.clear();this.bytesWindow.clear();
    this.bytes=0;this.maxSnapshotAgeMs=0;this.tasks.clear();this.latestUs=0;
  }
}
const api={RingBuffer,TelemetryStore};
if(typeof module==='object'&&module.exports)module.exports=api;
else root.MbaretechTelemetryStore=api;
})(typeof window!=='undefined'?window:globalThis);
