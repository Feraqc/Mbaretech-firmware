/* Gráficos Canvas reutilizables. Nunca acceden al transporte. */
(function (root) {
'use strict';
const F=typeof module==='object'&&module.exports?require('./telemetryPresentation.js'):root.MbaretechTelemetryPresentation;
const colors={ir:'#79c8df',line:'#d9a865',start:'#78c997',state:'#a997d8',step:'#d39ac0',motorL:'#6fb8e4',motorR:'#e4a16c',adcL:'#6fb8e4',adcR:'#e4a16c',fsm:'#a997d8',sensor:'#78c997'};
function prepare(canvas) {
  const width=Math.max(320,canvas.clientWidth||800),height=Math.max(80,canvas.clientHeight||200);
  const pixelRatio=Math.min(2,root.devicePixelRatio||1);
  if(canvas.width!==Math.round(width*pixelRatio)||canvas.height!==Math.round(height*pixelRatio)){
    canvas.width=Math.round(width*pixelRatio);canvas.height=Math.round(height*pixelRatio);
  }
  const ctx=canvas.getContext('2d');if(!ctx)return null;
  ctx.setTransform(pixelRatio,0,0,pixelRatio,0,0);ctx.clearRect(0,0,width,height);
  ctx.fillStyle='#101923';ctx.fillRect(0,0,width,height);
  ctx.font='10px ui-monospace,Consolas,monospace';
  return {ctx,width,height};
}
function axis(ctx,width,height,startUs,endUs,left=84) {
  ctx.strokeStyle='#273c4b';ctx.fillStyle='#8ca6b5';
  for(let i=0;i<=5;i++){
    const x=left+(width-left-10)*i/5;
    ctx.beginPath();ctx.moveTo(x,12);ctx.lineTo(x,height-22);ctx.stroke();
    const deviceMs=(startUs+(endUs-startUs)*i/5)/1000;
    const precision=endUs-startUs<=2000000?1:0;
    ctx.fillText(`${deviceMs.toFixed(precision)}${i===5?' ms':''}`,x-15,height-6);
  }
}
function xAt(us,startUs,endUs,width,left=84) {
  return left+(width-left-10)*Math.max(0,Math.min(1,(us-startUs)/Math.max(1,endUs-startUs)));
}
function drawLogic(canvas,events,endUs,windowMs,options={}) {
  const surface=prepare(canvas);if(!surface)return [];
  const {ctx,width,height}=surface,startUs=Math.max(0,endUs-windowMs*1000);
  axis(ctx,width,height,startUs,endUs,94);
  const resolver=options.resolver||new F.StateNameResolver();
  const rows=[...Array.from({length:7},(_,i)=>({key:`ir${i+1}`,label:`IR${i+1}`,color:colors.ir,
    read:e=>e.type==='ir_changed'&&e.index===i+1?e.detected:e.type==='sensors'?!!e.ir?.[i]:undefined})),
    ...[0,1].map(i=>({key:i?'lineR':'lineL',label:i?'LINE R':'LINE L',color:colors.line,
      read:e=>e.type==='line_changed'&&e.index===i?e.detected:e.type==='sensors'?!!e.line?.[i]:undefined})),
    {key:'start',label:'START',color:colors.start,read:e=>e.type==='start_changed'?e.start:e.type==='sensors'?e.start:undefined},
    {key:'state',label:'FSM STATE',color:colors.state,read:e=>{
      const id=e.type==='fsm_transition'?e.to:e.type==='fsm_status'?e.state:undefined;
      return id===undefined?undefined:resolver.getStateDisplayName(id,e.machine||e.displayMachine);
    }},
    {key:'step',label:'SUBFSM',color:colors.step,read:e=>{
      const step=e.type==='fsm_step'?e.nextStep:e.type==='fsm_status'?e.step:undefined;
      return step===undefined?undefined:resolver.getStepDisplayName(e.state,step,e.machine||e.displayMachine);
    }}].filter(row=>!options.visible||options.visible.has(row.key));
  if(!rows.length)return [];
  const rowHeight=(height-35)/rows.length;
  const hits=[];
  rows.forEach((row,index)=>{
    const y=10+rowHeight*(index+.5);
    ctx.fillStyle='#a5bdc9';ctx.fillText(row.label,7,y+3);
    ctx.strokeStyle='#263b49';ctx.beginPath();ctx.moveTo(94,y);ctx.lineTo(width-10,y);ctx.stroke();
    let previous=null,previousX=94;
    for(const event of events){
      if(event.us===undefined||event.us<startUs)continue;
      const value=row.read(event);if(value===undefined||value===null)continue;
      const x=xAt(event.us,startUs,endUs,width,94);
      const level=typeof value==='boolean'?y+(value?-rowHeight*.25:rowHeight*.25):y;
      ctx.strokeStyle=row.color;ctx.lineWidth=1.4;ctx.beginPath();
      if(previous!==null){ctx.moveTo(previousX,previous);ctx.lineTo(x,previous);ctx.lineTo(x,level);}
      else ctx.moveTo(x,level);
      ctx.stroke();
      if(typeof value!=='boolean'){
        ctx.fillStyle=row.color;ctx.fillText(String(value),x+3,y-3);
      }
      hits.push({x,y,event,label:row.label,value});
      previous=level;previousX=x;
    }
    if(previous!==null){ctx.beginPath();ctx.moveTo(previousX,previous);ctx.lineTo(width-10,previous);ctx.stroke();}
  });
  return hits;
}
function drawSeries(canvas,events,endUs,windowMs,series,options={}) {
  const surface=prepare(canvas);if(!surface)return [];
  const {ctx,width,height}=surface,startUs=Math.max(0,endUs-windowMs*1000),left=47;
  axis(ctx,width,height,startUs,endUs,left);
  const min=options.min??0,max=options.max??4095,range=Math.max(1,max-min);
  const hits=[];
  const selected=series.filter(item=>!options.visible||options.visible.has(item.key||item.label));
  for(let index=0;index<selected.length;index++){
    const item=selected[index],points=[];
    for(const event of events){
      if(event.us===undefined||event.us<startUs)continue;
      const value=item.read(event);if(!Number.isFinite(value))continue;
      const x=xAt(event.us,startUs,endUs,width,left);
      const y=12+(height-38)*(1-(value-min)/range);
      points.push({x,y,event,value,label:item.label});
    }
    ctx.strokeStyle=item.color||colors[item.key]||'#69b9d8';ctx.lineWidth=1.5;ctx.beginPath();
    points.forEach((point,i)=>i?ctx.lineTo(point.x,point.y):ctx.moveTo(point.x,point.y));ctx.stroke();
    ctx.fillStyle=item.color||colors[item.key]||'#69b9d8';ctx.fillText(item.label,left+4+index*105,12);
    hits.push(...points);
  }
  if(min<0&&max>0){const y=12+(height-38)*max/range;ctx.strokeStyle='#8594a0';ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-10,y);ctx.stroke();ctx.setLineDash([]);}
  ctx.fillStyle='#8ca6b5';ctx.fillText(String(max),3,18);ctx.fillText(String(min),3,height-25);
  return hits;
}
function drawStateTimeline(canvas,events,endUs,windowMs,options={}) {
  const surface=prepare(canvas);if(!surface)return [];
  const {ctx,width,height}=surface,startUs=Math.max(0,endUs-windowMs*1000),hits=[];
  axis(ctx,width,height,startUs,endUs,88);
  const resolver=options.resolver||new F.StateNameResolver();
  const rows=[{key:'state',label:'FSM STATE',types:['fsm_transition','fsm_status'],
      read:e=>resolver.getStateDisplayName(e.to??e.state,e.machine||e.displayMachine)},
    {key:'step',label:'SUBFSM',types:['fsm_step','fsm_status'],
      read:e=>resolver.getStepDisplayName(e.state,e.nextStep??e.step,e.machine||e.displayMachine)},
    {key:'run',label:'RUN',types:['fsm_status'],read:e=>e.running?'RUNNING':e.reason||'STOPPED'},
    {key:'start',label:'START',types:['start_changed','sensors'],read:e=>e.start?'ACTIVE':'STOPPED'}]
      .filter(row=>!options.visible||options.visible.has(row.key));
  rows.forEach((row,index)=>{
    const top=16+index*Math.max(31,(height-30)/Math.max(1,rows.length));
    ctx.fillStyle='#a5bdc9';ctx.fillText(row.label,5,top+15);
    const changes=events.filter(e=>e.us>=startUs&&row.types.includes(e.type)).map(e=>({event:e,value:row.read(e)}))
      .filter(point=>point.value!==undefined&&point.value!==null);
    let previous=null;
    for(const point of changes){
      if(previous&&previous.value===point.value)continue;
      if(previous)paint(previous,point.event.us);
      previous=point;
    }
    if(previous)paint(previous,endUs);
    function paint(point,until){
      const x=xAt(point.event.us,startUs,endUs,width,88),right=xAt(until,startUs,endUs,width,88);
      ctx.fillStyle=['#385f75','#5b517b','#567255','#785d47'][String(point.value).length%4];
      ctx.fillRect(x,top,Math.max(2,right-x),25);
      ctx.fillStyle='#e1eaf0';ctx.fillText(String(point.value),x+4,top+17,Math.max(0,right-x-6));
      hits.push({x,right,y:top,event:point.event,label:row.label,value:point.value,durationUs:until-point.event.us});
    }
  });
  return hits;
}
function nearest(hits,x,y,segment=false){
  let best=null,distance=Infinity;
  for(const hit of hits){
    const dx=segment?(x>=hit.x&&x<=hit.right?0:Infinity):Math.abs(hit.x-x);
    const score=dx+Math.abs(hit.y-y)*2;
    if(score<distance){best=hit;distance=score;}
  }
  return distance<35?best:null;
}
function valuesAtX(hits,x){
  const values=new Map();
  for(const hit of hits){
    const current=values.get(hit.label),distance=Math.abs(hit.x-x);
    if(!current||distance<current.distance)values.set(hit.label,{...hit,distance});
  }
  return [...values.values()];
}
function formatEvent(event) {
  return `${F.getEventTypeLabel(event.type)}  ${F.formatEventDetails(event)}`;
}
const api={drawLogic,drawSeries,drawStateTimeline,nearest,valuesAtX,formatEvent};
if(typeof module==='object'&&module.exports)module.exports=api;
else root.MbaretechTelemetryPanels=api;
})(typeof window!=='undefined'?window:globalThis);
