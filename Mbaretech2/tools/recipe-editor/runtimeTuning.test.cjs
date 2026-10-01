/* Separación receta/MCU/pendiente y ACK transaccional. */
const assert=require('node:assert/strict');
const {RuntimeTuning}=require('./runtimeTuning.js');
const tuning=new RuntimeTuning();
const offline=new RuntimeTuning();
offline.stage('backward_duration',2400,{id:'backward_duration',min:0,max:60000,step:1,writable:true});
assert.equal(offline.pending.get('backward_duration'),2400);
assert.throws(()=>offline.prepareSet(),/no confirmó/);
offline.ingest({type:'hello',machine:'state_test',paramRevision:0,bootId:99});
assert.equal(offline.pending.size,0);
tuning.ingest({type:'hello',machine:'state_test',paramRevision:0,bootId:100});
tuning.ingest({type:'param_schema',revision:0,index:0,count:2,
  id:'state.MOTOR_SEQUENCE.left',name:'Left motor',unit:'%',default:30,
  min:-100,max:100,step:1,applyPolicy:'next_state_entry',writable:true});
tuning.ingest({type:'param_schema',revision:0,index:1,count:2,
  id:'state.MOTOR_SEQUENCE.timer.0',name:'State timer',unit:'ms',default:350,
  min:0,max:60000,step:1,applyPolicy:'next_state_entry',writable:true});
assert(tuning.schemaComplete());
// Un segundo envío de index 0 no debe borrar los otros IDs recibidos.
tuning.ingest({type:'param_schema',revision:0,index:0,count:2,
  id:'state.MOTOR_SEQUENCE.left',name:'Left motor',unit:'%',default:30,
  min:-100,max:100,step:1,applyPolicy:'next_state_entry',writable:true});
assert.equal(tuning.schema.size,2);
tuning.ingest({type:'param_values',revision:0,index:0,count:2,id:'state.MOTOR_SEQUENCE.left',value:30});
tuning.ingest({type:'param_values',revision:0,index:1,count:2,id:'state.MOTOR_SEQUENCE.timer.0',value:350});
assert(tuning.valuesComplete());
assert.equal(tuning.relevant('MOTOR_SEQUENCE').length,2);
assert.throws(()=>tuning.stage('state.MOTOR_SEQUENCE.left',101),/fuera/);
tuning.stage('state.MOTOR_SEQUENCE.left',45);
tuning.stage('state.MOTOR_SEQUENCE.timer.0',370);
assert.equal(tuning.values.get('state.MOTOR_SEQUENCE.left'),30);
const command=tuning.prepareSet();
assert.equal(command.changes.length,2);
assert.equal(command.baseRevision,0);
assert.equal(command.machine,'state_test');
tuning.ingest({type:'param_ack',transaction:command.transaction,status:'rejected',revision:0,error:'revision conflict'});
assert.equal(tuning.pending.size,2);
const retry=tuning.prepareSet();
tuning.ingest({type:'param_ack',transaction:retry.transaction,status:'accepted',revision:1,effective:'next_state_entry',error:''});
assert.equal(tuning.pending.size,0);
assert.equal(tuning.values.get('state.MOTOR_SEQUENCE.left'),45);
assert.equal(tuning.revision,1);
// Un snapshot solicitado antes del ACK no debe rebajar revisión ni valor.
tuning.ingest({type:'param_values',revision:0,index:0,count:2,
  id:'state.MOTOR_SEQUENCE.left',value:30});
assert.equal(tuning.revision,1);
assert.equal(tuning.values.get('state.MOTOR_SEQUENCE.left'),45);
tuning.stage('state.MOTOR_SEQUENCE.left',50);
tuning.discard();
assert.equal(tuning.pending.size,0);
assert.equal(tuning.values.get('state.MOTOR_SEQUENCE.left'),45);
// Si se pierde el ACK, la consulta posterior puede confirmar la escritura.
tuning.stage('state.MOTOR_SEQUENCE.left',52);
tuning.ingest({type:'param_values',revision:2,index:0,count:2,
  id:'state.MOTOR_SEQUENCE.left',value:52});
assert.equal(tuning.pending.size,0);
assert.equal(tuning.values.get('state.MOTOR_SEQUENCE.left'),52);
// Un HELLO de otro arranque invalida esquema, valores y borrador.
tuning.stage('state.MOTOR_SEQUENCE.left',50);
assert.equal(tuning.ingest({type:'hello',machine:'state_test',paramRevision:0,bootId:101}),true);
assert.equal(tuning.schema.size,0);
assert.equal(tuning.values.size,0);
assert.equal(tuning.pending.size,0);
assert.equal(tuning.revision,0);
assert.equal(tuning.schemaComplete(),false);
// Un esquema explícitamente vacío está completo; antes se mostraba 1/? siempre.
tuning.ingest({type:'param_schema',revision:0,index:0,count:0});
tuning.ingest({type:'param_values',revision:0,index:0,count:0});
assert.equal(tuning.schemaComplete(),true);
assert.equal(tuning.valuesComplete(),true);
console.log('Correcto: Live Test separa defaults, ejecución, pendientes y ACK.');
