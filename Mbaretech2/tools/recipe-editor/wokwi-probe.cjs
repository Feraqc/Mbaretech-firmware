/* Check live telemetry from Wokwi's localhost port forward. Node 22+ needed. */
'use strict';

const endpoint = process.argv[2] || 'ws://127.0.0.1:8180/ws';
const expectedMachine = process.argv[3] || 'state_test';
const includeTransitions = process.argv.includes('--transitions');
const required = new Set(['hello', 'sensors', 'fsm_status', 'motor']);
if (includeTransitions) {
  required.add('fsm_transition');
  required.add('fsm_step');
}
if (typeof WebSocket !== 'function') {
  console.error('This probe requires Node.js 22+ with built-in WebSocket support.');
  process.exit(1);
}

const seen = new Set();
let finished = false;
let socket;
const deadline = setTimeout(() => finish(false), 30000);

function finish(success) {
  if (finished) return;
  finished = true;
  clearTimeout(deadline);
  if (socket) socket.close();
  console.log(`${success ? 'PASS' : 'FAIL'}: ${expectedMachine}; seen ${[...seen].join(', ')}`);
  if (!success) console.error(`Missing: ${[...required].filter(type => !seen.has(type)).join(', ')}`);
  process.exitCode = success ? 0 : 1;
}

function validFrame(frame) {
  if (!frame || typeof frame.type !== 'string') return false;
  if (!Number.isInteger(frame.seq) || !Number.isSafeInteger(frame.us) || frame.us < 0) return false;
  if (frame.type === 'hello') return frame.protocol === 1 && frame.schema === 2 && frame.machine === expectedMachine;
  if (frame.type === 'sensors') return typeof frame.t === 'number' &&
    typeof frame.start === 'boolean' && frame.ir?.length === 7 && frame.line?.length === 2;
  if (frame.type === 'fsm_status') return frame.machine === expectedMachine &&
    typeof frame.t === 'number' && typeof frame.elapsedMs === 'number';
  if (frame.type === 'motor') return Number.isInteger(frame.left) && Number.isInteger(frame.right);
  if (frame.type === 'fsm_transition') return frame.machine === expectedMachine &&
    typeof frame.from === 'string' && typeof frame.to === 'string' &&
    typeof frame.condition === 'string';
  if (frame.type === 'fsm_step') return frame.machine === expectedMachine &&
    typeof frame.state === 'string' && Number.isInteger(frame.step) &&
    Number.isInteger(frame.nextStep) && typeof frame.condition === 'string';
  return false;
}

function connect() {
  if (finished) return;
  socket = new WebSocket(endpoint);
  socket.onmessage = event => {
    let frame;
    try { frame = JSON.parse(String(event.data)); } catch { return; }
    if (!required.has(frame.type)) return;
    if (!validFrame(frame)) {
      console.error(`Invalid ${frame.type} frame: ${event.data}`);
      finish(false);
      return;
    }
    if (!seen.has(frame.type)) console.log(event.data);
    seen.add(frame.type);
    if ([...required].every(type => seen.has(type))) finish(true);
  };
  socket.onerror = () => {}; // A reboot may briefly close the forwarded port.
  socket.onclose = () => {
    if (!finished) setTimeout(connect, 200);
  };
}

connect();
