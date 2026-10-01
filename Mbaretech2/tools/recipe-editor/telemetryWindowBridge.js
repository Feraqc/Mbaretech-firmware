/* The console owns the robot connection; the editor receives only graph events. */
(function (root) {
'use strict';
const CHANNEL = 'mbaretech-telemetry';
const GRAPH_TYPES = new Set(['hello', 'start_changed', 'start_ack', 'fsm_transition', 'fsm_step',
  'ir_changed', 'line_changed', 'error', 'param_schema', 'param_values',
  'param_ack', 'param_changed']);

function createBridge(onMessage, Channel = root.BroadcastChannel) {
  if (!Channel) return {publish(){}, close(){}};
  const channel = new Channel(CHANNEL);
  channel.onmessage = event => onMessage?.(event.data);
  return {
    publish(message) { channel.postMessage(message); },
    close() { channel.close(); }
  };
}

function graphMessage(message, previousState, previousStep, previousRunning, previousReason) {
  if (GRAPH_TYPES.has(message.type)) return message;
  if (message.type === 'fsm_status' &&
      (message.state !== previousState || message.step !== previousStep ||
       message.running !== previousRunning || message.reason !== previousReason)) return message;
  return null;
}

const api = {CHANNEL, createBridge, graphMessage};
if (typeof module === 'object' && module.exports) module.exports = api;
else root.TelemetryWindowBridge = api;
})(typeof window !== 'undefined' ? window : globalThis);
