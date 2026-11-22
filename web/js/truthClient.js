// WebSocket client for /ws/truth: a thin wrapper over wsClient.js. No three, no DOM.
// WebSocketImpl is injectable for tests.
import { decodeBinary, makeRestartMessage } from './protocol.js';
import { createWsClient } from './wsClient.js';

export function createTruthClient(o) {
  const onSnapshot = typeof o.onSnapshot === 'function' ? o.onSnapshot : () => {};
  const c = createWsClient({
    ...o,
    decodeBinary,
    onData: (r) => { if (r.kind === 'snapshot') onSnapshot(r.snap); },
  });
  return {
    connect: c.connect,
    close: c.close,
    sendRestart() { c.send(makeRestartMessage()); },
    getState: c.getState,
    secondsSinceMessage: c.secondsSinceMessage,
  };
}
