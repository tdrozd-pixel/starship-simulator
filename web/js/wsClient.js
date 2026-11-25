// Generic reconnecting WebSocket client. No three, no DOM. WebSocketImpl is injectable for tests.
// decodeBinary(arrayBuffer) returns {kind:'error', reason}, {kind:'unknown'} or any other object,
// which is handed to onData. An error whose reason mentions "version" is fatal (no reconnect).
// o.protocol is the hello protocol version this client speaks (default 1).
// acceptHello(msg) may return a reason string to reject a hello (fatal).
import { parseControl } from './protocol.js';

export function createWsClient(o) {
  const WS = o.WebSocketImpl || globalThis.WebSocket;
  const setT = o.setTimeoutFn || ((f, ms) => setTimeout(f, ms));
  const clearT = o.clearTimeoutFn || ((t) => clearTimeout(t));
  const nowFn = o.nowFn || (() => performance.now() / 1000);
  const cb = (name) => (typeof o[name] === 'function' ? o[name] : () => {});
  let ws = null, state = 'closed', attempt = 0, timer = null, fatal = false, wanted = false;
  let lastMessageWall = 0;

  function setState(s, extra) {
    state = s;
    cb('onState')(s, extra);
  }

  function fail(reason) {
    fatal = true;
    cb('onProtocolError')(reason);
    setState('error:' + reason);
    if (ws) { try { ws.close(); } catch (e) { /* ignore */ } }
  }

  function scheduleReconnect() {
    if (!wanted || fatal) return;
    const delay = Math.min(5, 0.5 * Math.pow(2, attempt)) * (0.75 + 0.5 * Math.random());
    attempt++;
    setState('reconnecting', delay);
    timer = setT(open, delay * 1000);
  }

  function open() {
    timer = null;
    setState('connecting');
    try {
      ws = new WS(o.url);
    } catch (e) {
      scheduleReconnect();
      return;
    }
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => { attempt = 0; lastMessageWall = nowFn(); setState('open'); };
    ws.onmessage = (ev) => {
      lastMessageWall = nowFn();
      const d = ev.data;
      if (typeof d === 'string') {
        const m = parseControl(d);
        if (!m) return;
        if (m.type === 'hello') {
          if (m.protocol !== (o.protocol ?? 1)) { fail('protocol version ' + m.protocol + ' unsupported'); return; }
          const why = typeof o.acceptHello === 'function' ? o.acceptHello(m) : null;
          if (why) { fail(why); return; }
          cb('onHello')(m);
        } else if (m.type === 'run_started') cb('onRunStarted')(m);
        return;
      }
      const r = o.decodeBinary(d);
      if (r.kind === 'unknown') return;
      if (r.kind === 'error') {
        if (/version/.test(r.reason)) fail(r.reason);
        else cb('onProtocolError')(r.reason);
        return;
      }
      cb('onData')(r);
    };
    ws.onclose = () => { ws = null; if (!fatal) scheduleReconnect(); };
    ws.onerror = () => { /* onclose follows */ };
  }

  return {
    connect() { wanted = true; fatal = false; if (!ws && !timer) open(); },
    close() {
      wanted = false;
      if (timer) { clearT(timer); timer = null; }
      if (ws) { try { ws.close(); } catch (e) { /* ignore */ } }
      setState('closed');
    },
    send(m) { if (ws && state === 'open') ws.send(m); },
    getState() { return state; },
    secondsSinceMessage() { return nowFn() - lastMessageWall; },
  };
}
