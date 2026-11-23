// Widget library for the GNC console: renders the declarative screen specs of regimes.js.
// DOM-bound. buildScreen creates the DOM and chart objects of one screen; update() refreshes the
// text (and, when asked, the canvases) of that screen only, so a screen that is not shown costs
// nothing. Widgets read values through paramFormat.js and the model from gncModel.js.
import { APID, NUM_ENGINES, RAD2DEG } from './gncPackets.js';
import { fmtSigned } from './gncModel.js';
import { formatParam, paramUnit, resolveParam, parseParam } from './paramFormat.js';
import { measure } from './canvasUtil.js';
import { drawGimbalBox } from './gimbalView.js';
import { drawAttitude } from './attitudeView.js';
import { drawTimeline } from './timelineView.js';

function h(tag, cls, text) {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (text !== undefined) el.textContent = text;
  return el;
}

// Text node holder that writes only when the string changes.
function textCell(el) {
  let last = null;
  return (s) => { if (s !== last) { el.textContent = s; last = s; } };
}

function panel(title, ...children) {
  const el = h('section', 'panel');
  if (title) el.append(h('h2', '', title.toUpperCase()));
  el.append(...children);
  return el;
}

function gimbalText(cmd, meas) {
  const f = (r) => fmtSigned(r * RAD2DEG, 2);
  const c = cmd ? 'CMD  P ' + f(cmd.pitch) + ' Y ' + f(cmd.yaw) : 'CMD  --';
  const m = meas ? 'MEAS P ' + f(meas.pitch) + ' Y ' + f(meas.yaw) : 'MEAS --';
  const t = cmd ? 'THR  ' + (cmd.on ? (cmd.throttle * 100).toFixed(0) + ' %' : 'OFF') : 'THR  --';
  return c + '<br>' + (meas && !meas.valid ? '<span class="bad">' + m + ' INVALID</span>' : m) + '<br>' + t;
}

// ctx: {store, charts, map, timeline, link}. Each builder returns {el, update?, mount?, destroy?}.
const BUILDERS = {
  readouts(spec, ctx) {
    const el = h('div', 'big-row' + (spec.small ? ' small' : ''));
    const cells = spec.items.map((it) => {
      const box = h('div', 'big');
      const val = h('span', 'val', '--');
      const unit = paramUnit(it);
      box.append(h('span', 'lbl', it.label), val, h('span', 'unit', unit));
      el.append(box);
      return { it, set: textCell(val) };
    });
    return { el, update(model) { for (const c of cells) c.set(formatParam(c.it, ctx.store, model)); } };
  },

  lamps(spec, ctx) {
    const box = h('div', 'lamps');
    const lamps = spec.items.map((it) => { const e = h('span', 'lamp', it.label); box.append(e); return { it, e, p: parseParam(it.param) }; });
    return {
      el: spec.title ? panel(spec.title, box) : box,
      update() { for (const l of lamps) l.e.classList.toggle('on', !!resolveParam(ctx.store, l.p)); },
    };
  },

  table(spec, ctx) {
    const t = h('table', 'kv');
    if (spec.head) {
      const tr = h('tr');
      tr.append(h('th'), ...spec.head.map((x) => h('th', '', x)));
      t.append(tr);
    }
    const rows = spec.rows.map((r) => {
      const tr = h('tr');
      tr.append(h('td', '', r.label));
      const cells = spec.head ? r.cells.map(() => h('td', '', '--')) : [h('td', '', '--')];
      tr.append(...cells);
      for (let i = r.cells.length; spec.head && i < spec.head.length; i++) tr.append(h('td', '', '--')); // short rows: blank cells
      t.append(tr);
      const unit = r.unit !== undefined ? r.unit : (spec.head ? '' : paramUnit(r.cells[0]));
      return { r, unit, sets: cells.map(textCell) };
    });
    return {
      el: spec.title ? panel(spec.title, t) : t,
      update(model) {
        for (const row of rows) {
          const vals = row.r.cells.map((c) => formatParam(c, ctx.store, model));
          if (spec.head) row.sets.forEach((set, i) => set(vals[i]));
          else row.sets[0](vals.join(row.r.sep || '') + (row.unit ? ' ' + row.unit : ''));
        }
      },
    };
  },

  map(spec, ctx) {
    const canvas = h('canvas');
    canvas.className = 'map';
    const slot = h('div', 'map-slot');
    slot.append(canvas);
    const kids = [slot];
    if (spec.note) kids.push(h('div', 'note', spec.note));
    let n = 1;
    return {
      el: panel(spec.title, ...kids),
      update(model, canvases) {
        if (!canvases || (++n & 1)) return; // the map redraws at half the canvas rate (2.5 Hz): its trail only grows slowly
        const gnc = ctx.store.latest[APID.GNC], nav = ctx.store.latest[APID.NAV];
        ctx.map.draw(canvas, model.frame, ctx.store.latest[APID.CONFIG] || null,
          gnc ? { east: gnc.eastSetpoint, north: gnc.northSetpoint } : null, nav ? { east: nav.east, north: nav.north } : null);
      },
    };
  },

  gimbals(spec, ctx) {
    const grid = h('div', 'gimbals');
    const boxes = [];
    for (let i = 0; i < NUM_ENGINES; i++) {
      const box = h('div', 'gbox');
      const canvas = h('canvas');
      const val = h('div', 'gval');
      box.append(h('div', 'gname', 'ENGINE ' + (i + 1)), canvas, val);
      grid.append(box);
      boxes.push({ canvas, val, last: '' });
    }
    const legend = h('div', 'note');
    legend.innerHTML = '<span class="key dot"></span> measured <span class="key ring"></span> commanded <span class="amber-t">amber: feedback invalid</span>';
    return {
      el: panel(spec.title, grid, legend),
      update(model, canvases) {
        const gnc = ctx.store.latest[APID.GNC], fb = ctx.store.latest[APID.FEEDBACK];
        for (let i = 0; i < NUM_ENGINES; i++) {
          const cmd = gnc ? gnc.engines[i] : null, meas = fb ? fb.engines[i] : null, b = boxes[i];
          if (canvases) drawGimbalBox(b.canvas, cmd, meas);
          const html = gimbalText(cmd, meas);
          if (b.last !== html) { b.val.innerHTML = html; b.last = html; }
        }
      },
    };
  },

  attitude(spec) {
    const canvas = h('canvas');
    canvas.className = 'attitude';
    return { el: panel(spec.title, canvas), update(model, canvases) { if (canvases) drawAttitude(canvas, model.att); } };
  },

  timeline(spec, ctx) {
    const box = h('div', 'tl');
    const canvas = h('canvas');
    box.append(canvas);
    let lastSegs = -1;
    return {
      el: panel(spec.title, box),
      update(model, canvases) {
        if (canvases || ctx.timeline.segs.length !== lastSegs) { lastSegs = ctx.timeline.segs.length; drawTimeline(canvas, ctx.timeline.segs, ctx.store.met); }
      },
    };
  },

  note(spec) { return { el: h('div', 'note', spec.text) }; },

  chart(spec, ctx) {
    const cell = h('div', 'chart-cell');
    return { el: cell, mount() { ctx.charts.add(cell, spec); } };
  },

  stack(spec, ctx) {
    const el = h('div', 'stack');
    const kids = spec.children.map((c) => build(c, ctx));
    for (const k of kids) el.append(k.el);
    return compose(el, kids);
  },

  // Collapsible detail, closed by default; its contents are updated only while open.
  details(spec, ctx) {
    const el = h('details', 'more');
    el.append(h('summary', '', spec.title));
    const kids = spec.children.map((c) => build(c, ctx));
    for (const k of kids) el.append(k.el);
    const inner = compose(el, kids);
    let lastModel = null;
    el.addEventListener('toggle', () => { if (el.open && lastModel) inner.update(lastModel, true); });
    return { ...inner, update(model, canvases) { lastModel = model; if (el.open) inner.update(model, canvases); } };
  },

  // Per-packet table: wall-clock rate, received and lost counts, staleness.
  diagnostics(spec, ctx) {
    const t = h('table', 'kv diag');
    const head = h('tr');
    head.append(...['PACKET', 'APID', 'RATE /s', 'RECEIVED', 'LOST', 'LOSS %', 'STATE'].map((x) => h('th', '', x)));
    t.append(head);
    const rows = ctx.apids.map(([apid, name]) => {
      const tr = h('tr');
      const cells = [name, '0x' + Number(apid).toString(16).toUpperCase(), '--', '--', '--', '--', '--'].map((x) => h('td', '', x));
      tr.append(...cells);
      t.append(tr);
      return { apid, name, sets: cells.slice(2).map(textCell) };
    });
    return {
      el: panel(spec.title, t),
      update(model) {
        for (const r of rows) {
          const c = ctx.store.counts[r.apid];
          const rate = ctx.link.rates[r.apid];
          r.sets[0](Number.isFinite(rate) ? rate.toFixed(1) : '--');
          r.sets[1](c ? String(c.received) : '--');
          r.sets[2](c ? String(c.lost) : '--');
          r.sets[3](c && c.received + c.lost > 0 ? (100 * c.lost / (c.received + c.lost)).toFixed(1) : '--');
          const st = model.stale[r.name];
          r.sets[4](st === 'ok' ? 'OK' : st === 'stale' ? 'STALE' : 'NO DATA');
        }
      },
    };
  },
};

function build(spec, ctx) {
  const b = BUILDERS[spec.type];
  if (!b) throw new Error('unknown widget type: ' + spec.type);
  return b(spec, ctx);
}

// Combine child widgets: update, mount and destroy are forwarded to every child.
function compose(el, kids) {
  return {
    el,
    update(model, canvases) { for (const k of kids) if (k.update) k.update(model, canvases); },
    mount() { for (const k of kids) if (k.mount) k.mount(); },
  };
}

// Build the DOM of a screen into parent and return {update(model, canvases), destroy()}.
// Charts are created after insertion so they see their final width.
export function buildScreen(screen, ctx, parent) {
  const root = h('div', 'screen');
  root.dataset.screen = screen.id;
  const all = [];
  for (const rowSpec of screen.rows) {
    const rowEl = h('div', 'row');
    rowEl.style.gridTemplateColumns = rowSpec.map((w) => 'minmax(0,' + (w.w || 1) + 'fr)').join(' ');
    for (const w of rowSpec) { const k = build(w, ctx); rowEl.append(k.el); all.push(k); }
    root.append(rowEl);
  }
  parent.append(root);
  for (const c of root.querySelectorAll('canvas')) measure(c); // one layout read per build
  for (const k of all) if (k.mount) k.mount();
  const runtime = compose(root, all);
  return {
    update: runtime.update,
    destroy() { ctx.charts.destroy(); root.remove(); },
  };
}
