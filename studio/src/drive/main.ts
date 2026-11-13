// The driver dashboard page (drive.html, served at /drive): loads the layout
// from /drive/layout.json, holds one /drive feed socket through DriveModel
// and redraws every RENDER_MS and at each freshness deadline (DrawSchedule).
// Everything it loads comes from the bridge that served it; it keeps nothing
// in browser storage.

import './drive.css';

import {DriveModel, type DriveSocket, RECONNECT_MS, RENDER_MS} from './feed';
import {type Layout, parseLayout} from './layout';
import {DrawSchedule, esc, METRICS, outOfView, outOfViewText, Painter, renderShell, renderStatus, renderWidget, type Surface} from './view';

const root = document.getElementById('drive')!;
const now = () => performance.now();

function tracked(layout: Layout): string[] {
  const names: string[] = [];
  for (const w of layout.widgets) {
    if (w.kind === 'graph') names.push(...w.series);
    if (w.kind === 'field') names.push(w.channels.x, w.channels.y);
  }
  return names;
}

async function loadLayout(): Promise<Layout> {
  for (;;) {
    try {
      const response = await fetch('/drive/layout.json', {cache: 'no-store'});
      if (response.ok) return parseLayout(await response.json());
      root.innerHTML = `<p class="loading">No dashboard layout from the bridge (HTTP ${response.status}); ` +
          'set [studio] dashboard in robot.toml. Retrying.</p>';
    } catch (error) {
      root.innerHTML = `<p class="loading">Cannot load the dashboard layout: ${esc(String(error))}. Retrying.</p>`;
    }
    await new Promise(resolve => setTimeout(resolve, RECONNECT_MS));
  }
}

async function run() {
  const layout = await loadLayout();
  document.title = `${layout.title} · talOS`;
  const scheme = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const model = new DriveModel(`${scheme}//${location.host}/drive`,
                               // A WebSocket's handlers take an Event the model does not read.
                               url => new WebSocket(url) as unknown as DriveSocket,
                               tracked(layout), layout);
  model.clock = now;
  root.innerHTML = renderShell(layout);
  const status = root.querySelector('.status-slot')!;
  const grid = root.querySelector<HTMLElement>('.grid')!;
  const body = (e: Element) => e.querySelector<HTMLElement>('.body');
  const surface: Surface<Element> = {
    status: html => {
      status.innerHTML = html;
    },
    create: html => {
      const template = document.createElement('template');
      template.innerHTML = html;
      return template.content.firstElementChild!;
    },
    put: (_index, next, previous) => {
      if (previous) grid.replaceChild(next, previous);
      else grid.appendChild(next);
    },
    scrollTop: e => body(e)?.scrollTop ?? 0,
    setScrollTop: (e, top) => {
      const b = body(e);
      if (b) b.scrollTop = Math.max(0, Math.min(top, b.scrollHeight - b.clientHeight));
    },
    errored: on => {
      grid.classList.toggle('errored', on);
    }
  };
  // Each list widget's heading says how many of its rows are out of view,
  // measured here where the rows are laid out (outOfView does the counting).
  const countOutOfView = () => {
    for (const list of grid.querySelectorAll('.w')) {
      const badge = list.querySelector('.out-of-view'), b = body(list);
      if (!badge || !b) continue;
      let text = '';
      if (b.scrollHeight > b.clientHeight) {
        const origin = b.getBoundingClientRect().top - b.scrollTop;
        const tops = [...b.querySelectorAll('li')].map(li => li.getBoundingClientRect().top - origin);
        text = outOfViewText(outOfView(tops, METRICS.row, b.clientHeight, b.scrollTop));
      }
      if (badge.textContent !== text) badge.textContent = text;
    }
  };
  grid.addEventListener('scroll', countOutOfView, {capture: true, passive: true});
  window.addEventListener('resize', countOutOfView);
  // A drawing fault must not stop the page or leave the grid looking
  // current: Painter reports it, and the grid is marked until a good draw.
  const painter = new Painter(surface);
  const schedule = new DrawSchedule(model, t => {
    painter.paint(() => {
      model.tick(t);
      return {status: renderStatus(model, t, layout.skipped),
              widgets: layout.widgets.map(w => renderWidget(w, model, t))};
    });
    countOutOfView();
  }, {
    now,
    wall: () => Date.now(),
    setTimeout: (fn, ms) => window.setTimeout(fn, ms),
    clearTimeout: handle => window.clearTimeout(handle)
  });
  root.addEventListener('click', event => {
    if ((event.target as Element).closest('[data-action="reconnect"]')) {
      model.reconnect(now());
      schedule.frame();
    }
  });
  // A background tab's timers run at most once a second: on its return the
  // page draws at once rather than show a frame up to 1 s old as current.
  document.addEventListener('visibilitychange', () => schedule.frame());
  model.start(now());
  schedule.frame();
  setInterval(() => schedule.frame(), RENDER_MS);
}

void run();
