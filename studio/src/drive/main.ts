// The driver dashboard page (drive.html, served at /drive): loads the layout
// from /drive/layout.json, holds one /drive feed socket through DriveModel
// and redraws every RENDER_MS. Everything it loads comes from the bridge that
// served it; it keeps nothing in browser storage.

import './drive.css';

import {DriveModel, type DriveSocket, RECONNECT_MS, RENDER_MS} from './feed';
import {type Layout, parseLayout} from './layout';
import {esc, renderDashboard, renderStatus, renderWidgets} from './view';

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
                               tracked(layout));
  model.clock = now;
  root.innerHTML = renderDashboard(layout, model, now());
  const status = root.querySelector('.status-slot')!;
  const grid = root.querySelector<HTMLElement>('.grid')!;
  let shownStatus = '', shownGrid = '';
  const render = () => {
    const t = now();
    model.tick(t);
    const nextStatus = renderStatus(model, t), nextGrid = renderWidgets(layout, model, t);
    if (nextStatus !== shownStatus) status.innerHTML = shownStatus = nextStatus;
    if (nextGrid !== shownGrid) grid.innerHTML = shownGrid = nextGrid;
  };
  root.addEventListener('click', event => {
    if ((event.target as Element).closest('[data-action="reconnect"]')) {
      model.reconnect(now());
      render();
    }
  });
  model.start(now());
  render();
  setInterval(() => {
    try {
      render();
    } catch (error) {
      // A drawing fault must not stop the page: say so and keep going.
      status.textContent = `Dashboard drawing error: ${String(error)}`;
    }
  }, RENDER_MS);
}

void run();
