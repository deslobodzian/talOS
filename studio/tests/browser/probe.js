// The driver dashboard's checks, run inside the page (drive_browser_test.py
// evaluates this file once, then calls window.__probe). Everything here reads
// the live DOM and the page's own clock (performance.now); a failure is an
// object naming the element by a CSS path, with its outerHTML and geometry.
(() => {
  const EPS = 0.5;      // px: a box this far past an edge is past it
  // px: text wider than its box by more is cut. Layout widths are whole
  // 1/64 px units, so this is below one unit: a word whose box is one unit
  // narrower than its text has shrunk, whether or not the ellipsis then
  // shows (a state word was once ellipsized for 0.03 px).
  const TEXT_EPS = 0.001;
  const MIN_FONT_PX = 12;
  const LISTS = '.w-subsystems, .w-checks, .w-alerts';
  // A widget showing one of these shows no value.
  const PLACEHOLDER_TAGS = new Set(['NO DATA', 'NO POSE', 'unknown']);

  // Non-intrusive hooks installed before drive.html loads:
  // 1) A monotonic wall-clock offset so sleep-detection and visibilitychange
  //    tests can advance Date.now() relative to performance.now() (zero by
  //    default, leaving Date.now() unchanged for every other check).
  // 2) Observation of DrawSchedule's freshness-deadline setTimeout (<= 350 ms)
  //    so the stale-transition check knows the exact STALE_MS + 1 deadline
  //    armed from the last new telemetry tick.
  const realDateNow = Date.now.bind(Date);
  let wallOffsetMs = 0;
  Date.now = () => realDateNow() + wallOffsetMs;
  const shiftWallClock = deltaMs => { wallOffsetMs += deltaMs; };

  const realSetTimeout = window.setTimeout.bind(window);
  const realClearTimeout = window.clearTimeout.bind(window);
  const trackedTimers = new Map();
  let lastFiredTimer = null;
  let lastClearedTimer = null;
  window.setTimeout = function(fn, ms, ...args) {
    const delay = Number(ms);
    const track = typeof fn === 'function' && Number.isFinite(delay) && delay > 0 && delay <= 350;
    const armedAt = track ? performance.now() : 0;
    let handle;
    const wrapped = track ? function(...cbArgs) {
      const entry = trackedTimers.get(handle);
      trackedTimers.delete(handle);
      if (entry) lastFiredTimer = {...entry, firedAt: performance.now()};
      return fn.apply(this, cbArgs);
    } : fn;
    handle = realSetTimeout(wrapped, ms, ...args);
    if (track) trackedTimers.set(handle, {handle, armedAt, delay, targetAt: armedAt + delay});
    return handle;
  };
  window.clearTimeout = function(handle) {
    const entry = trackedTimers.get(handle);
    if (entry) {
      trackedTimers.delete(handle);
      lastClearedTimer = {...entry, clearedAt: performance.now()};
    }
    return realClearTimeout(handle);
  };

  const box = r => ({left: +r.left.toFixed(3), top: +r.top.toFixed(3),
                     right: +r.right.toFixed(3), bottom: +r.bottom.toFixed(3)});

  function path(el) {
    const parts = [];
    for (let e = el; e && e.nodeType === 1 && e.id !== 'drive'; e = e.parentElement) {
      let part = e.tagName.toLowerCase();
      if (e.classList.length) part += '.' + [...e.classList].join('.');
      const parent = e.parentElement;
      if (parent && [...parent.children].filter(c => c.tagName === e.tagName).length > 1)
        part += `:nth-of-type(${[...parent.children].filter(c => c.tagName === e.tagName).indexOf(e) + 1})`;
      parts.unshift(part);
    }
    return parts.join(' > ');
  }

  function failure(check, el, detail, extra = {}) {
    return {check, element: path(el), detail, text: (el.textContent || '').slice(0, 200),
            rect: box(el.getBoundingClientRect()), outerHTML: el.outerHTML.slice(0, 4000), ...extra};
  }

  // The box an element's overflow clips its content to (its padding box).
  function clipBox(el) {
    const r = el.getBoundingClientRect();
    const left = r.left + el.clientLeft, top = r.top + el.clientTop;
    return {left, top, right: left + el.clientWidth, bottom: top + el.clientHeight};
  }

  const clips = el => {
    const s = getComputedStyle(el);
    return s.overflowX !== 'visible' || s.overflowY !== 'visible';
  };
  const isListBody = el => el.classList.contains('body') && el.parentElement?.matches(LISTS);
  const outside = (r, c) => r.left < c.left - EPS || r.top < c.top - EPS ||
      r.right > c.right + EPS || r.bottom > c.bottom + EPS;
  const own = el => [...el.childNodes].some(n => n.nodeType === 3 && n.textContent.trim());

  // The width the element's text lays out at, and the width its content box
  // gives it: text wider than its box is cut (ellipsized or clipped) even
  // where scrollWidth, an integer, rounds the difference away.
  function textFit(el) {
    const range = document.createRange();
    range.selectNodeContents(el);
    const text = range.getBoundingClientRect().width;
    const s = getComputedStyle(el);
    const content = el.getBoundingClientRect().width - parseFloat(s.paddingLeft) -
        parseFloat(s.paddingRight) - parseFloat(s.borderLeftWidth) - parseFloat(s.borderRightWidth);
    return {text, content};
  }

  // How many rows of a list body are out of view, from the rows' boxes.
  function outOfView(body) {
    const c = clipBox(body);
    let above = 0, below = 0;
    for (const li of body.querySelectorAll('li')) {
      const r = li.getBoundingClientRect();
      if (r.bottom > c.bottom + EPS && r.top >= c.top - EPS) below++;
      else if (r.top < c.top - EPS) above++;
    }
    return {above, below};
  }
  const words = n => [n.above ? `${n.above} above` : '', n.below ? `+${n.below} below` : '']
      .filter(Boolean).join(' · ');

  function listState(kind) {
    const w = document.querySelector(`.w-${kind}`);
    const body = w.querySelector('.body');
    const rows = [...body.querySelectorAll('li')];
    return {scrollTop: body.scrollTop, scrollHeight: body.scrollHeight, clientHeight: body.clientHeight,
            badge: w.querySelector('.out-of-view')?.textContent ?? null,
            measured: words(outOfView(body)), rows: rows.map(li => li.textContent)};
  }

  // State words, tags and headings under `root`: never cut, not even by a
  // fraction of a pixel.
  function textCut(root) {
    const failures = [];
    for (const el of root.querySelectorAll('.tag, .level, .mode, .out-of-view, .w h2, .status b')) {
      if (!el.textContent.trim()) continue;
      const {text, content} = textFit(el);
      if (el.scrollWidth > el.clientWidth || text > content + TEXT_EPS)
        failures.push(failure('text cut', el,
                              `text ${text.toFixed(3)}px in a ${content.toFixed(3)}px box (scrollWidth ${el.scrollWidth}, clientWidth ${el.clientWidth})`));
    }
    return failures;
  }

  // List widgets: every visible row whole, the heading's out-of-view count
  // what the rows' boxes show, and (with `wordsToo`) the heading and every word
  // in the list whole.
  function listChecks(page, wordsToo = true) {
    const failures = [];
    for (const w of page.querySelectorAll(LISTS)) {
      const body = w.querySelector('.body');
      const c = clipBox(body);
      for (const li of body.querySelectorAll('li')) {
        const r = li.getBoundingClientRect();
        const visible = r.bottom > c.top + EPS && r.top < c.bottom - EPS;
        if (visible && outside(r, c))
          failures.push(failure('list row cut', li, `row ${JSON.stringify(box(r))} in a list showing ${JSON.stringify(box(c))}`));
      }
      const h2 = w.querySelector('h2');
      if (!h2 || outside(h2.getBoundingClientRect(), clipBox(w)))
        failures.push(failure('list heading hidden', h2 ?? w, 'heading outside its widget'));
      const badge = w.querySelector('.out-of-view');
      const expected = body.scrollHeight > body.clientHeight ? words(outOfView(body)) : '';
      if ((badge?.textContent ?? '') !== expected)
        failures.push(failure('out-of-view count wrong', badge ?? w,
                              `heading says "${badge?.textContent ?? ''}", rows out of view: "${expected}"`));
      if (wordsToo) failures.push(...textCut(w));
    }
    return failures;
  }

  // What REQ-OPS-49 asks of the page as it is now laid out: `scope` 'page'
  // for everything (the viewports it names); 'lists' for the list widgets
  // only (a smaller viewport, where only lists are held to it); 'rows' for
  // their rows and out-of-view counts alone.
  function audit(scope = 'page') {
    const page = document.querySelector('#drive .page');
    if (!page) return [{check: 'page', element: '#drive', detail: 'no dashboard drawn', rect: null,
                        outerHTML: document.body.outerHTML.slice(0, 4000)}];
    if (scope !== 'page') return listChecks(page, scope === 'lists');
    const failures = [];
    const view = {left: 0, top: 0, right: innerWidth, bottom: innerHeight};
    const doc = document.scrollingElement;
    if (doc.scrollHeight > innerHeight || doc.scrollWidth > innerWidth)
      failures.push(failure('page scrolls', doc,
                            `document ${doc.scrollWidth}x${doc.scrollHeight} in a ${innerWidth}x${innerHeight} viewport`));

    for (const el of page.querySelectorAll('*')) {
      if (el.closest('svg') && el.tagName.toLowerCase() !== 'svg') continue;
      const r = el.getBoundingClientRect();
      if (r.width === 0 && r.height === 0) continue;
      // Rows a list body scrolls out of view are the list checks' business.
      let clipped = null, scrolled = false;
      for (let a = el.parentElement; a && a !== page.parentElement; a = a.parentElement) {
        if (isListBody(a)) { scrolled = true; break; }
        if (clips(a) && outside(r, clipBox(a))) { clipped = a; break; }
      }
      if (scrolled) continue;
      if (outside(r, view))
        failures.push(failure('beyond the viewport', el, `outside ${innerWidth}x${innerHeight}`));
      else if (clipped)
        failures.push(failure('hidden by an ancestor\'s overflow', el,
                              `cut by ${path(clipped)} ${JSON.stringify(box(clipBox(clipped)))}`));
      if (own(el)) {
        const size = parseFloat(getComputedStyle(el).fontSize);
        if (size < MIN_FONT_PX) failures.push(failure('text under 12 px', el, `font-size ${size}px`));
      }
    }
    failures.push(...textCut(page.querySelector('.top')));
    for (const w of page.querySelectorAll('.w'))
      if (!w.matches(LISTS)) failures.push(...textCut(w));

    // Every widget shows a value, and none reads STALE.
    for (const w of page.querySelectorAll('.w')) {
      if (w.classList.contains('stale') || w.querySelector('.tag-stale'))
        failures.push(failure('widget stale', w, 'marked STALE'));
      for (const absent of w.querySelectorAll('.absent'))
        failures.push(failure('widget without a value', absent, absent.textContent));
      for (const t of w.querySelectorAll('.tag, .mode'))
        if (PLACEHOLDER_TAGS.has(t.textContent.trim()))
          failures.push(failure('widget without a value', t, t.textContent));
    }
    failures.push(...listChecks(page));
    // That subsystem rows whose names differ only past the cell's width
    // (front_left_camera, front_right_camera) still read differently is
    // studio/tests/drive.test.ts's "a 2026 subsystems row keeps a readable
    // name beside its widest state word": this subject has no such rows.
    return failures;
  }

  const status = () => {
    const s = document.querySelector('.status');
    return s ? {link: [...s.classList].find(c => c.startsWith('link-')) ?? '', text: s.textContent} : null;
  };
  const telemetryWidgets = () => [...document.querySelectorAll('.w')].filter(w => !w.matches('.w-subsystems'));
  const current = () => status()?.link === 'link-ok' && !document.querySelector('.w.stale');
  // The session is up: every subsystems row reads UP.
  const allUp = () => {
    const rows = [...document.querySelectorAll('.w-subsystems li')];
    return rows.length > 0 && rows.every(li => li.querySelector('.tag-good')) && rows.length;
  };
  const valued = () => current() && !document.querySelector('.w .absent') &&
      ![...document.querySelectorAll('.w .tag, .w .mode')].some(t => PLACEHOLDER_TAGS.has(t.textContent.trim()));

  function headerState(kind) {
    const w = document.querySelector(`.w-${kind}`);
    const header = w.querySelector(':scope > header');
    const c = clipBox(header);
    const measure = el => {
      if (!el) return null;
      const fit = textFit(el);
      const r = el.getBoundingClientRect();
      return {
        text: el.textContent,
        rect: box(r),
        scrollWidth: el.scrollWidth,
        clientWidth: el.clientWidth,
        textWidth: +fit.text.toFixed(3),
        contentWidth: +fit.content.toFixed(3),
        textOverflow: getComputedStyle(el).textOverflow,
        insideHeader: !outside(r, c),
      };
    };
    return {
      header: box(c),
      h2: measure(header.querySelector('h2')),
      meta: measure(header.querySelector('.meta')),
      badge: measure(header.querySelector('.out-of-view')),
    };
  }

  function staleStyle() {
    return telemetryWidgets().map(w => {
      const body = w.querySelector('.body');
      const big = w.querySelector('.big');
      return {
        kind: [...w.classList].find(c => c.startsWith('w-')) ?? '',
        staleClass: w.classList.contains('stale'),
        tag: w.querySelector('.tag-stale')?.textContent ?? null,
        opacity: body ? parseFloat(getComputedStyle(body).opacity) : null,
        lineThrough: big ? getComputedStyle(big).textDecorationLine.includes('line-through') : null,
        absentCount: w.querySelectorAll('.absent').length,
      };
    });
  }

  // Resolves with `predicate()`'s first truthy value, re-checked on every
  // change to the page; rejects after `boundMs`, which only bounds a hang.
  function waitFor(predicate, boundMs, what) {
    return new Promise((resolve, reject) => {
      const first = predicate();
      if (first) return resolve(first);
      const observer = new MutationObserver(() => {
        const value = predicate();
        if (value) {
          observer.disconnect();
          realClearTimeout(timer);
          resolve(value);
        }
      });
      observer.observe(document, {subtree: true, childList: true, attributes: true,
                                                  characterData: true});
      const timer = realSetTimeout(() => {
        observer.disconnect();
        reject(new Error(`${what}: not within ${boundMs} ms; status ${JSON.stringify(status())}`));
      }, boundMs);
    });
  }

  // Two frames: whatever a viewport change or a scroll set off is laid out
  // and its handlers have run.
  const frames = () => new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));

  // Arms the watch for staleness and returns the page's clock: the caller
  // stops the producer after this returns, so the producer's last tick is no
  // earlier than this instant less one producer period.
  function armStale(boundMs) {
    lastFiredTimer = null;
    lastClearedTimer = null;
    const armedAt = performance.now();
    window.__stale = (async () => {
      const staleAt = await waitFor(() => telemetryWidgets().every(w => w.classList.contains('stale')) &&
                                        performance.now(), boundMs, 'telemetry widgets stale');
      const deadlineTimer = lastFiredTimer ?? lastClearedTimer;
      const styles = staleStyle();
      const named = await waitFor(() => status()?.link === 'link-telemetry' && status(), boundMs,
                                  'status names telemetry');
      return {armedAt, staleAt, status: named, namedAt: performance.now(),
              widgets: telemetryWidgets().length, deadlineTimer,
              firedByDeadlineTimer: Boolean(lastFiredTimer), styles};
    })();
    return armedAt;
  }

  // Arms listeners around main.ts's document `visibilitychange` handler:
  // when the tab hides and then returns to `visible`, the window capture
  // listener runs before main.ts (recording pre-draw state and shifting the
  // wall clock by `shiftMs`), main.ts runs `schedule.frame()` synchronously,
  // and the document target listener registered here runs immediately after
  // inside the same event dispatch (before any timer or WebSocket callback).
  function armVisibilityReturn(shiftMs, boundMs) {
    let sawHidden = false;
    let before = null;
    let settled = false;
    window.__visHidden = new Promise((resolve, reject) => {
      if (document.visibilityState === 'hidden') return resolve(true);
      const onHide = () => {
        if (document.visibilityState === 'hidden') {
          document.removeEventListener('visibilitychange', onHide);
          realClearTimeout(hideTimer);
          resolve(true);
        }
      };
      document.addEventListener('visibilitychange', onHide);
      const hideTimer = realSetTimeout(() => {
        document.removeEventListener('visibilitychange', onHide);
        reject(new Error(`tab did not become hidden within ${boundMs} ms (state=${document.visibilityState})`));
      }, boundMs);
    });
    window.__visReturn = new Promise((resolve, reject) => {
      const cleanup = () => {
        window.removeEventListener('visibilitychange', onCapture, true);
        document.removeEventListener('visibilitychange', onAfter, false);
        realClearTimeout(timer);
      };
      const onCapture = () => {
        if (document.visibilityState === 'hidden') {
          sawHidden = true;
          return;
        }
        if (sawHidden && document.visibilityState === 'visible' && !settled) {
          before = {
            status: status(),
            staleCount: document.querySelectorAll('.w.stale').length,
            totalWidgets: document.querySelectorAll('.w').length,
          };
          shiftWallClock(shiftMs);
        }
      };
      const onAfter = () => {
        if (sawHidden && document.visibilityState === 'visible' && !settled) {
          settled = true;
          const after = {
            status: status(),
            staleCount: document.querySelectorAll('.w.stale').length,
            totalWidgets: document.querySelectorAll('.w').length,
          };
          cleanup();
          resolve({sawHidden, before, after});
        }
      };
      window.addEventListener('visibilitychange', onCapture, true);
      document.addEventListener('visibilitychange', onAfter, false);
      const timer = realSetTimeout(() => {
        if (!settled) {
          settled = true;
          cleanup();
          reject(new Error(`visibilitychange return: not within ${boundMs} ms (sawHidden=${sawHidden}, state=${document.visibilityState})`));
        }
      }, boundMs);
    });
    return true;
  }

  window.__probe = {audit, listState, headerState, staleStyle, waitFor, frames,
                    current, valued, allUp, status, armStale, shiftWallClock,
                    armVisibilityReturn};
})();
