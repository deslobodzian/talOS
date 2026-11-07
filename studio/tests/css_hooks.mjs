// The `load` hook that makes CSS imports no-ops under node. Only `.css` URLs
// are intercepted; everything else falls through to the next loader (tsx's
// TypeScript transform, then the default).

export async function load(url, context, nextLoad) {
  if (url.endsWith('.css')) {
    return {format: 'module', source: '', shortCircuit: true};
  }
  return nextLoad(url, context);
}
