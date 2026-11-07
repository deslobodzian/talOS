// Registers a tiny module loader so the node test runner (tsx) can import CSS
// the way the browser bundler does: a `.css` import resolves to an empty ESM
// module instead of failing with ERR_UNKNOWN_FILE_EXTENSION.
//
// Use via `--import tests/css_loader.mjs` (see package.json's test script), or
// import it from tests/run.mjs before the test file is loaded. The hook itself
// lives in css_hooks.mjs so this entry can be re-imported without re-running
// register() on itself.

import {register} from 'node:module';

register('./css_hooks.mjs', import.meta.url);
