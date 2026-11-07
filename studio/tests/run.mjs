import {realpathSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import {register} from 'tsx/esm/api';
// Under Bazel this runs from a runfiles tree of symlinks while tsx resolves
// each source to its real path, so name the tsconfig by its real path too
// (studio/BUILD.bazel also turns off rules_js's fs patches for these tests).
// Otherwise the tsconfig `include`s none of the sources and tsx silently falls
// back to the classic JSX transform (React.createElement), which the
// automatic-runtime .tsx sources do not use.
register({
  tsconfig: realpathSync(fileURLToPath(new URL('../tsconfig.json', import.meta.url)))
});
// CSS imports are no-ops under node (the browser bundler handles them).
await import('./css_loader.mjs');
await import(`./${process.argv[2] ?? 'core.test.ts'}`);
