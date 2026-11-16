// Two pages from one build: Studio (index.html, served at /) and the driver
// dashboard (drive.html, served at /drive). Their scripts and styles are
// emitted as assets/<name>-<hash>.<ext>, which the bridge serves as
// immutable (studio/bridge/README.md "HTTP and WebSocket views").
//
// The inputs are resolved against the working directory, Vite's root (both
// `pnpm build` and //studio:web_dist run from studio/), not against this
// file's real path: under Bazel the sandbox reaches it through a symlink,
// and a page outside the root cannot be emitted.
import {resolve} from 'node:path';

export default {
  build: {
    rollupOptions: {
      input: {
        main: resolve('index.html'),
        drive: resolve('drive.html'),
      },
    },
  },
};
