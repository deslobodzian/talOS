import {register} from 'tsx/esm/api';
register();
await import(`./${process.argv[2] ?? 'core.test.ts'}`);
