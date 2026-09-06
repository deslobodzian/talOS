import {register} from 'tsx/esm/api';
register();
const {startAgent} = await import('./server.ts');
const agent = startAgent();
process.once('SIGINT', agent.close);
process.once('SIGTERM', agent.close);
