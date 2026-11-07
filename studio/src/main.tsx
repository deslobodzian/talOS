// Studio entry point. Thin on purpose: create the data layer (worker-backed
// store) once, wrap the shell in the StudioProvider so every panel and the
// shell read the same hooks, and hand the rest to the shell.

import {createRoot} from 'react-dom/client';

import {createWorkerStudio, StudioProvider} from './data/studio';
import {Shell} from './shell/Shell';
import './style.css';

const studio = createWorkerStudio();

createRoot(document.getElementById('root')!).render(
    <StudioProvider studio={studio}>
      <Shell/>
    </StudioProvider>);
