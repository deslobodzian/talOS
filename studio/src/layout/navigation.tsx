// Panel-to-shell navigation. A panel never imports the shell (that would be a
// cycle through the registry); instead the shell provides a small navigation
// context and panels consume it. The default is a no-op, so a panel renders
// fine standalone -- including under react-dom/server in tests.

import {createContext, useContext} from 'react';

export type PanelNavigation = {
  // Ensure a panel of this type is visible on the active page, adding it if
  // necessary.
  openPanel(panelId: string): void;
};

const PanelNavigationContext = createContext<PanelNavigation>({
  openPanel: () => {}
});

export const PanelNavigationProvider = PanelNavigationContext.Provider;

export function usePanelNavigation(): PanelNavigation {
  return useContext(PanelNavigationContext);
}
