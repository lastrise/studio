// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { PropsWithChildren, useMemo } from "react";

import { AppSetting } from "@foxglove/studio-base/AppSetting";
import Panel from "@foxglove/studio-base/components/Panel";
import PanelToolbar from "@foxglove/studio-base/components/PanelToolbar";
import { useExtensionRegistry } from "@foxglove/studio-base/context/ExtensionRegistryContext";
import PanelCatalogContext, {
  PanelCatalog,
  PanelInfo,
} from "@foxglove/studio-base/context/PanelCatalogContext";
import { useAppConfigurationValue } from "@foxglove/studio-base/hooks/useAppConfigurationValue";
import panels from "@foxglove/studio-base/panels";

export default function PanelCatalogProvider(
  props: PropsWithChildren<unknown>,
): React.ReactElement {
  const [showDebugPanels = false] = useAppConfigurationValue<boolean>(AppSetting.SHOW_DEBUG_PANELS);
  const extensionRegistry = useExtensionRegistry();

  const wrappedExtensionPanels = useMemo<PanelInfo[]>(() => {
    const extensionPanels = extensionRegistry.getRegisteredPanels();

    return extensionPanels.map((panel) => {
      const panelType = `${panel.extensionName}.${panel.registration.name}`;
      const PanelComponent = panel.registration.component;
      const PanelWrapper = () => {
        return (
          <>
            <PanelToolbar floating />
            <PanelComponent />
          </>
        );
      };
      PanelWrapper.panelType = panelType;
      PanelWrapper.defaultConfig = {};
      PanelWrapper.supportsStrictMode = true;
      return {
        category: "misc",
        title: panel.registration.name,
        component: Panel(PanelWrapper),
      };
    });
  }, [extensionRegistry]);

  const allPanels = useMemo(() => {
    return [...panels.builtin, ...panels.hidden, ...panels.debug, ...wrappedExtensionPanels];
  }, [wrappedExtensionPanels]);

  const visiblePanels = useMemo(() => {
    // debug panels are hidden by default, users can enable them within app settings
    if (showDebugPanels) {
      return [...panels.builtin, ...panels.debug, ...wrappedExtensionPanels];
    }

    return [...panels.builtin, ...wrappedExtensionPanels];
  }, [showDebugPanels, wrappedExtensionPanels]);

  const panelsByType = useMemo(() => {
    const byType = new Map<string, PanelInfo>();

    for (const panel of allPanels) {
      const type = panel.component.panelType;
      byType.set(type, panel);
    }
    return byType;
  }, [allPanels]);

  const provider = useMemo<PanelCatalog>(() => {
    return {
      getPanels() {
        return visiblePanels;
      },
      getPanelByType(type: string) {
        return panelsByType.get(type);
      },
    };
  }, [panelsByType, visiblePanels]);

  return (
    <PanelCatalogContext.Provider value={provider}>{props.children}</PanelCatalogContext.Provider>
  );
}