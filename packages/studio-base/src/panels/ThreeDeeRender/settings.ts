// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { CameraState, DEFAULT_CAMERA_STATE } from "@foxglove/regl-worldview";
import { Topic } from "@foxglove/studio";
import { SettingsTreeNode } from "@foxglove/studio-base/components/SettingsTreeEditor/types";

const ONE_DEGREE = Math.PI / 180;

export type ThreeDeeRenderConfig = {
  cameraState: CameraState;
  followTf?: string;
};

export function buildSettingsTree(
  config: ThreeDeeRenderConfig,
  _topics: ReadonlyArray<Topic>,
): SettingsTreeNode {
  const { cameraState } = config;

  // prettier-ignore
  return {
    children: {
      cameraState: {
        label: "Camera",
        fields: {
          distance: { label: "Distance", input: "number", value: cameraState.distance, step: 1 },
          perspective: { label: "Perspective", input: "boolean", value: cameraState.perspective },
          phi: { label: "Phi", input: "number", value: cameraState.phi, step: ONE_DEGREE },
          // target: {}, // Needs input: "vector3"
          // targetOffset: {}, // Needs input: "vector3"
          // targetOrientation: {}, // Needs input: "quaternion"
          thetaOffset: { label: "Theta", input: "number", value: cameraState.thetaOffset, step: ONE_DEGREE },
          fovy: { label: "Y-Axis FOV", input: "number", value: cameraState.fovy, step: ONE_DEGREE },
          near: { label: "Near", input: "number", value: cameraState.near, step: DEFAULT_CAMERA_STATE.near },
          far: { label: "Far", input: "number", value: cameraState.far, step: 1 },
        },
      },
    },
  };
}
