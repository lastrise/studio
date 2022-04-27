// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/
//
// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright 2018-2021 Cruise LLC
//
//   This source code is licensed under the Apache License, Version 2.0,
//   found at http://www.apache.org/licenses/LICENSE-2.0
//   You may not use this file except in compliance with the License.

// All message types supported by Rviz
// http://wiki.ros.org/rviz/DisplayTypes

import { Time } from "@foxglove/rostime";

export type Namespace = Readonly<{
  topic: string;
  name: string;
}>;

export type MutablePoint = {
  x: number;
  y: number;
  z: number;
};
export type Point = Readonly<MutablePoint>;
type Points = readonly Point[];

export type MutablePoint2D = {
  x: number;
  y: number;
};
export type Point2D = Readonly<MutablePoint2D>;

export type Header = Readonly<{
  frame_id: string;
  stamp: Time;
  seq: number;
}>;

export type StampedMessage = Readonly<{
  header: Header;
}>;

export type GeometryMsgs$PoseArray = Readonly<{
  header: Header;
  poses: Pose[];
}>;

export type GeometryMsgs$PolygonStamped = Readonly<{
  header: Header;
  polygon: Polygon;
}>;

type Duration = Time;

type MutableOrientation = {
  x: number;
  y: number;
  z: number;
  w: number;
};
export type Orientation = Readonly<MutableOrientation>;

export type Scale = Readonly<{
  x: number;
  y: number;
  z: number;
}>;

export type Color = Readonly<{
  r: number;
  g: number;
  b: number;
  a: number;
}>;

export type Pose = Readonly<{
  position: Point;
  orientation: Orientation;
}>;

// NOTE: Deep mutability.
export type MutablePose = {
  position: MutablePoint;
  orientation: MutableOrientation;
};

export type Polygon = Readonly<{
  points: Points;
}>;

export type FloatArray = ReadonlyArray<number> | Readonly<Float32Array> | Readonly<Float64Array>;

export type LaserScan = Readonly<{
  header: Header;
  angle_increment: number;
  angle_max: number;
  angle_min: number;
  intensities: FloatArray;
  range_max: number;
  range_min: number;
  ranges: FloatArray;
  scan_time?: number;
  time_increment?: number;
}>;

export type PoseStamped = Readonly<
  StampedMessage & {
    pose: Pose;
  }
>;

type Colors = readonly Color[];

// Markers
export type BaseMarker = Readonly<
  StampedMessage & {
    ns: string;
    id: string | number; // TODO: Actually just a number
    action: 0 | 1 | 2 | 3;
    pose: MutablePose;
    scale: Scale;
    color?: Color;
    colors?: Colors;
    lifetime?: Time;
    frame_locked: boolean;
    points?: Point[];
    text?: string;
    mesh_resource?: string; // TODO: required
    mesh_use_embedded_materials?: boolean;
    primitive?: string;
    metadata?: Readonly<Record<string, unknown>>;
  }
>;

type MultiPointMarker = Readonly<{
  points: Points;
  colors?: Colors;
}>;

type ArrowSize = Readonly<{
  shaftLength?: number;
  shaftWidth: number;
  headLength: number;
  headWidth: number;
}>;

export type ArrowMarker = Readonly<
  BaseMarker & {
    type: 0;
    points?: Points;
    // used for hard-coded arrows with geometry_msgs/PoseStamped
    // not part of the original ros message
    size?: ArrowSize;
  }
>;

export type CubeMarker = Readonly<
  BaseMarker & {
    type: 1;
  }
>;

export type SphereMarker = Readonly<
  BaseMarker & {
    type: 2;
  }
>;

export type CylinderMarker = Readonly<
  BaseMarker & {
    type: 3;
  }
>;

export type LineStripMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      closed?: boolean;
      type: 4;
    }
>;

export type LineListMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      type: 5;
    }
>;

export type CubeListMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      type: 6;
    }
>;

export type SphereListMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      type: 7;
    }
>;

export type PointsMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      type: 8;
    }
>;

export type TextMarker = Readonly<
  BaseMarker & {
    type: 9;
    text: string;
  }
>;

export type GlLineListMarker = Readonly<{
  color: Float32Array;
  points: Float32Array;
}>;

export type MeshMarker = Readonly<
  BaseMarker & {
    type: 10;
    mesh_resource: string;
    mesh_use_embedded_materials: boolean;
  }
>;

type NavMsgs$MapMetaData = Readonly<{
  map_load_time: Time;
  resolution: number;
  width: number;
  height: number;
  origin: Pose;
}>;

export type NavMsgs$OccupancyGrid = Readonly<{
  header: Header;
  info: NavMsgs$MapMetaData;
  data: ReadonlyArray<number> | Readonly<Int8Array>;
}>;

export type NavMsgs$Path = Readonly<{
  header: Header;
  poses: PoseStamped[];
}>;

export type OccupancyGridMessage = Readonly<{
  header: Header;
  name: string;
  type: 101;
  alpha?: number;
  info: NavMsgs$MapMetaData;
  pose: MutablePose;
  data: ReadonlyArray<number> | Readonly<Int8Array>;
}>;

export type TriangleListMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      type: 11;
    }
>;

export type InstancedLineListMarker = Readonly<
  BaseMarker &
    MultiPointMarker & {
      type: 108;
      metadataByIndex?: readonly Readonly<unknown[]>[];
      scaleInvariant?: boolean;
    }
>;

export type ColorMarker = Readonly<
  BaseMarker & {
    type: 110;
  }
>;

export type Marker =
  | ArrowMarker
  | CubeMarker
  | CubeListMarker
  | SphereMarker
  | SphereListMarker
  | CylinderMarker
  | LineStripMarker
  | LineListMarker
  | CubeListMarker
  | PointsMarker
  | TextMarker
  | MeshMarker
  | TriangleListMarker
  | MeshMarker
  | InstancedLineListMarker
  | ColorMarker;

export type MarkerArray = Readonly<{
  markers: readonly Marker[];
}>;

export type PointField = Readonly<{
  name: string;
  offset: number;
  datatype: number;
  count: number;
}>;

export type PointCloud2 = StampedMessage & {
  fields: readonly PointField[];
  height: number;
  width: number;
  is_bigendian: boolean;
  point_step: number; // Length of point in bytes
  row_step: number; // Length of row in bytes
  data: Uint8Array;
  is_dense: boolean | number;
  // this is appended by scene builder
  type: 102;
  // this is appended by scene builder
  pose?: MutablePose;
};

export type Image = Readonly<
  StampedMessage & {
    height: number;
    width: number;
    encoding: string;
    is_bigendian: boolean;
    step: number;
    data: Uint8Array;
  }
>;

export type CompressedImage = Readonly<
  StampedMessage & {
    format: string;
    data: Uint8Array;
  }
>;

export type VelodynePacket = Readonly<{
  stamp: Time;
  data: Uint8Array; // 1206 bytes
}>;

export type VelodyneScan = Readonly<
  StampedMessage & {
    packets: VelodynePacket[];
  }
>;

export type VelodyneScanDecoded = Readonly<
  PointCloud2 & {
    packets: VelodynePacket[];
  }
>;

export type PointCloud = PointCloud2 | VelodyneScanDecoded;

type Transform = Readonly<{
  rotation: Orientation;
  translation: Point;
}>;

export type TF = Readonly<
  StampedMessage & {
    transform: Transform;
    child_frame_id: string;
  }
>;

export enum ImageMarkerType {
  CIRCLE = 0,
  LINE_STRIP = 1,
  LINE_LIST = 2,
  POLYGON = 3,
  POINTS = 4,
  // TEXT is not part of visualization_msgs/ImageMarker, but we include it to
  // support existing frameworks that have extended this message definition
  TEXT = 5,
}

export enum ImageMarkerAction {
  ADD = 0,
  REMOVE = 1,
}

export type ImageMarker = Readonly<{
  header: Header;
  ns: string;
  id: number;
  type: ImageMarkerType;
  action: ImageMarkerAction;
  position: Point;
  scale: number;
  outline_color: Color;
  filled: boolean;
  fill_color: Color;
  lifetime: Duration;
  points: Points;
  outline_colors: Colors;
  // `text` is not part of visualization_msgs/ImageMarker, but we include it to
  // support existing frameworks that have extended this message definition
  text?: { data: string };
}>;

export type ImageMarkerArray = Readonly<{
  markers: ImageMarker[];
}>;

type Roi = Readonly<{
  x_offset: number;
  y_offset: number;
  height: number;
  width: number;
  do_rectify: false;
}>;

// Empty string indicates no distortion model
export type DistortionModel = "plumb_bob" | "rational_polynomial" | "";

export type CameraInfo = Readonly<{
  width: number;
  height: number;
  binning_x: number;
  binning_y: number;
  roi: Roi;
  distortion_model: DistortionModel;
  D: FloatArray;
  K: FloatArray;
  P: FloatArray;
  R: FloatArray;
}>;

export type JointState = Readonly<{
  header: Header;
  name: string[];
  position: FloatArray;
  velocity: FloatArray;
  effort: FloatArray;
}>;


//добавление дд сообщений

export type AntiInstagramThresholds = {
  low: number[];
  high: number[];
};

export type AprilTagDetection = {
  tag_id: number;
  tag_family:string;
  hamming: number;
  decision_margin: number;
  homography: number[];
  center: number[];
  corners: number[];
  pose_error: number;
};

export type AprilTagDetectionArray = {
  header: Header;
  detections: AprilTagDetection[];
};

export type AprilTagsWithInfos = {
  header: Header;
  detections: AprilTagDetection[];
  infos: TagInfo[];
};

export type BoolStamped = {
  header: Header;
  data: boolean;
};

export type ButtonEvent = {
 EVENT_SINGLE_CLICK: 0;
 EVENT_HELD_3SEC: 10;
 EVENT_HELD_10SEC: 20;
 event: number;
};

export type CarControl = {
 header: Header;
 speed: number;
 steering: number;
 need_steering: boolean;
};

export type CoordinationClearance = {
  header: Header;
  status: number;
  NA: -1;
  WAIT: 0;
  GO: 1;
};

export type CoordinationSignal = {
   signal: string;

  //these must match with LED_protocol.yaml
  //OFF: light_off;
  //ON: light_on;
  //ON: traffic_light_go;
  //SIGNAL_A: CAR_SIGNAL_A;
  //SIGNAL_B: CAR_SIGNAL_B;
  //SIGNAL_C: CAR_SIGNAL_C;
  //SIGNAL_GREEN: CAR_SIGNAL_GREEN;
  //SIGNAL_PRIORITY: CAR_SIGNAL_PRIORITY;
  //SIGNAL_SACRIFICE_FOR_PRIORITY: CAR_SIGNAL_SACRIFICE_FOR_PRIORITY;
  
  //TL_GO_ALL: tl_go_all;
  //TL_STOP_ALL: tl_stop_all;
  //TL_GO_N: tl_go_N;
  //TL_GO_S: tl_go_S;
  //TL_GO_W: tl_go_W;
  //TL_GO_E: tl_go_E;
  //TL_YIELD: tl_yield;
};

export type DiagnosticsCodeProfiling = {
  node: string;                            // Node publishing this message
  block: string;                             // Name of the profiled code block
  frequency: number;                        // Execution frequency of the block
  duration: number;                         // Last execution time of the block (in seconds)
  filename: string;                          // Filename in which this block resides
  line_nums: number[];                    // Start and end line of the block in the file
  time_since_last_execution: number;      // Seconds since last execution
};

export type DiagnosticsCodeProfilingArray = {
  header: Header;
   blocks: DiagnosticsCodeProfiling[];
};

export type DiagnosticsRosLink = {
  // Link direction
  LINK_DIRECTION_INBOUND: 0;
  LINK_DIRECTION_OUTBOUND: 1;
  
  node: string;         // Node publishing this message
  topic: string;         // Topic transferred over the link
  remote: string;        // Remote end of this link
  direction: number;      // Link direction
  connected: boolean    // Status of the link
  transport:string;    // Type of transport used for this link
  messages: number;    // Number of messages transferred over this link
  dropped: number;     // Number of messages dropped over this link
  bytes: number;      // Volume of data transferred over this link
  frequency: number;   // Link frequency (Hz)
  bandwidth: number;   // Link bandwidth (byte/s)
};

export type DiagnosticsRosLinkArray = {
 header: Header;
 links: DiagnosticsRosLink[];
};


export type DiagnosticsRosNode = {
  // Node type (this has to match duckietown.NodeType)
  NODE_TYPE_GENERIC: 0;
  NODE_TYPE_DRIVER: 1;
  NODE_TYPE_PERCEPTION: 2;
  NODE_TYPE_CONTROL: 3;
  NODE_TYPE_PLANNING: 4;
  NODE_TYPE_LOCALIZATION: 5;
  NODE_TYPE_MAPPING: 6;
  NODE_TYPE_SWARM: 7;
  NODE_TYPE_BEHAVIOR: 8;
  NODE_TYPE_VISUALIZATION: 9;
  NODE_TYPE_INFRASTRUCTURE: 10;
  NODE_TYPE_COMMUNICATION: 11;
  NODE_TYPE_DIAGNOSTICS: 12;
  NODE_TYPE_DEBUG: 20;
  
 // Node health (this has to match duckietown.NodeHealth)
  NODE_HEALTH_UNKNOWN: 0;
  NODE_HEALTH_STARTING: 5;
  NODE_HEALTH_STARTED: 6;
  NODE_HEALTH_HEALTHY: 10;
  NODE_HEALTH_WARNING: 20;
  NODE_HEALTH_ERROR: 30;
  NODE_HEALTH_FATAL: 40;
  
  header: Header;
  name: string;             // Node publishing this message
  help: string;             // Node description
  type: number;              // Node type (see NODE_TYPE_X above)
  health: number;            // Node health (see NODE_HEALTH_X above)
  health_reason: string;    // String describing the reason for this health status (if any)
  health_stamp: number;    // Time when the health status changed into the current
  enabled: boolean;            // Status of the switch
  uri: string;              // RPC URI of the node
  machine: string;          // Machine hostname or IP where this node is running
  module_type: string;      // Module containing this node
  module_instance: string;  // ID of the instance of the module running this node
 };

 export type DiagnosticsRosParameterArray = {
  header: Header;
  params: NodeParameter[];    // List of parameters
 };

 export type DiagnosticsRosProfiling = {
  header: Header;
  units: DiagnosticsRosProfilingUnit[];    // List of profiling units
 };

 export type DiagnosticsRosProfilingUnit = {
  // Link direction
  LINK_DIRECTION_INBOUND: 0;
  LINK_DIRECTION_OUTBOUND: 1;
  
  node: string;        // Node publishing this message
  name: string;         // Name of the profiled unit
  time: number;        // Execution time of the unit
 };

 export type DiagnosticsRosTopic = {
  // Topic direction (this has to match duckietown.TopicDirection)
 TOPIC_DIRECTION_INBOUND: 0;
 TOPIC_DIRECTION_OUTBOUND: 1;

// Topic type (this has to match duckietown.TopicType)
 TOPIC_TYPE_GENERIC: 0;
 TOPIC_TYPE_DRIVER: 1;
 TOPIC_TYPE_PERCEPTION: 2;
 TOPIC_TYPE_CONTROL: 3;
 TOPIC_TYPE_PLANNING: 4;
 TOPIC_TYPE_LOCALIZATION: 5;
 TOPIC_TYPE_MAPPING: 6;
 TOPIC_TYPE_SWARM: 7;
 TOPIC_TYPE_BEHAVIOR: 8;
 TOPIC_TYPE_VISUALIZATION: 9;
 TOPIC_TYPE_INFRASTRUCTURE: 10;
 TOPIC_TYPE_COMMUNICATION: 11;
 TOPIC_TYPE_DIAGNOSTICS: 12;
 TOPIC_TYPE_DEBUG: 20;

node: string;                     // Node publishing this message
name: string;                      // Topic object of the diagnostics
help: string;                      // Topic description
type: number;                       // Topic type
direction: number;                 // Topic direction
frequency: number;              // Topic frequency (Hz)
effective_frequency: number;     // Topic (effective) frequency (Hz)
healthy_frequency: number;       // Frequency at which this topic can be considered healthy
bandwidth: number;               // Topic bandwidth (byte/s)
enabled: boolean;                    // Topic switch
};
 
export type DiagnosticsRosTopicArray = {
  header: Header;
  topics: DiagnosticsRosTopic[];
 };

 export type DisplayFragment = {
  header: Header;

  // Enum: region
  REGION_FULL: 0;
  REGION_HEADER: 1;
  REGION_BODY: 2;
  REGION_FOOTER: 3;
  
  // Enum: page
  PAGE_ALL: 255;
  
  // fragment ID and destination page and region
  id: string;
  region: number;
  page: number;
  
  // fragment content
  data: Image;
  
  // location on the display where to show the fragment
  //location: RegionOfInterest;
  
  // Z index in the Z-buffer of the segment
  z: number;
  
  // Time-to-Live in seconds of the fragment (-1 for infinite, do not abuse)
  ttl: number;
 };

 export type DroneControl = {
  //Roll Pitch Yaw(rate) Throttle Commands, simulating output from
  //remote control. Values range from 1000 to 2000
  //which corespond to values from 0% to 100%
  
  roll: number;
  pitch: number;
  yaw: number;
  throttle: number;
 };

 export type DroneMode = {
  // Power supply status constants
  DRONE_MODE_DISARMED: 0;
  DRONE_MODE_ARMED: 1;
  DRONE_MODE_FLYING: 2;
  
  // The drone status  as reported. Values defined above
  drone_mode: number;
 };

 export type DuckieSensor = {
  // Sensors send value and type messages
  // For analog sensors value = 0..4095 and fvalue = 0.0..1.0
  // For digital sensors value= 0..1 and fvalue = 0.0
  value: number;
  fvalue: number;
  is_analog: boolean;
  name: string;
 };

 export type DuckiebotLED = {
  header: Header;
  //colors: sColorRGBA[];
 };

 export type EncoderStamped = {
  header: Header;
  vel_encoder: number;
  count: number;
 };

 export type EpisodeStart = {
  header: Header;
  episode_name: string;
  other_payload_yaml: string;
 };

 export type FSMState = {
  header: Header;
  state: string;
  
  // pseudo constants
  LANE_FOLLOWING: "LANE_FOLLOWING";
  INTERSECTION_COORDINATION: "INTERSECTION_COORDINATION";
  INTERSECTION_CONTROL: "INTERSECTION_CONTROL";
  NORMAL_JOYSTICK_CONTROL: "NORMAL_JOYSTICK_CONTROL";
  SAFE_JOYSTICK_CONTROL: "SAFE_JOYSTICK_CONTROL";
  PARKING: "PARKING";
  ARRIVE_AT_STOP_LINE: "ARRIVE_AT_STOP_LINE";
  LANE_RECOVERY: "LANE_RECOVERY";
  INTERSECTION_RECOVERY: "INTERSECTION_RECOVERY";
  CALIBRATING: "CALIBRATING";
  CALIBRATING_CALC: "CALIBRATING_CALC";
  
  //List of states
  // LANE_FOLLOWING
  // INTERSECTION_COORDINATION
  // INTERSECTION_CONTROL
  // NORMAL_JOYSTICK_CONTROL
  // SAFE_JOYSTICK_CONTROL
  // PARKING
  // ARRIVE_AT_STOP_LINE
  // LANE_RECOVERY
  // INTERSECTION_RECOVERY
  // CALIBRATING
  // CALIBRATING_CALC
 };

 export type IntersectionPose = {
  header: Header;
  x: number;
  y: number;
  theta: number;
  type: number;
  likelihood: number;
 };

 export type IntersectionPoseImg = {
  header: Header;
  x: number;
  y: number;
  theta: number;
  type: number;
  likelihood: number;
  img: CompressedImage;
 };

 export type IntersectionPoseImgDebug = {
  header: Header;
  x: number;
  y: number;
  theta: number;
  type: number;
  likelihood: number;
  x_init: number;
  y_init: number;
  theta_init: number;
  img: CompressedImage;
 };

 export type KinematicsParameters = {
  parameters: number[];
 };

 export type KinematicsWeights = {
  weights: number[];
 };

 export type LEDDetection = {
  //timestamp1: time;		// initial timestamp of the camera stream used 
  //timestamp2: time;		// final timestamp of the camera stream used 
  pixels_normalized: Vector2D;
  frequency: number; 
  color: string;        // will be r, g or b 
  confidence: number; // some value of confidence for the detection (TBD)
  
  // for debug/visualization
  signal_ts: number[];
  signal: number[];
  fft_fs: number[];
  fft: number[];
 };

 export type LEDDetectionArray = {
  detections: LEDDetection[];
 };

 export type LEDDetectionDebugInfo = {
  state: number; //0: idle, 1: capturing, 2: processing
  capture_progress: number;
  
  cell_size: number[];
  crop_rect_norm: number[];
  
  variance_map: CompressedImage;
  candidates: Vector2D[];
  
  led_all_unfiltered: LEDDetectionArray;
 };

 export type LEDInterpreter = {
  header: Header;
  vehicle: boolean;
  trafficlight: boolean;
 };

 export type LEDPattern = {
  header: Header;
  color_list: string[];
  //rgb_vals: ColorRGBA[];
  color_mask: number[];
  frequency: number;
  frequency_mask: number[];
 };

 export type LanePose = {
  header: Header;
  d: number;   //lateral offset
  d_ref: number; //lateral offset reference
  phi: number; //heading error
  phi_ref: number; //heading error reference
  d_phi_covariance: number[];
  curvature: number;
  curvature_ref: number; // Refernece Curvature
  v_ref: number; // Referenece Velocity
  status: number; //Status of duckietbot 0 if normal, 1 if error is encountered
  in_lane: boolean; //Status of duckietbot in lane
  
  //Enum for status
  NORMAL: 0;
  ERROR: 1;
 };

 export type LightSensor = {
  header: Header;
  r: number;
  g: number;
  b: number;
  c: number;
  real_lux: number;
  lux: number;
  temp: number;
 };

 export type LineFollowerStamped = {
  header: Header;
  valid: boolean; // True iff the ADC reading was valid
  // All of the following values are normalized line brightness, between 0 and 1.
  // Specifically, an ADC voltage of 0 is mapped to 0, and 3.3V is mapped to 1.0.
  outer_right: number;
  inner_right: number;
  inner_left: number;
  outer_left: number;
 };

 export type MaintenanceState = {
  //header: Header;
  state: string;
  
  // pseudo constants
  WAY_TO_MAINTENANCE: "WAY_TO_MAINTENANCE";
  WAY_TO_CHARGING: "WAY_TO_CHARGING";
  CHARGING: "CHARGING";
  WAY_TO_CALIBRATING: "WAY_TO_CALIBRATING";
  CALIBRATING: "CALIBRATING";
  WAY_TO_CITY: "WAY_TO_CITY";
  NONE: "NONE";
 };

 export type NodeParameter = {
  // Parameter type (this has to match duckietown.TopicType)
  PARAM_TYPE_UNKNOWN: 0;
  PARAM_TYPE_STRING: 1;
  PARAM_TYPE_INT: 2;
  PARAM_TYPE_FLOAT: 3;
  PARAM_TYPE_BOOL: 4;
  
  node: string;         // Name of the node
  name: string;         // Name of the parameter (fully resolved)
  help: string;         // Description of the parameter
  type: number;          // Type of the parameter (see PARAM_TYPE_X above)
  min_value: number;   // Min value (for type INT, UINT, and FLOAT)
  max_value: number;   // Max value (for type INT, UINT, and FLOAT)
  editable: boolean;       // Editable (it means that the node will be notified for changes)
 };

 export type ObstacleImageDetection = {
  bounding_box: Rect;
  type: ObstacleType;
  };

 export type ObstacleImageDetectionList = {
  header: Header;
  list: ObstacleImageDetection[];
  imwidth: number;
  imheight: number;
  };
  
  export type ObstacleProjectedDetection = {
    location: Point;
    type: ObstacleType;
    distance: number;
  };

  export type ObstacleProjectedDetectionList = {
    header: Header;
    list: ObstacleProjectedDetection[];
  };

  export type ObstacleType = {
    DUCKIE: 0;
    CONE: 1;
    type: number;
  };

  export type ParamTuner = {
    header: Header;
    cross_track_err: number;
    cross_track_integral: number;
    diff_cross_track_err: number;
    heading_err: number;
    heading_integral: number;
    diff_heading_err: number;
    dt: number;
  };

  export type Pixel = {
    u: number;
    v: number;
  };

  export type Pose2DStamped = {
    header: Header;
    x: number;
    y: number;
    theta: number;
  };

  export type Rect = {
    // all in pixel coordinate
    // (x, y, w, h) defines a rectangle
    x: number;
    y: number;
    w: number;
    h: number;
  };

  export type Rects = {
    rects: Rect[];
  };

  export type SceneSegments = {
    segimage: Image;
    rects: Rect[];
  };

  export type Segment = {
    WHITE: 0;
    YELLOW: 1;	
    RED: 2;
    color: number;
    pixels_normalized: Vector2D[];
    normal: Vector2D;
    
    points: Point[];
  };

  export type SegmentList = {
    header: Header;  
    segments: Segment[];
  };

  export type SignalsDetection = {
    header: Header;  

    // this is what we can see at the intersection:
    front: string;
    right: string;
    left: string;
    
    // For the first backoff approach
    // led_detected
    // no_led_detected
    
    // Each of these can be:
    NO_CAR: 'no_car_detected';
    SIGNAL_A: 'car_signal_A';
    SIGNAL_B: 'car_signal_B';
    SIGNAL_C: 'car_signal_C';
    SIGNAL_PRIORITY: 'car_signal_priority';
    SIGNAL_SACRIFICE_FOR_PRIORITY: 'car_signal_sacrifice_for_priority';
    
    NO_CARS: 'no_cars_detected';
    CARS   : 'cars_detected';
    
    
    // Plus we can see the traffic light
    
    // for the moment we assume that no traffic light exists
    
    traffic_light_state: string;
    
    NO_TRAFFIC_LIGHT: 'no_traffic_light';
    STOP: 'tl_stop';
    GO: 'tl_go';
    YIELD: 'tl_yield';
  };

  export type SignalsDetectionETHZ17 = {
    header: Header;  

    // this is what we can see at the intersection:
    //front: string;
    //right: string;
    //left : string;
    
    // For the first backoff approach
    led_detected: string;
    no_led_detected: string;
    
    // Each of these can be:
    //NO_CAR: 'no_car_detected';
    SIGNAL_A: 'car_signal_A';
    SIGNAL_B: 'car_signal_B';
    SIGNAL_C: 'car_signal_C';
    
    NO_CARS: 'no_cars_detected';
    CARS   : 'cars_detected';
    
    
    // Plus we can see the traffic light
    
    // for the moment we assume that no traffic light exists
    
    //traffic_light_state
    
    //NO_TRAFFIC_LIGHT: 'no_traffic_light';
    //STOP: 'tl_stop';
    GO: 'tl_go';
    //YIELD: 'tl_yield';
  };

  export type SourceTargetNodes = {
    source_node: string;
    target_node: string;
  };

  export type StopLineReading = {
    header: Header;
    stop_line_detected: boolean;
    at_stop_line: boolean;
    stop_line_point: Point; //this is in the "lane frame"
  };

  export type StreetNameDetection = {
    //Mirrors TagDetection.h in the apriltags pkg
    good: boolean;
    id: number;
    p: number[];
    cxy: number[];
    observedPerimeter: number;
    homography: number[];
    orientation: number;
    hxy: number[];
    transform: Transform;
    text: string;
  };

  export type StreetNames = {
    header: Header;
    detections: StreetNameDetection[];
  };

  export type TagInfo = {
    header: Header;
    id: number;
    
    //(StreetName, TrafficSign, Localization, Vehicle)
    tag_type: number;
    
    S_NAME: 0;
    SIGN: 1;	
    LIGHT: 2;
    LOCALIZE: 3;
    VEHICLE: 4;
    
   street_name: string;
    
    traffic_sign_type: number;
    // (12 possible traffic sign types)
    
    STOP: 5;
    YIELD: 6;
    NO_RIGHT_TURN: 7;
    NO_LEFT_TURN: 8;
    ONEWAY_RIGHT: 9;
    ONEWAY_LEFT: 10;
    FOUR_WAY: 11;
    RIGHT_T_INTERSECT: 12;
    LEFT_T_INTERSECT: 13;
    T_INTERSECTION: 14;
    DO_NOT_ENTER: 15;
    PEDESTRIAN: 16;
    T_LIGHT_AHEAD: 17;
    DUCK_CROSSING: 18;
    PARKING: 19;
    
   vehicle_name: string;
    
    // Just added a single number for location. Probably want to use Vector2D.msg, but I get errors when I try to add it.
    location: number;
  };
  
  export type ThetaDotSample = {
    d_L: number;
    d_R: number;
    dt: number;
    theta_angle_pose_delta: number;
  };

  export type Trajectory = {
    header: Header;
    //pos: Vector3Stamped[];
    //vel: Vector3Stamped[];
    //acc: Vector3Stamped[];
    //jerk: Vector3Stamped[];
  };

  export type TurnIDandType = {
    tag_id: number;
    turn_type: number;
  };

  export type Twist2DStamped = {
    header: Header;
    v: number;
    omega: number;
  };

  export type Vector2D = {
    x: number;
    y: number;
  };

  export type VehicleCorners = {
    header: Header;
    //corners: Point32[];
    //detection: Bool;
    H: number;
    W: number;
  };

  export type VehiclePose = {
    header: Header;
    //rho: Float32;
    //theta: Float32;
    //psi: Float32;
    //detection: Bool;
  };

  export type Vsample = {
    d_L: number;
    d_R: number;
    dt: number;
    theta_angle_pose_delta: number;
    x_axis_pose_delta: number;
    y_axis_pose_delta: number;
  };

  export type WheelEncoderStamped = {
    // Enum: encoder type
    ENCODER_TYPE_ABSOLUTE: 0;
    ENCODER_TYPE_INCREMENTAL: 1;
    
    header: Header;
    data: number;
    resolution: number;
    type: number;
  };

  export type WheelsCmd = {
    vel_left: number;
    vel_right: number;
  };

  export type WheelsCmdDBV2Stamped = {
    header: Header;
    gamma: number;           //"vel_left" changed to "gamma", RFMH_2019_02_26
    vel_wheel: number;       //"vel_right" changed to "vel_wheel", RFMH_2019_02_26
    trim: number;            //included "trim" to be accessible in the wheels_driver_node as well, RFMH_2019_04_01
  };

  export type WheelsCmdStamped = {
    header: Header;
    vel_left: number;
    vel_right: number;
   };