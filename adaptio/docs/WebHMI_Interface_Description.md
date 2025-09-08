# WebHMI Interface Description

## Introduction

The WebHMI is a supplementary user interface to the main HMI. It provides control and observability for Adaptio specific functions. It is currently used for calibration and specification of joint parameters and also provides observability for slides status and Adaptio version.

In a production setting, the WebHMI is accessed using a browser at <https://adaptio.local>. The frontend application accessed by the  browser in turn connects to the Adaptio application via websocket using the "WebHMI Interface"

In an automated test environment, the WebHMI interface can be accessed by sending/receiving messages via websocket on port 8080 on adaptio.local. For test purposes such messages can be used to access additional "service mode" functions to start joint tracking and control axis positions.

One tool for sending/receiving messages via websocket is wscat.
In the adaptio-vue-service-frontend repo it can be started with:
$ nix develop
$ npx wscat -c ws://adaptio.local:8080

The WebHMI Interface is a set of json messages distinguished by the "name" attribute in the message.

## AdaptioVersion

**Message:** GetAdaptioVersion \
**Direction:** In \
**Description:** Request version

**Example:**

```json
{"name":"GetAdaptioVersion","payload":{}}
```

**Message:** GetLaserToTorchCalibrationRsp \
**Direction:** Out \
**Description:** Adaptio version

**Example:**

```json
{"name":"GetAdaptioVersionRsp","payload":{"version":"0.1.8"}}
```

## JointGeometry

**Message:** SetJointGeometry \
**Direction:** In \
**Description:** Set and store joint geometry

**Example:**

```json
{
  "name":"SetJointGeometry",
  "payload":{
    "groove_depth_mm":120.0,
    "left_joint_angle_rad":0.139,
    "left_max_surface_angle_rad":0.35,
    "right_joint_angle_rad":0.139,
    "right_max_surface_angle_rad":0.35,
    "upper_joint_width_mm":42.0
  }
}
```

**Message:** GetJointGeometry \
**Direction:** In \
**Description:** Request stored joint geometry

**Example:**

```json
{"name":"GetJointGeometry","payload":{}}
```

**Message:** GetJointGeometryRsp \
**Direction:** Out \
**Description:** Stored joint geometry

**Example:**

```json
{
  "name":"GetJointGeometryRsp",
  "payload":{
    "groove_depth_mm":120.0,
    "left_joint_angle_rad":0.139,
    "left_max_surface_angle_rad":0.35,
    "right_joint_angle_rad":0.139,
    "right_max_surface_angle_rad":0.35,
    "upper_joint_width_mm":42.0
  }
}
```

## LaserToTorchCalibration

The precondition for a successful calibration sequence is that the scanner can detect a joint which matches the geometry of a "calibration fixture" as specified in the configuration file "calibration_fixture_joint_geometry.yaml".

For test purposes it is possible to use a simulated camera which reads images from a file. This is used in the system test.

A picture explaining the calibration setup, including the fixture, is found in the frontend.

**Message:** LaserToTorchCalibration \
**Direction:** In \
**Description:** Request a calibration measurement and calculation, no state change or storage

**Example:**

```json
{"name":"LaserToTorchCalibration","payload":{"offset":200,"angle":0.26,"stickout":30}}
```

**Message:** LaserToTorchCalibrationRsp \
**Direction:** Out \
**Description:** The "valid" attribute indicates if the calibration was successful (valid=true) or not (valid=false).

**Example:**

```json
{"name":"LaserToTorchCalibrationRsp","payload":{"angle":0.26,"valid":true,"x":6.9,"y":200.1,"z":-29.5}}
```

**Message:** SetLaserToTorchCalibration \
**Direction:** In \
**Description:** Set and store calibration data in the application

**Example:**

```json
{"name":"SetLaserToTorchCalibration","payload":{"angle":0.26,"x":6.9,"y":200.1,"z":-29.5}}
```

**Message:** SetLaserToTorchCalibrationRsp \
**Direction:** Out \
**Description:** The "result" attribute indicates if the set operation was successful (result=true) or not (result=false).

**Example:**

```json
{"name":"SetLaserToTorchCalibrationRsp","payload":{"result":true}}
```

**Message:** GetLaserToTorchCalibration \
**Direction:** In \
**Description:** Request stored calibration data

**Example:**

```json
{"name":"GetLaserToTorchCalibration","payload":{}}
```

**Message:** GetLaserToTorchCalibrationRsp \
**Direction:** Out \
**Description:** Response with stored calibration data

**Example:**

```json
{"name":"GetLaserToTorchCalibrationRsp","payload":{"angle":0.26,"valid":true,"x":6.9,"y":200.1,"z":-29.5}}
```

## WeldObjectCalibration

The precondition for a successful calibration sequence is that the scanner can detect a joint which matches the weld object geometry as specified in the configuration file "joint_geometry.yaml".

For test purposes it is possible to use a simulated camera which reads images from a file. This is used in the system test.

**Message:** WeldObjectCalibration \
**Direction:** In \
**Description:** Request a calibration measurement and calculation, no state change or storage

**Example:**

```json
{"name":"WeldObjectCalibration","payload":{"radius":8000,"stickout":30}}
```

**Message:** WeldObjectCalibrationRsp \
**Direction:** Out \
**Description:** The "valid" attribute indicates if the calibration was successful (valid=true) or not (valid=false).

**Example:**

```json
{"name":"WeldObjectCalibrationRsp","payload":{"valid":true,"y":-8000,"z":-14.7}}
```

**Message:** SetWeldObjectCalibration \
**Direction:** In \
**Description:** Set and store calibration data in the application

**Example:**

```json
{"name":"SetWeldObjectCalibration","payload":{"y":-8000,"z":-14.7}}
```

**Message:** SetWeldObjectCalibrationRsp \
**Direction:** Out \
**Description:** The "result" attribute indicates if the operation was successful (result=true) or not (result=false).

**Example:**

```json
{"name":"SetWeldObjectCalibrationRsp","payload":{"result":true}}
```

**Message:** GetWeldObjectCalibration \
**Direction:** In \
**Description:** Request stored calibration data

**Example:**

```json
{"name":"GetWeldObjectCalibration","payload":{}}
```

**Message:** GetWeldObjectCalibrationRsp \
**Direction:** Out \
**Description:** Response with stored calibration data

**Example:**

```json
{"name":"GetWeldObjectCalibrationRsp","payload":{"valid":true,"y":-8000,"z":-14.7}}
```

## ActivityStatus

Activity status can have a value according to:

```cpp
enum class ActivityStatusE : uint32_t {
  IDLE                          = 0,
  LASER_TORCH_CALIBRATION       = 1,
  WELD_OBJECT_CALIBRATION       = 2,
  TRACKING                      = 3,
  SERVICE_MODE_TRACKING         = 4,
  SERVICE_MODE_KINEMATICS       = 5,
  SERVICE_MODE_IMAGE_COLLECTION = 6
};
```

**Message:** GetActivityStatus \
**Direction:** In \
**Description:** Request current activity status

**Example:**

```json
{"name":"GetActivityStatus","payload":{}}
```

**Message:** GetActivityStatusRsp \
**Direction:** Out \
**Description:** Current activity status

**Example:**

```json
{"name":"GetActivityStatusRsp","payload":{"value":0}}
```

## SlidesStatus

Slides can be in the position requested by Adaptio or not.

**Message:** GetSlidesStatus \
**Direction:** In \
**Description:** Request current slides status

**Example:**

```json
{"name":"GetSlidesStatus","payload":{}}
```

**Message:** GetSlidesStatusRsp \
**Direction:** Out \
**Description:** Current slides status \

**Example:**

```json
{"name":"GetSlidesStatusRsp","payload":{"horizontal_in_position":false,"vertical_in_position":false}}
```

## SlidesPosition

Horizontal and vertical position (in mm)

**Message:** GetSlidesPosition \
**Direction:** In \
**Description:** Request current slides position

**Example:**

```json
{"name":"GetSlidesPosition","payload":{}}
```

**Message:** GetSlidesPositionRsp \
**Direction:** Out \
**Description:** Current slides position

**Example:**

```json
{"name":"GetSlidesPositionRsp","payload":{"horizontal":5.0,"vertical":10.0}}
```

## ServiceModeTracking

Joint tracking can be started as a service mode function. This is mainly for test purposes.

**Message:** ServiceModeTracking \
**Direction:** In \
**Description:** Request to enter service mode tracking

**Example:**

```json
{"name":"ServiceModeTracking","payload":{}}
```

**Message:** StartTracking \
**Direction:** In \
**Description:** Request to start service mode tracking

**Example:**

```json
{"name":"StartTracking","payload":{"horizontal_offset":5.0, "vertical_offset":10.0}}
```

## ServiceModeKinematicsControl

Slides position can be set using a service mode function. This is mainly for test purposes.

**Message:** ServiceModeKinematicsControl \
**Direction:** In \
**Description:** Request to enter service mode for kinematics control

**Example:**

```json
{"name":"ServiceModeKinematicsControl","payload":{}}
```

**Message:** SetSlidesPosition \
**Direction:** In \
**Description:** Request to set slides position

**Example:**

```json
{"name":"SetSlidesPosition","payload":{"horizontal":30.0, "vertical":30.0}}
```

## Automatic Bead Placement (ABP) Parameters

**Message:** StoreABPParameters \
**Direction:** In \
**Description:** Set and store ABP Parameters

**Example:**

```json
{
  "name": "StoreABPParameters",
  "payload": {
    "beadOverlap": 42.0,
    "beadSwitchAngle": 15.0,
    "capBeads": 3,
    "capCornerOffset": 1.5,
    "capInitDepth": 7.0,
    "heatInput": {
      "max": 2.9,
      "min": 2.1
    },
    "kGain": 2.0,
    "stepUpLimits": [
      25.0,
      30.0,
      35.0,
      40.0,
      50.0
    ],
    "stepUpValue": 99.1,
    "wallOffset": 3.7,
    "weldSpeed": {
      "max": 105.0,
      "min": 95.0
    },
    "weldSystem2Current": {
      "max": 700.0,
      "min": 650.0
    }
  }
}
```

**Message:** StoreABPParametersRsp \
**Direction:** Out \
**Description:** Result of storage request

**Example:**

```json
{"name":"StoreABPParametersRsp","payload":{"result":"ok"}}
```

**Message:** GetABPParameters \
**Direction:** In \
**Description:** Request stored ABP Parameters

**Example:**

```json
{"name":"GetABPParameters","payload":{}}
```

**Message:** GetABPParametersRsp \
**Direction:** Out \
**Description:** Stored ABP Parameters

**Example:**

```json
{
  "name": "GetABPParametersRsp",
  "payload": {
    "beadOverlap": 42.0,
    "beadSwitchAngle": 14.999999999999998,
    "capBeads": 3,
    "capCornerOffset": 1.5,
    "capInitDepth": 7.0,
    "firstBeadPosition": "left",
    "heatInput": {
      "max": 2.9,
      "min": 2.1
    },
    "kGain": 2.0,
    "stepUpLimits": [
      25.0,
      30.0,
      35.0,
      40.0,
      50.0
    ],
    "stepUpValue": 99.1,
    "wallOffset": 3.7,
    "weldSpeed": {
      "max": 105.0,
      "min": 95.0
    },
    "weldSystem2Current": {
      "max": 700.0,
      "min": 650.0
    }
  }
}
```

## Adaptio Settings

**Message:** SetSettings \
**Direction:** In \
**Description:** Set adaptio settings

**Example:**

```json
{
  "name": "SetSettings",
  "payload": {
    "useEdgeSensor": true,
  }
}
```

**Message:** SetSettingsRsp \
**Direction:** Out \
**Description:** Set adaptio settings response

**Example:**

```json
{
  "name": "SetSettingsRsp",
  "payload": {
    "result": "ok",
  }
}
```

**Message:** GetSettings \
**Direction:** In \
**Description:** Get adaptio settings
**Example:**

```json
{
  "name": "GetSettings",
  "payload": {}
}
```

**Message:** GetSettingsRsp \
**Direction:** Out \
**Description:** Get adaptio settings response

**Example:**

```json
{
  "name": "GetSettingsRsp",
  "payload": {
    "useEdgeSensor": true,
  }
}
```
