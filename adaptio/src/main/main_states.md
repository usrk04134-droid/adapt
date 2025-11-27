# Main Application State Machines

This document describes the state machines and state coordination in the main Adaptio application, including `ManagementServer`, `WeldControl`, `ActivityStatus`, and `CalibrationStatus`.

## Overview

The main application coordinates multiple state machines that work together:

1. **ActivityStatus**: Tracks current Adaptio activity (IDLE, TRACKING(this includes ABP), calibration operations)
2. **WeldControl**: Manages weld control modes (IDLE, JOINT_TRACKING, AUTOMATIC_BEAD_PLACEMENT) and states (IDLE, WELDING)
3. **ManagementServer ReadyState**: Determines what operations are ready based on certain conditions
4. **CalibrationStatus**: Provides validity of weld object calibration

## ActivityStatus State Machine

```plantuml

title Activity Status State Machine

skinparam backgroundColor #DCE8F7
skinparam state {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
    StartColor #000000
    EndColor #000000
}

[*] --> IDLE : Initial

state IDLE
state LASER_TORCH_CALIBRATION
state WELD_OBJECT_CALIBRATION {
    state CALIBRATION_AUTO_MOVE
    [*] --> CALIBRATION_AUTO_MOVE
}
state TRACKING

IDLE --> TRACKING : JointTrackingStart
IDLE --> LASER_TORCH_CALIBRATION : Laser torch calibration started
IDLE --> WELD_OBJECT_CALIBRATION : Weld object calibration started

TRACKING --> IDLE : Stop

LASER_TORCH_CALIBRATION --> IDLE : Calibration complete
WELD_OBJECT_CALIBRATION --> IDLE : Calibration complete

```

### ActivityStatus States

#### IDLE

- **Description**: System is idle, no active operations
- **Set by**: `StopActiveFunction()` in ManagementServer, calibration completion

#### TRACKING

- **Description**: Joint tracking is active
- **Set by**: `OnTrackingStart()` in ManagementServer when joint tracking begins
- **Note**: This is the only activity status initiated from PLC

#### LASER_TORCH_CALIBRATION

- **Description**: Laser torch calibration is in progress
- **Set by**: Calibration manager when laser torch calibration starts

#### WELD_OBJECT_CALIBRATION

- **Description**: Weld object calibration is in progress
- **Set by**: Calibration manager when weld object calibration starts

#### CALIBRATION_AUTO_MOVE

- **Description**: Calibration sequence is performing automatic slide movement
- **Set by**: `CalibrationSequenceRunner` when calibration sequence starts
- **Note**: This status causes `ReadyState` to become `NOT_READY_AUTO_CAL_MOVE`

## WeldControl State Machine

```plantuml

title Weld Control State Machine

skinparam backgroundColor #DCE8F7
skinparam state {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
    StartColor #000000
    EndColor #000000
}

[*] --> IDLE_MODE : Initial

state "Mode: IDLE" as IDLE_MODE {
    state IDLE_STATE
    [*] --> IDLE_STATE
}

state "Mode: JOINT_TRACKING" as JT_MODE {
    state JT_IDLE
    state JT_WELDING
    [*] --> JT_IDLE
    JT_IDLE --> JT_WELDING : arcing && velocity > 0
    JT_WELDING --> JT_IDLE : !arcing || velocity <= 0
}

state "Mode: AUTOMATIC_BEAD_PLACEMENT" as ABP_MODE {
    state ABP_IDLE
    state ABP_WELDING
    [*] --> ABP_IDLE
    ABP_IDLE --> ABP_WELDING : arcing && velocity > 0
    ABP_WELDING --> ABP_IDLE : !arcing || velocity <= 0
}

IDLE_MODE --> JT_MODE : JointTrackingStart
JT_MODE --> ABP_MODE : AutoBeadPlacementStart (FILL or CAP)
ABP_MODE --> JT_MODE : AutoBeadPlacementStop
JT_MODE --> IDLE_MODE : Stop
ABP_MODE --> IDLE_MODE : Stop

```

### WeldControl Modes

#### IDLE

- **Description**: Weld control is idle, no active tracking or welding
- **Transitions to JOINT_TRACKING**: Via `JointTrackingStart()`
- **State**: Always `IDLE` (no welding possible)

#### JOINT_TRACKING

- **Description**: Joint tracking mode is active
- **Transitions to AUTOMATIC_BEAD_PLACEMENT**: Via `AutoBeadPlacementStart()`
- **Transitions to IDLE**: Via `Stop()`
- **State**: Can be `IDLE` or `WELDING` (based on arcing and velocity)

#### AUTOMATIC_BEAD_PLACEMENT

- **Description**: Automatic bead placement mode is active
- **Layer Types**: FILL or CAP
- **Transitions to JOINT_TRACKING**: Via `AutoBeadPlacementStop()`
- **Transitions to IDLE**: Via `Stop()`
- **State**: Can be `IDLE` or `WELDING` (based on arcing and velocity)
- **Note**: Can switch between FILL and CAP layers without returning to JOINT_TRACKING

### WeldControl States (within each mode)

#### IDLE

- **Condition**: Not welding (no arcing or velocity <= 0)
- **Description**: System is not welding with any Adaptio function

#### WELDING

- **Condition**: `arcing == true && weld_axis_angular_velocity > 0`
- **Description**: Actively welding
- **Transition**: Automatically determined in `ProcessInput()` based on weld system state

## Weld Session

The weld session tracks the state of an ongoing welding operation for Automatic Bead Placement (ABP).

### Structure

Three boolean flags:

- **`active`**: Whether a weld session is currently active
- **`resume_blocked`**: Whether resuming is blocked (prevents ABP from starting)
- **`ready_for_cap`**: Whether the system is ready for CAP layer operations

### Updating

- **Activation**: Set to active when `AutoBeadPlacementStart()` is called from `JOINT_TRACKING` mode
- **State updates**:

   - `ready_for_cap` set when confident slice buffer is sufficient or CAP layer is started directly
   - `resume_blocked` set when handover to manual is required (insufficient buffer fill ratio)

- **Clearing**: Cleared when arcing detected in IDLE/JOINT_TRACKING modes (manual welding) or via WebHMI `ClearWeldSession` message

### Usage

- **ABP readiness**: `resume_blocked` prevents ABP from starting in `ABPReady()` check
- **CAP readiness**: `ready_for_cap` determines if ABP CAP mode is ready in `UpdateReady()`
- **State coordination**: Session state affects `weld_control_abp_ready`, which influences `ReadyState`

## ManagementServer ReadyState

The `ReadyState` is a computed state based on multiple conditions. It serves as input to the management client statemachine which determines what operations the PLC can request.

```plantuml

title Ready State Computation Logic

skinparam backgroundColor #DCE8F7
skinparam state {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}

state "Compute ReadyState" as COMPUTE {
    state "Check ActivityStatus" as CHECK_ACTIVITY
    state "Check Conditions" as CHECK_COND
    state "Determine ReadyState" as DETERMINE
    
    CHECK_ACTIVITY --> CHECK_COND
    CHECK_COND --> DETERMINE
}

state "NOT_READY" as NOT_READY
state "NOT_READY_AUTO_CAL_MOVE" as AUTO_CAL
state "TRACKING_READY" as TRACK_READY
state "ABP_READY" as ABP_READY
state "ABP_CAP_READY" as CAP_READY
state "ABP_AND_ABP_CAP_READY" as BOTH_READY

COMPUTE --> AUTO_CAL : activity_status == CALIBRATION_AUTO_MOVE
COMPUTE --> NOT_READY : Default or conditions not met
COMPUTE --> TRACK_READY : !busy_from_webhmi &&\nweld_object_cal_valid &&\njoint_geometry &&\nweld_control_jt_ready
COMPUTE --> ABP_READY : TRACKING_READY &&\nweld_control_abp_ready
COMPUTE --> CAP_READY : TRACKING_READY &&\nweld_control_abp_cap_ready
COMPUTE --> BOTH_READY : TRACKING_READY &&\nweld_control_abp_ready &&\nweld_control_abp_cap_ready

```

### ReadyState Values

#### NOT_READY

- **Conditions**: Default state when conditions are not met
- **Description**: System is not ready for any PLC-driven sequences

#### NOT_READY_AUTO_CAL_MOVE

- **Conditions**: `activity_status == CALIBRATION_AUTO_MOVE`
- **Description**: System is busy with calibration auto-move sequence
- **Note**: Overrides all other ready states

#### TRACKING_READY

- **Conditions**:
   - `!busy_from_webhmi` (ActivityStatus is IDLE or TRACKING)
   - `weld_object_cal_valid == true`
   - `joint_geometry.has_value() == true`
   - `weld_control_jt_ready == true`
- **Description**: System is ready for joint tracking

#### ABP_READY

- **Conditions**: `TRACKING_READY && weld_control_abp_ready`
- **Description**: System is ready for ABP (fill layer)

#### ABP_CAP_READY

- **Conditions**: `TRACKING_READY && weld_control_abp_cap_ready`
- **Description**: System is ready for ABP CAP (cap layer)

#### ABP_AND_ABP_CAP_READY

- **Conditions**: `TRACKING_READY && weld_control_abp_ready && weld_control_abp_cap_ready`
- **Description**: System is ready for both ABP fill and cap layers

### ReadyState Update Triggers

The `ReadyState` is recomputed when:

- `WeldControl` ready modes change (via subscription)
- `CalibrationStatus` changes (via subscription)
- `ActivityStatus` changes (via subscription)
- `JointGeometryProvider` changes (via subscription)

## CalibrationStatus

`CalibrationStatus` is not a state machine but provides a boolean validation:

- **`WeldObjectCalibrationValid()`**: Returns `true` if weld object calibration is valid
- **Purpose**: Used as a condition for `TRACKING_READY` state
- **Subscription**: ManagementServer subscribes to changes and updates ReadyState accordingly

## State Coordination

### ActivityStatus to ReadyState

The `ActivityStatus` directly influences `ReadyState`:

- **CALIBRATION_AUTO_MOVE**: Forces `ReadyState` to `NOT_READY_AUTO_CAL_MOVE`
- **IDLE or TRACKING**: Allows `ReadyState` to progress to `TRACKING_READY` (if other conditions met)
- **Other calibration states**: Set `busy_from_webhmi = true`, preventing `TRACKING_READY`

### WeldControl to ReadyState

The `WeldControl` mode and readiness influence `ReadyState`:

- **`weld_control_jt_ready`**: Set when `WeldControl` reports `JOINT_TRACKING` mode is ready
- **`weld_control_abp_ready`**: Set when `WeldControl` reports `AUTOMATIC_BEAD_PLACEMENT` with `FILL` layer is ready
- **`weld_control_abp_cap_ready`**: Set when `WeldControl` reports `AUTOMATIC_BEAD_PLACEMENT` with `CAP` layer is ready

### ManagementServer to ActivityStatus

The `ManagementServer` sets `ActivityStatus`:

- **TRACKING**: Set when `OnTrackingStart()` is called
- **IDLE**: Set when `StopActiveFunction()` is called

### ManagementServer to WeldControl

The `ManagementServer` controls `WeldControl`:

- **JointTrackingStart**: Calls `weld_control_->JointTrackingStart()`
- **JointTrackingUpdate**: Calls `weld_control_->JointTrackingUpdate()`
- **AutoBeadPlacementStart**: Calls `weld_control_->AutoBeadPlacementStart()`
- **AutoBeadPlacementStop**: Calls `weld_control_->AutoBeadPlacementStop()`
- **Stop**: Calls `weld_control_->Stop()`

## Typical State Flow

### Starting Joint Tracking

1. PLC sends `TrackingStart` to `ManagementClient`
2. `ManagementClient` sends `TrackingStart` to `ManagementServer`
3. `ManagementServer`:
   - Sets `ActivityStatus` to `TRACKING`
   - Calls `WeldControl::JointTrackingStart()`
4. `WeldControl` changes mode from `IDLE` to `JOINT_TRACKING`
5. `ManagementServer` updates `ReadyState` (may become `TRACKING_READY`)

### Starting ABP from Tracking

1. PLC sends `ABPStart` to `ManagementClient`
2. `ManagementClient` sends `ABPStart` to `ManagementServer`
3. `ManagementServer` calls `WeldControl::AutoBeadPlacementStart(LayerType::FILL)`
4. `WeldControl` changes mode from `JOINT_TRACKING` to `AUTOMATIC_BEAD_PLACEMENT`
5. `WeldControl` state may transition from `IDLE` to `WELDING` when arcing starts

### Calibration Sequence

1. WebHMI initiates calibration sequence
2. `CalibrationSequenceRunner` sets `ActivityStatus` to `CALIBRATION_AUTO_MOVE`
3. `ManagementServer` detects change and sets `ReadyState` to `NOT_READY_AUTO_CAL_MOVE`
4. `ManagementClient` receives `ReadyState` update and overrides output to show calibration mode
5. When calibration completes, `ActivityStatus` returns to `IDLE` and `ReadyState` recalculates

## Notes

- The `WeldControl` state (IDLE/WELDING) is automatically determined based on weld system arcing status and weld axis velocity
- The `ReadyState` is computed, not a state machine with explicit transitions
- Multiple components subscribe to state changes to maintain consistency
- Calibration operations (from WebHMI) take precedence over PLC-driven operations
