# Management Client State Machine

This document describes the state machine in `ManagementClient` for managing the interface state between the PLC and Adaptio.

## State Machine Diagram

```plantuml

title Management Client - Interface State Machine

skinparam backgroundColor #DCE8F7
skinparam state {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
    StartColor #000000
    EndColor #000000
}

[*] --> IDLE : start

state IDLE
state TRACKING
state ABP
state ABP_CAP
state ERROR

IDLE --> TRACKING : start + TRACKING
IDLE --> ABP : start + ABP
IDLE --> ABP_CAP : start + ABP_CAP

TRACKING --> ABP : start + ABP
TRACKING --> ABP_CAP : start + ABP_CAP
TRACKING --> IDLE : stop

ABP --> TRACKING : start + TRACKING
ABP --> ABP_CAP : start + ABP_CAP
ABP --> IDLE : stop

ABP_CAP --> TRACKING : start + TRACKING
ABP_CAP --> IDLE : stop

ERROR --> IDLE : stop

note right of ERROR
  Reachable from all states:
  - Validation failure
  - Scanner error
  - Groove timeout
  - Heartbeat lost
end note


```

## States

### IDLE

- **Output**: `ready=true`, `active=false`, `error=false`, `active_sequence_type=NONE`
- **Description**: Initial state and default state when no sequence is active. System is ready to accept new sequence requests.

### TRACKING

- **Output**: `ready=false`, `active=true`, `error=false`, `active_sequence_type=TRACKING`
- **Description**: Joint tracking sequence is active. Can transition to ABP or ABP_CAP sequences.

### ABP

- **Output**: `ready=false`, `active=true`, `error=false`, `active_sequence_type=AUTO_WELDING`
- **Description**: Adaptive Bead Placement (ABP) sequence is active. Can transition back to TRACKING or to ABP_CAP.

### ABP_CAP

- **Output**: `ready=false`, `active=true`, `error=false`, `active_sequence_type=AUTO_CAP_WELDING`
- **Description**: ABP Cap sequence is active. Can transition back to TRACKING. Cannot transition to ABP.

### ERROR

- **Output**: `ready=false`, `active=true`, `error=true`, `active_sequence_type=NONE`
- **Description**: Error state. System has encountered an error condition. Can only transition to IDLE via stop or shutdown.

## Transitions

### Input-Driven (from PLC) Transitions

#### Start Sequence

- **Trigger**: `OnAdaptioInput` with `start=true`
- **Conditions**:
   - Ready state validation must pass
   - Current state must allow the transition
- **Actions**:
   - Send corresponding management messages

#### Stop

- **Trigger**: `OnAdaptioInput` with `stop=true`
- **Actions**:
   - Send `Stop` message if in TRACKING, ABP, or ABP_CAP
   - Transition to IDLE
   - Reset `handover_to_manual_` flag

#### Shutdown (not shown in diagram)

- **Trigger**: `OnAdaptioInput` with `shutdown=true`
- **Actions**:
   - Send `Shutdown` message
   - Transition to IDLE

### Event-Driven (from main) Transitions

#### Scanner Error

- **Trigger**: `OnScannerError`
- **Transition**: Any state to ERROR

#### Groove Data Timeout

- **Trigger**: `OnTrackingStoppedGrooveDataTimeout`
- **Transition**: Any state to ERROR

#### Graceful Stop (not graceful in current version)

- **Trigger**: `OnGracefulStop`
- **Transition**: Any state to ERROR
- **Note**: Used to signal PLC to stop weld procedure

#### Heartbeat Lost

- **Trigger**: `HeartbeatLost()`
- **Transition**: TRACKING or ABP to ERROR
- **Actions**: Send `Stop` message before transitioning

#### Disconnected

- **Trigger**: `Disconnected(reason_code)`
- **Transition**: TRACKING or ABP to ERROR
- **Actions**: Send `Stop` message before transitioning

## Ready State Validation

Before starting a sequence, the system validates that the `ReadyState` allows the requested sequence:

- **TRACKING**: Requires `ReadyState != NOT_READY`
- **ABP**: Requires `ReadyState == ABP_READY || ABP_AND_ABP_CAP_READY`
- **ABP_CAP**: Requires `ReadyState == ABP_CAP_READY || ABP_AND_ABP_CAP_READY`

If validation fails, the system transitions to ERROR state with an error log message.

## Special Cases

### ABP_CAP to ABP Transition

This transition is explicitly **not allowed** and will cause a transition to ERROR state.

### Tracking Input Updates

When in TRACKING state, `OnTrackingInput` updates trigger `SendTrackingUpdate()` messages to keep the tracking parameters synchronized.

### Handover to Manual

The `handover_to_manual_` flag is set when `OnOnNotifyHandoverToManual` is received. This flag is reset when transitioning to IDLE state.

## Auto Calibration Move State

The `ReadyState::NOT_READY_AUTO_CAL_MOVE` is a special case that affects the output but **does not change the InterfaceState** shown in the state machine diagram above.

### Relationship to State Machine

- **InterfaceState**: Remains unchanged. The InterfaceState is independent of the ReadyState and can be any state (IDLE, TRACKING, ABP, ABP_CAP, or ERROR). However, in practice, when `NOT_READY_AUTO_CAL_MOVE` is active, the InterfaceState is typically **IDLE** because:
   - Calibration sequences are initiated from the WebHMI, not from PLC input
   - The calibration sequence directly controls kinematics, which would conflict with active PLC sequences
   - If a PLC sequence was active, it would typically need to be stopped before calibration can begin
- **ActivityStatus**: When `activity_status_` is `CALIBRATION_AUTO_MOVE`, the management server sets `ReadyState` to `NOT_READY_AUTO_CAL_MOVE`. This indicates the system is busy with a calibration operation initiated from WebHMI.
- **ReadyState**: Set to `NOT_READY_AUTO_CAL_MOVE` when `activity_status_ == CALIBRATION_AUTO_MOVE`
- **Output Override**: The `Update()` method overrides the normal state output when this ReadyState is active, regardless of the current InterfaceState:
   - `ready = false`
   - `active = true`
   - `sequence_type = SEQUENCE_AUTO_CAL_MOVE`
   - All ready flags set to `false`

### Purpose

This state was added as a late addition to handle a case where:

- A calibration sequence is started from the WebHMI
- The PLC must be informed to accept slide position requests from Adaptio

### Future Considerations

In future versions, the flow of control will be reversed and this logic will be redesigned. In some sense, the current implementation is a workaround that may be refactored in the future. If the current architecture persists, it may be better to make AUTO_CAL_MOVE a separate state in the `InterfaceState` enum.
