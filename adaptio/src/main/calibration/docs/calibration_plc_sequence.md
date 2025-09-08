# Diagram

```plantuml

title Weld Object Calibration Procedure

skinparam backgroundColor #DCE8F7
skinparam sequenceParticipant {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}

hide footbox

participant "PLC" as PLC
participant "Adaptio" as Adaptio
participant "WebHMI" as WebHMI

  == Preconditions adaptio ready, object not moving==
  Adaptio -> PLC : AdaptioOutput(active=0, ready=1, active_sequence_type=0)
  PLC -> Adaptio : AxisInput(weld_axis_velocity = 0)

  == Phase 1: Manual touch left / right ==
  note over WebHMI : Operator presses WeldObjectCalibration start
  WebHMI -> Adaptio : WeldObjectCalStart
  Adaptio -> PLC : AdaptioOutput(active=1, ready=0, active_sequence_type=3)
  note over WebHMI : Operator instructed to touch left wall
  note over PLC : Sequence type 3 = manual touch sense left mode

  note over PLC : Joystick used to move wire to touch left wall
  note over WebHMI : Operator presses left wall
  WebHMI -> Adaptio : WeldObjectCalLeftPos
  PLC -> Adaptio : AxisInput(slide_position)
  WebHMI <- Adaptio : WeldObjectCalLeftPosRsp

  Adaptio -> PLC : AdaptioOutput(active=1, ready=0, active_sequence_type=4)
  note over WebHMI : Operator instructed to touch right wall
  note over PLC : Sequence type 4 = manual touch sense right mode
  note over PLC : Joystick used to move wire to touch right wall
  note over WebHMI : Operator presses right wall
  WebHMI -> Adaptio : WeldObjectCalRightPos
  PLC -> Adaptio : AxisInput(slide_position)
  WebHMI <- Adaptio : WeldObjectCalRightPosRsp

  == Phase 2: Automatic measurements ==

  Adaptio -> PLC : AdaptioOutput(active=1, ready=0, active_sequence_type=5)
  note over PLC : Sequence type 5 = auto measurement, plc accepts SetSlides

  loop All positions in grid
    PLC <- Adaptio : AxisOutput(slide_position)
    PLC -> Adaptio : AxisInput(slide_position)
  end

  WebHMI <- Adaptio : WeldObjectCalResult

  == Postconditions ==
  Adaptio -> PLC : AdaptioOutput(active=0, ready=1, active_sequence_type=0)

```
