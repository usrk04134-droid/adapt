# Diagram

```plantuml

hnote over Controller
Operator starts joint tracking
end note

Controller -> ControllerThread : AdaptioInput(start=1, sequence=1)
Controller -> ControllerThread : TrackInput(mode, h_offset)
ControllerThread -> AdaptioCentral : management::JointTrackingStart

AdaptioCentral -> Scanner : scanner::Start

Scanner -> AdaptioCentral : scanner::SliceData

AdaptioCentral -> ControllerThread : kinematics::GetSlidesPosition

ControllerThread -> AdaptioCentral : kinematics::GetSlidesPositionRsp

hnote over AdaptioCentral
Transform SliceData to Machine Coord
Calculate wanted slides position
end note
AdaptioCentral -> ControllerThread : kinematics::SetSlidesPosition

ControllerThread -> Controller : AxisOutput(horiz, vert)

hnote over AdaptioCentral
SliceData supervision timeout - 5sec
(abw points unavailable)
end note

AdaptioCentral -> Scanner : scanner::Stop
AdaptioCentral -> ControllerThread : kinematics::Release
ControllerThread <- AdaptioCentral : management::JointTrackingFailed

ControllerThread -> Controller : AdaptioOutput(error=1)

```
