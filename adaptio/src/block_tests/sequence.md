# Diagram

```plantuml

AdaptioCentral -> Controller : Kinematics::Connect

hnote over AdaptioCentral, Scanner
Joint tracking function started
end note
Controller -> AdaptioCentral : Management::JointTrackingStart

AdaptioCentral -> Scanner : Scanner::StartScanner

hnote over AdaptioCentral, Scanner
ABW points sent from Scanner
end note
Scanner -> AdaptioCentral : Scanner::CoordinatesInput

AdaptioCentral -> Controller : Kinematics::GetPosition

Controller -> AdaptioCentral : Kinematics::GetPositionResponse

hnote over AdaptioCentral, Scanner
New axis position requested
end note
AdaptioCentral -> Controller : Kinematics::SetPosition

```
