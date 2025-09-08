# Diagram

```plantuml

hnote over Adaptio, PLC
Start joint tracking
end note
Adaptio -> PLC : Execute=True, NewPosition(horiz, vertical)
PLC -> Adaptio : InPosition=false
Adaptio -> PLC : Execute=True, NewPosition(horiz, vertical)
PLC -> Adaptio : InPosition=true
Adaptio -> PLC : Execute=True, NewPosition(horiz, vertical)
PLC -> Adaptio : InPosition=false
Adaptio -> PLC : Execute=True, NewPosition(horiz, vertical)

hnote over Adaptio, PLC
End joint tracking
end note

Adaptio -> PLC : Execute=False, Stop=True

```
