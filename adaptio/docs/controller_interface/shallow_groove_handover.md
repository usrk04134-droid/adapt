# Shallow groove handover

```plantuml

hnote over Controller, Adaptio
Precondition: Tracking or ABP active
end note

hnote over Adaptio
Groove shallow
-3 sec timeout
end note

Adaptio -> Controller : TrackOutput(active=1, shallow=1, error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller
Operator alerted
Operator switches to manual horizontal tracking
end note

Controller -> Adaptio : AdaptioInput(start=0, stop=1)
Adaptio -> Controller : TrackOutput(active=0, shallow=0, error=0)
Adaptio -> Controller : AdaptioOutput(active=0, ready=1, error=0)

hnote over Controller, Adaptio
Postcondition: Adaptio ready
end note

```
