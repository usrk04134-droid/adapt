# Back to tracking from ABP

```plantuml

hnote over Controller, Adaptio
Precondition: ABP active
end note

Adaptio -> Controller : TrackOutput(active=1,  error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller, Adaptio
User deactivates ABP:
-PLC changes to tracking
-PLC sets trackingmode to center
-PLC calculates center offset based on current position.
end note

Controller -> Adaptio : AdaptioInput(start=1, stop=0, sequence=1)
Controller -> Adaptio : TrackInput(mode=1/center, offset) (start acting on this)
Adaptio -> Controller : TrackOutput(active=1, error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller, Adaptio
Postcondition: Tracking active
end note

```
