# Tracking start from idle

```plantuml

hnote over Controller, Adaptio
Precondition: Adaptio ready
end note

Adaptio -> Controller : TrackOutput(active=0,  error=0)
Adaptio -> Controller : AdaptioOutput(active=0,ready=1, error=0)

hnote over Controller
Start tracking
end note

Controller -> Adaptio : AdaptioInput(start=1, stop=0, sequence=1)
Controller -> Adaptio : TrackInput(mode, offset)
Adaptio -> Controller : TrackOutput(active=1, error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller, Adaptio
Postcondition: Tracking active
end note

```
