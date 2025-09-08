# Start ABP from tracking

```plantuml

hnote over Controller, Adaptio
Precondition: Tracking active
end note

Adaptio -> Controller : TrackOutput(active=1,  error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller
Start ABP
end note

Controller -> Adaptio : AdaptioInput(start=1, stop=0, sequence=2)
Adaptio -> Controller : TrackOutput(active=1, error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller, Adaptio
Postcondition: ABP active
end note

```
