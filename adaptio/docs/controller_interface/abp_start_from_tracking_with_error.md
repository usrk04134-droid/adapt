# ABP start from tracking with error

```plantuml

hnote over Controller, Adaptio
Precondition: Tracking active, APB parameters NOT set
end note

Adaptio -> Controller : TrackOutput(active=1,  error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Controller
Start ABP
Error bit is set because no ABP parameters
end note

Controller -> Adaptio : AdaptioInput(start=1, stop=0, sequence=2)
Adaptio -> Controller : TrackOutput(active=1, error=1)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=1)

hnote over Controller, Adaptio
Postcondition: error
end note

```
