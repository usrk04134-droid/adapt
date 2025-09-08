# Stop

```plantuml

hnote over Controller, Adaptio
Precondition: Tracking or ABP active
end note

Adaptio -> Controller : TrackOutput(active=1, error=0)
Adaptio -> Controller : AdaptioOutput(active=1,ready=0, error=0)

hnote over Controller
Stop
end note

Controller -> Adaptio : AdaptioInput(start=0, stop=1)
Adaptio -> Controller : TrackOutput(active=0, error=0)
Adaptio -> Controller : AdaptioOutput(active=0, ready=1, error=0)

hnote over Controller, Adaptio
Postcondition: Adaptio ready
end note

```
