# Error to ready

```plantuml

hnote over Controller, Adaptio
Status active with error
end note

Adaptio -> Controller : TrackOutput(active=1, shallow=0, error=1)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=1)


hnote over Controller
Stop
end note

Controller -> Adaptio : AdaptioInput(start=0, stop=1)

hnote over Controller, Adaptio
Adaptio ready without error
end note

Adaptio -> Controller : TrackOutput(active=0, shallow=0, error=0)
Adaptio -> Controller : AdaptioOutput(active=0, ready=1, error=0)


```
