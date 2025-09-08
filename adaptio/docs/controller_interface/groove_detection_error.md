# Groove detection error

```plantuml

hnote over Controller, Adaptio
Precondition: Tracking or ABP active
end note

Adaptio -> Controller : TrackOutput(active=1,  error=0)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=0)

hnote over Adaptio
Groove estimate unavailable
-5 sec timeout
end note

hnote over Controller, Adaptio
Status active with error
end note

Adaptio -> Controller : TrackOutput(active=1, error=1)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=1)

```
