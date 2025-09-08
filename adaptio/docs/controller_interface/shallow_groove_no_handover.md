# Shallow groove no handover

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
Operator alerted but
does not switch to manual
horizontal tracking
end note

hnote over Adaptio
Groove estimate unavailable
-5 sec timeout
end note

Adaptio -> Controller : TrackOutput(active=1, shallow=0, error=1)
Adaptio -> Controller : AdaptioOutput(active=1, ready=0, error=1)

hnote over Controller, Adaptio
Status active with error
end note

```
