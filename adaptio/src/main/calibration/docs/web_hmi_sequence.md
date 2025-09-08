# Diagram

```plantuml

skinparam backgroundColor #DCE8F7
skinparam sequenceParticipant {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}

hide footbox

participant "Adaptio" as Adaptio
participant "WebHMI" as WebHMI

== Update Frontend ==
WebHMI -> Adaptio: WeldObjectCalGet

alt ok
    WebHMI <- Adaptio: WeldObjectCalGetRsp(result:"ok")
    note over WebHMI: Status: Calibrated
else fail
    WebHMI <- Adaptio: WeldObjectCalGetRsp(result:"fail")
    note over WebHMI: Status: Not calibrated
end


== Calibration success ==
note over Adaptio
  preconditions:
  - idle
  - valid laser torch config
  - valid joint geometry
end note

note over WebHMI : "Start Calibration" button press
WebHMI -> Adaptio : WeldObjectCalStart
WebHMI <- Adaptio : WeldObjectCalStartRsp(result:"ok")

note over WebHMI : "Left wall" button press
WebHMI -> Adaptio : WeldObjectCalLeftPos
Adaptio -> WebHMI : WeldObjectCalLeftPosRsp(result:"ok")

note over WebHMI : "Right wall"  button press
WebHMI -> Adaptio : WeldObjectCalRightPos
Adaptio -> WebHMI : WeldObjectCalRightPosRsp(result:"ok")

loop until progress = 100%
    WebHMI <- Adaptio: WeldObjectCalProgress(progress:val)
end

WebHMI <- Adaptio: WeldObjectCalResult(result:"ok")

WebHMI -> Adaptio: WeldObjectCalSet
WebHMI <- Adaptio: WeldObjectCalSetRsp(result:"ok")
note over Adaptio : idle
note over WebHMI: Status: Calibrated

== Calibration failure ==
alt start fail
    WebHMI <- Adaptio : WeldObjectCalStartRsp(result:"fail")
    note over Adaptio : idle

else left/right fail
    WebHMI <- Adaptio : WeldObjectCalLeft/RightPosRsp(result:"fail")
    note over Adaptio : idle

else sequence/calculation fail
    WebHMI <- Adaptio : WeldObjectCalResult(result:"fail")
    note over Adaptio : idle

end

== Stop/reset ==

note over Adaptio : busy / in calibration

WebHMI -> Adaptio : WeldObjectCalStop
WebHMI <- Adaptio : WeldObjectCalStopRsp(result:"ok")
note over Adaptio : idle

WebHMI -> Adaptio: WeldObjectCalGet

alt ok
    WebHMI <- Adaptio: WeldObjectCalGetRsp(result:"ok")
    note over WebHMI: Status: Calibrated
else fail
    WebHMI <- Adaptio: WeldObjectCalGetRsp(result:"fail")
    note over WebHMI: Status: Not calibrated
end
