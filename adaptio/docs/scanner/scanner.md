# Scanner

## Introduction

When the scanner is started it continously sends slices (set of 7 ABW points) to the main thread.
They are sent with a 50 ms pace and are sent even if the scanner can not calculate the ABW points for an
image. The confidence in each slice indicates the quality of the ABW points.

If the scanner sent slices with confidence LOW/NO for a period of three seconds it
internally change mode to only use the approximation of ABW0_x/ABW6_x to
calculate the ABW points if the the approximation is available.

### Confidence

Each slice sent from scanner to main have a confidence.

* HIGH
   * Both walls found and the depth on left and right side is at least 7 mm
* MEDIUM
   * Both walls found but depth is less than 7 mm
   * One wall found and the depth on one side is at least 6 mm
   * If approximation of ABW0_x/ABW6_x is used
* LOW
   * One wall found and the depth on both sides are less than 6 mm
   * Zero walls found
* NO
   * Not able to calculate the ABW points for current image and the latest calculated are sent
   * At startup of scanner until a median slice is available. The ABW points is set to 0 in those slices.

### Input data

Several input data are used by the scanner to exctract the ABW points.

* Image
   * The image captured by the camera of the joint
* Joint geometry
   * Entered by the operator in WEBHMI. Used for finding groove and check tolerances
* Groove width
   * If roller beds are homed main will update scanner with the groove width from earlier revolutions at the same weld axis position
* Approximation of ABW0 and ABW6 horizontally
   * If roller beds are homed and edge sensor is available main will update scanner with an approximation of ABW0 and ABW6 horizontal position

```plantuml
title Process one image

skinparam backgroundColor #DCE8F7
skinparam sequenceParticipant {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}

hide footbox

participant "Scanner" as Scanner
participant "Main" as Main

  alt Roller beds not HOMED - no re-feed from Main
  note over Scanner: With this input data it is required to find both walls otherwise confidence is NO
    alt Groove depth > ~5mm
      Scanner -> Main : Slice(Confidence = HIGH/MEDIUM)
    else Groove depth < ~5mm
      Scanner -> Main : Slice(Confidence = NO)
    end
    
  else Roller beds HOMED - re-feed from Main
    alt No edge sensor
      note over Main: Main will refeed groove width for current weld axis position
      Main -> Scanner: Update(Groove width)

      Scanner -> Main : Slice(Confidence = HIGH/MEDIUM/LOW/NO)

    else Edge sensor
      note over Main: Main will re-feed groove width and appoximation for current weld axis position
      Main -> Scanner: Update(Groove width, approximation)
      alt LOW or NO for 3 consecutive seconds
        note over Scanner: The approximation is used. Never exit this state if entered
        Scanner -> Main : Slice(Confidence = MEDIUM)
      else
        Scanner -> Main : Slice(Confidence = HIGH/MEDIUM/LOW/NO)
      end
    end
  end
```
