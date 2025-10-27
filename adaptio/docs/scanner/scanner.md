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

### Horizontal ROI auto-shrinking (performance)

To increase camera frame rate beyond the vertical-only ROI reduction, the scanner now also reduces the horizontal ROI dynamically based on the median slice and current ABW positions.

- The camera hardware ROI `Width/OffsetX` is adjusted at runtime to tightly bracket ABW0..ABW6 with a safety margin.
- The ROI is computed from the median slice’s ABW0 and ABW6, unioned with the current frame’s ABW0/ABW6 when available, and expanded by a margin to keep context around the groove.
- Minimum ROI size is enforced to avoid starving downstream algorithms.

Parameters (defaults in code):
- Minimum horizontal width: 500 px
- Horizontal margin around ABW endpoints: 150 px
- Reconfigure threshold: 40 px change in start/width

Impact:
- Default 3500x2500 → 3500x500 already gave ~50 fps. With horizontal shrinking the effective ROI can drop well below 3500, further increasing achievable fps while reducing PCIe/CPU load.

How it works (high level):
- On each successfully parsed image, the scanner computes desired horizontal bounds from the median ABW0/ABW6 in image pixels and applies margins.
- If bounds differ sufficiently from current, it calls the image provider to apply `OffsetX/Width` and updates the camera model so workspace↔image transforms remain correct.

### Measuring improvement (fps and temperatures)

Use existing Prometheus metrics to quantify improvements:
- Processing rate (fps): `sum(rate(scanner_image_process_success_total[30s]))` (sum across labels found={0,1,2}). This approximates end-to-end fps. For camera-side fps, read your grabber/driver if exposed; otherwise processing rate is sufficient when the pipeline keeps up.
- Basler camera temperatures: `basler_camera_temperature`, `basler_camera_max_temperature`, and `basler_camera_temperate_status` gauges.

CPU temperature:
- If node exporter or lm-sensors is present, use those to collect CPU temps. On many systems you can read `/sys/class/thermal/**/temp` (values in millidegrees C).

Suggested experiment procedure:
1) Baseline (horizontal ROI disabled): run for ≥2 minutes, record mean fps and temperatures.
2) Enable horizontal ROI auto-shrinking (default on): run the same program/test and record the same metrics.
3) Compare averages and max values.

### Capture→slider request latency

Scanner timestamps each image close to the camera exposure time. The gauge
`adaptio_abw_latency_lpcs_seconds` reports the latency from image capture
until ABW data is produced by the scanner (in the laser plane coordinate system).

To approximate “camera capture → new slider position requested”, add your
actuation latency on top of ABW latency (the portion between scanner output
and the slider command). If your control path exposes a timestamp for when a
slider command is sent, subtract the ABW timestamp (carried with the slice) to
derive this additional component.
