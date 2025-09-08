# ADAPTIO

## Tracking

### With edge sensor

When welding close to the top surface it is difficult to determine the
groove's position but with edge sensor, scanner input, and historical groove
information combined it is possible to use joint tracking for the entire
groove, including welding of the CAP layer.

Restarting Adaptio, re-calibration, or weld axis homing will clear
historical groove information and can impact the possibility to use joint
tracking if any of these operations are done when close to the top surface.

If the edge sensor value becomes unavailable, e.g. the senor loses contact
with the weld object, an 'Edge sensor not available' event will be triggered
and Adaptio will stop ungracefully*.

#### Parameters

Enhanced tracking with edge sensor support requires the following parameter
to be configured before weld start.

##### HMI - ABP parameters

- Use edge sensor
   - Must be enabled for enhanced tracking.
- Edge sensor placement
   - Right or left side of the weld object.

### Without edge sensor

Without edge sensor support it is possible to use joint tracking until a
groove depth of approximately 6 mm and at this point a popup will be shown in
the HMI asking the operator to change to manual mode. When the popup is shown
the operator needs to change to manual mode within 25 seconds otherwise a
'Handover fault' event fill be triggered and Adaptio will stop ungracefully*.

## Auto CAP

The Auto CAP procedure places beads in the CAP layer automatically. The beads
are evenly placed left to right and the first and last beads are placed with
an offset from the groove's top corners. When the last bead in the layer is
finished the application will stop ungracefully*.

### Parameters

Auto CAP required the following input parameters to be configured before weld
start.

#### HMI - ABP parameters

- Use edge sensor
   - Must be enabled for Auto CAP.
- Edge sensor placement
   - Right or left side of the weld object.
- Bead switch angle
   - Same value used for both CAP and FILL layers.
- Bead switch overlap
   - Same value used for both CAP and FILL layers.
- Cap beads
   - Number of evenly placed beads in the CAP layer.
- Cap corner offset
   - Offset from the groove's top corners for the first and last bead. For
     example, a value of 3 places the first and last bead 3 mm from the
     original groove's top corners towards the middle.
- Cap init depth
   - Groove depth parameter used to determine if the next layer is CAP layer.
   - Next layer is CAP if the groove's depth is less than the 'Cap init depth'
     value when welding the last FILL layer. The height of the last fill layer
     should be included in the value.

### Transitions

Auto CAP available is indicated by orange outline of the 'Auto cap' button in
the HMI and it can enabled from either joint tracking or ABP modes.

#### From joint tracking

Auto CAP will be available when groove depth is less than 7 mm and when the
button is pressed Adaptio will immediately position the torch for the first
bead in the CAP layer i.e. to the left side.

#### Transition from ABP

Adaptio will notify the operator approximately 25 seconds before the last
bead in the last FILL layer is finished by displaying a popup in the HMI.

When the popup is shown the operator needs to do the following steps:

1. Close the popup by pressing the 'Confirm' button.
2. Operator agrees that the next layer should be CAP
   1. Press the 'Auto cap' button in the HMI
      1. Adaptio will finish the current bead before starting the CAP layer.
   2. Change to CAP weld recipe in the HMI.

If the operator does not agree that the next layer is CAP when the popup is
shown he/she needs to change to joint tracking or manual mode. If a
transition to auto CAP, joint tracking, or manual is not done within 25
seconds a 'Handover fault' event will be triggered and Adaptio will stop
ungracefully*.

---

*An ungraceful stop is an immediate termination of the welding process
without following normal shutdown procedures. This type of stop occurs when
Adaptio encounters critical errors or timeout conditions that require
immediate intervention. During an ungraceful stop, the system halts all
operations instantly to prevent potential damage or safety hazards, but this
may leave the weld in an incomplete state that requires manual assessment
before resuming operations.
