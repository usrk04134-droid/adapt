# Preface

This report summarizes weld data from a weld job (weldcontrol log). For each selected angular position around the workpiece, the report aggregates samples into labeled points and visualizes them over the X–Z plane. Labels use the format **(layerNo, beadNo)** for each point, where `layerNo` may be adjusted across ABP sessions.

## What’s shown

- **Bead area (mm²):** An estimate of deposited bead area derived from wire speed/diameter and weld speed, as a sum of the two weld systems.
- **Heat input (kJ/mm):** The sum of heat-input values reported by the two weld systems.
- **Current (A):** The actual welding current reported for weld system 2.
- **Weld speed (mm/s):** The weld speed (linear speed) calculated from the weld radius and angular velocity.
- **Bead slice area ratio (–):** A relative indicator of how the current bead’s “slice” (compartment of the groove) compares to the average slice for the layer. Values >1 mean the current slice is larger than average; <1 means smaller than average. As an example, for a layer with three beads, the remaining empty groove area is divided into three compartments (slices).
- **Groove area ratio (–):** A relative (inverse) indicator derived from the empty-layer area at the current angle compared to the empty-layer average for all angles. Values ≈1 indicate typical area at that angle; >1 or <1 indicate a groove narrower or wider than average, respectively.

Each subplot overlays one or two **ABW profiles**:

- **Gray** shows an abw profile from the start of the process.  
- **Green** (if available) shows a profile captured during a post-welding scan.

## Coordinates and reference

All coordinates in the plots are **relative to `abw0`** (the left top corner of the groove). We use `hybrid groove` as reference in the figures. The `hybrid groove` is what is used by Adaptio for the joint tracking. It is a combination of direct and delayed scanner data. The bead positions are calculated as follows:

- **X:** `slides.actual.horizontal - abw0.x`
- **Z:** `slides.actual.vertical - stickout - abw0.z`

All coordinates are expressed in the Machine Coordinate System (MCS) used in Adaptio. MCS is also directly used by the PLC slides control. The joint tracking and bead placement functions use a combination of realtime and delayed scanner data (`hybrid groove`). When the joint moves rapidly due to for example longitudinal welds, the system acts on realtime scanner data. In situations when overlapping beads, the system instead acts on delayed data.

## Layer numbering across sessions

Layer numbering carries across multiple ABP sessions. The report tracks **ABP start** events and **steady ABP** entries and adjusts the stored `layerNo` so that layers increase monotonically from the beginning to the end of the log. Beads without annotation were welded using joint tracking (not abp).

## Angular selection

For each **pos_deg** in 45° steps around the workpiece, the script collects steady(not bead repositioning), ABP-mode samples within a small angular window (±2° by default). Adjacent samples with constant `(layerNo, beadNo)` are averaged into one cluster for plotting and labeling.

## Weld settings

The first page lists the most recent applicable ABP parameters found in the log, including limits and derived units (e.g., weld speed in both **cm/min** and **mm/s**).

## Notes & limitations

- Values are averaged per cluster only; no additional filtering is applied in this report.
- The plots show relative positions with **X inverted**. This is the same way the camera sees the groove.
