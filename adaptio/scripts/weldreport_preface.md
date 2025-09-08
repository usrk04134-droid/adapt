# Preface

This report summarizes weld data from a weld job (weldcontrol log). For each selected angular position around the workpiece, the report aggregates samples into labeled points and visualizes them over the torch X–Z plane. Labels use the format **(layerNo, beadNo)** for each point, where `layerNo` may be adjusted across ABP sessions(start/stop) as described below.

## What’s shown

- **Bead area (mm²):** An estimate of deposited bead area derived from wire speed/diameter and weld speed.
- **Heat input (kJ/mm):** The sum of heat-input values reported by the two weld systems.
- **Current (A):** The actual welding current reported for weld system 2.
- **Weld speed (mm/s):** The weld speed (linear speed) calculated from the weld radius and angular velocity.
- **Bead slice area ratio (–):** A relative indicator of how the current bead’s “slice” (compartment of the groove) compares to the average slice for the layer. Values >1 mean the current slice is larger than average; <1 means smaller than average. As an example, for a layer with three beads, the remaining empty groove area is divided into three compartments (slices).
- **Groove area ratio (–):** A relative (inverse) indicator derived from the empty-layer area at the current angle compared to the empty-layer average for all angles. Values ≈1 indicate typical area at that angle; >1 or <1 indicate a groove narrower or wider than average, respectively.

Each subplot overlays one or two **ABW profiles**:

- **Gray** shows an abw profile from the start of the process, with Z values offset by the **stickout**.  
- **Green** (if available) shows a profile captured during a post-welding scan.

## Coordinates and reference

All torch coordinates in the plots are **relative to `mcs[0]`** (the local groove reference at the same scanner epoch). We use `mcs` rather than the delayed `mcsDelayed` representation. Relative positions are computed as:

- **X:** `slides.actual.horizontal - mcs[0].x`
- **Z:** `slides.actual.vertical - mcs[0].z`

MCS is the Machine Coordinate System used in Adaptio. This is also directly used by the PLC slides control. ABW profile points are plotted relative to this same reference, with Z offset by the current stickout. The joint tracking and bead placement functions use a combination of realtime and delayed scanner data. There is no specification of exactly how these should be used under different circumstances. When the joint moves rapidly due to for example longitudinal welds, the system acts on realtime scanner data. In situations when overlapping beads, the system instead acts on delayed data. In this report `mcs` (realtime scanner data), rather than `mcsDelayed` (delayed scanner data), is used as reference. This explains why some figures show bead locations seemingly below the abw profile.

## Layer numbering across sessions

Layer numbering carries across multiple ABP starts. The report tracks **ABP start** events and **steady ABP** entries and adjusts the stored `layerNo` so that layers increase monotonically from the beginning to the end of the log. The first ABP start initializes the session; subsequent ABP starts advance the layer baseline to the maximum layer encountered so far.
Beads without annotation were welded using joint tracking (not abp).

## Angular selection

For each **pos_deg** in 45° steps around the workpiece, the script collects steady(not bead repositioning), ABP-mode samples within a small angular window (±2° by default). Adjacent samples with constant `(layerNo, beadNo)` are averaged into one cluster for plotting and labeling.

## Weld settings

The first page lists the most recent applicable ABP parameters found in the log, including limits and derived units (e.g., weld speed in both **cm/min** and **mm/s**).

## Notes & limitations

- Values are averaged per cluster only; no additional filtering is applied in this report.
- The plots show relative positions with **X inverted**. This is the same way the camera sees the groove.
