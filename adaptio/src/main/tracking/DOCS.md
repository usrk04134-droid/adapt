# Diagram

```plantuml
left to right direction

package tracking {
  class "TrackingManagerImpl" as tracking::TrackingManagerImpl
  class "HorizontalTracking" as tracking::HorizontalTracking
  class "VerticalTracking" as tracking::VerticalTracking
  interface "TrackingManager" as tracking::TrackingManager
  interface "TrackingObserver" as tracking::TrackingObserver

  tracking::TrackingManagerImpl --> tracking::TrackingObserver
  tracking::TrackingManagerImpl --> tracking::HorizontalTracking
  tracking::TrackingManagerImpl --> tracking::VerticalTracking
  tracking::TrackingManagerImpl --|>  tracking::TrackingManager
}

```
