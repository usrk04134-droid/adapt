# Diagram

```plantuml

title Joint Geometry Classes

skinparam backgroundColor #DCE8F7
skinparam class {
    BackgroundColor #6AA5F0
    BorderColor #000000
    FontColor #FFFFFF
}
skinparam package {
    BackgroundColor #DCE8F7
    BorderColor #000000
    FontColor #000000
}

package "joint_geometry\n<NS>" {
    class JointGeometry

    interface JointGeometryProvider {
        + GetJointGeometry() : std::optional<JointGeometry>
    }

    class JointGeometryProviderImpl {
        + GetJointGeometry() : std::optional<JointGeometry>
        - SubscribeWebHmi()
        - storage_ : SqlMultiStorage<StoredJointGeometry>
    }

    class StoredJointGeometry {
        + CreateTable()
        + StoreFn()
        + RemoveFn()
        + GetAllFn()
    }
}

' Relationships
JointGeometryProvider <|-- JointGeometryProviderImpl
JointGeometryProvider -up-> JointGeometry : <<uses>>
JointGeometryProviderImpl --> StoredJointGeometry : <<uses>>


```
