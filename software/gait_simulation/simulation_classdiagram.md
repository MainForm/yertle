# Simulation Class Diagram

```mermaid
classDiagram
    class Simulation {
        -int physics_client
        -int fps
        -bool closed
        -URDFManager urdf_manager
        +client_id int
        +step() void
        +is_running() bool
        +close() void
        +load_urdf(path, base_position, base_orientation, use_fixed_base, global_scaling) URDFObject
        +remove_urdf(obj) void
    }

    class URDFManager {
        -int physics_client
        -dict objects
        -bool closed
        +load(path, base_position, base_orientation, use_fixed_base, global_scaling) URDFObject
        +remove(obj) void
        +close() void
    }

    class URDFObject {
        -int physics_client
        -int body_id
        -Path path
        -bool is_valid
        +body_id int
        +path Path
        +is_valid bool
        +get_base_pose() tuple
    }

    Simulation "1" *-- "1" URDFManager : owns
    URDFManager "1" *-- "0..*" URDFObject : manages
    Simulation ..> URDFObject : returns and removes
```

`Simulation` owns the PyBullet connection and delegates URDF lifecycle
operations to `URDFManager`. The manager creates and tracks each `URDFObject`,
while `URDFObject` acts as a handle to a body loaded in the simulation.
