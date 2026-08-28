# Simulator Class Diagram

```mermaid
classDiagram
    class Simulator {
        -int physics_client
        -int fps
        -bool closed
        -URDFManager urdf_manager
        +Simulator __enter__()
        +void __exit__(exc_type, exc_value, traceback)
        -void _ensure_open()
        +client_id int
        +step() void
        +is_running() bool
        +close() void
        +load_urdf(path: str or Path, base_position: tuple, base_orientation: tuple, use_fixed_base: bool, global_scaling: float) URDFObject
        +remove_urdf(obj: URDFObject) void
    }

    class URDFManager {
        -int physics_client
        -dict objects
        -bool closed
        -void _ensure_open()
        +load(path: str or Path, base_position: tuple, base_orientation: tuple, use_fixed_base: bool, global_scaling: float) URDFObject
        +remove(obj: URDFObject) void
        +close() void
    }

    class URDFObject {
        -int physics_client
        -int body_id
        -str or Path path
        -bool is_valid
        -list~Sequence~ joints_info
        +body_id int
        +path str or Path
        +is_valid bool
        +get_base_pose() tuple
        -invalidate() void
        -ensure_valid() void
        -ensure_joint_index(index) void
        -ensure_motor_index(index) void
        -ensure_motor_angles(target_joint_angles: Sequence) list~tuple~
        -get_all_joint_info() list~Sequence~
        +get_joint_state(joint_index) list
        +get_motor_indices() tuple~int~
        +get_motor_angle_limits() dict
        +reset_motor_angle(motor_index: int, target_angle: float) void
        +set_motor_angle(index, angle, force) void
        +set_multiple_motors_angle(motor_controls: Sequence) void
    }

    Simulator "1" *-- "1" URDFManager : owns
    URDFManager "1" *-- "0..*" URDFObject : manages
    Simulator ..> URDFObject : returns and removes
```

`Simulator` owns the PyBullet connection and delegates URDF lifecycle
operations to `URDFManager`. The manager creates and tracks each `URDFObject`,
while `URDFObject` acts as a handle to a body loaded in the simulation and
provides joint inspection and motor-control operations.
