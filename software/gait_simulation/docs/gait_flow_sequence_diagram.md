# Gait Flow Sequence Diagram

Forward walking during one iteration of the main simulation loop.

```mermaid
sequenceDiagram
    participant Main as main
    participant Yertle
    participant GaitManager
    participant PhaseManager

    loop while simulator.is_running()
        Main->>Yertle: update_gait(delta_time)
        Yertle->>GaitManager: step_gait(delta_time)
        GaitManager->>PhaseManager: step(delta_time)
        PhaseManager-->>GaitManager: Current phase
        GaitManager->>GaitManager: Calculate foot offsets
        GaitManager-->>Yertle: Foot offsets for two diagonal leg pairs
        Yertle->>Yertle: Calculate target foot positions
        Yertle->>Yertle: Solve inverse kinematics
        Yertle->>Yertle: Apply leg joint angles
        Yertle-->>Main: Update complete
        Note over Main: simulator.step()
    end
```

`delta_time = 1.0 / simulator.fps`. While stopped, `update_gait()` returns immediately; the gait calculations above run only while walking.
