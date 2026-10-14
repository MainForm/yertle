# Forward Gait Flow Diagram

```mermaid
flowchart TD
    Start["Set forward direction"]
    Running{"Simulation running?"}
    Gait["Calculate foot trajectories"]
    IK["Calculate joint angles"]
    Motor["Set leg motor targets"]
    Step["Advance physics"]
    End(["End"])

    Start --> Running
    Running -->|Yes| Gait --> IK --> Motor --> Step --> Running
    Running -->|No| End

    click Start href "https://github.com/MainForm/yertle/blob/d3afb1239eb35b7fb929203b3da426ff1f01af00/software/gait_simulation/workspace/main.py#L41" "View call site on GitHub" _blank
    click Running href "https://github.com/MainForm/yertle/blob/d3afb1239eb35b7fb929203b3da426ff1f01af00/software/gait_simulation/workspace/main.py#L61" "View call site on GitHub" _blank
    click Gait href "https://github.com/MainForm/yertle/blob/d3afb1239eb35b7fb929203b3da426ff1f01af00/software/gait_simulation/workspace/simulator/yertle.py#L152" "View call site on GitHub" _blank
    click IK href "https://github.com/MainForm/yertle/blob/d3afb1239eb35b7fb929203b3da426ff1f01af00/software/gait_simulation/workspace/simulator/yertle.py#L164" "View call site on GitHub" _blank
    click Motor href "https://github.com/MainForm/yertle/blob/d3afb1239eb35b7fb929203b3da426ff1f01af00/software/gait_simulation/workspace/simulator/yertle.py#L184" "View call site on GitHub" _blank
    click Step href "https://github.com/MainForm/yertle/blob/d3afb1239eb35b7fb929203b3da426ff1f01af00/software/gait_simulation/workspace/main.py#L63" "View call site on GitHub" _blank
```

Click a block to open the corresponding method call on GitHub. This diagram shows continuous forward walking while the Up arrow is held; initialization and keyboard callback details are omitted.

For a closer look at the gait, run the scripts in [gait_visualization](../workspace/gait_visualization/) to inspect the Matplotlib graphs of the forward gait, leg forward kinematics, and leg inverse kinematics.
