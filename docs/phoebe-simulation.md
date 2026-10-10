# Phoebe fuel simulation

Run **WPILib: Simulate Robot Code** (Java 17), or `./gradlew simulateJava`.
The existing PathPlanner chooser, driver USB 0, operator USB 1, and named
commands drive the simulation. The GUI is disabled by default in this repo;
select the simulation GUI in WPILib when you want its controls. Enable the robot
and choose autonomous or teleop using the simulation Driver Station.

## View Phoebe and fuel

The custom AdvantageScope robot asset is `assets/Robot_Phoebe2026` (name
`Phoebe_2026`). Copy that entire folder into AdvantageScope's custom assets
folder and restart/reload assets. Connect AdvantageScope to the simulator's
NetworkTables server at `localhost`, open a 3D Field tab, and select the 2026 field.

- Add `/AdvantageKit/RealOutputs/FuelSim/Robot` as the robot pose, selecting
  `Phoebe_2026` as its model.
- Add `/FuelSim/Fuels` as game pieces, selecting the field's Fuel model.
- Plot `/AdvantageKit/RealOutputs/FuelSim/StoredFuel`, `LaunchedFuel`,
  `BlueScore`, and `RedScore` to inspect inventory and scoring.

The supplied GLB is a single rigid model with no configured moving components.
The intake/hood simulation changes collection and shot behavior but does not
articulate the mesh. Robot +X is both intake-forward and shooter-forward.

## Model assumptions

FuelSim simulates fuel motion, collisions, intake pickup, and geometric hub
scoring. It uses the existing simulated swerve pose and field-relative velocity.
It does not simulate driving over bumps, robot pitch, or full match scoring rules.

A new field starts with eight preloaded fuel and a provisional 40-fuel hopper.
Pickup requires a deployed intake and forward roller output; shots require
both spinning flywheels and forward loader/spindexer output. Disabled robots
cannot collect or fire. Reverse unjam does not create shots. The dashboard
`FuelSim/Reset field` command restores the field, inventory, and scores.

Launch geometry parameters remain **uncalibrated** and editable on the dashboard.
The feed-rate default is user-selected:

| Parameter | Initial value |
| --- | --- |
| Wheel radius | 0.0508 m |
| Exit speed / wheel surface speed | 0.50 |
| Launch height (robot center) | 0.65 m |
| Hood retracted / extended elevation | 70 / 45 degrees |
| Combined feed rate | 8 fuel/s |

`IMG_9981.mov` (30 fps) shows approximately 22 outgoing fuel in the main burst
from about 3.0 to 6.2 seconds: roughly 7 fuel/s overall. The initial burst is
faster and the later spacing is uneven. Overlapping balls and camera movement
make this an estimate, not an exact count. The simulator uses the user-selected constant rate of 8 fuel/s;
it does not yet reproduce the burst/gap pattern. An existing dashboard override
of `FuelSim/FuelsPerSecond` takes precedence over the new default.

Bumper footprint is 0.879 m square (PathPlanner settings); bumper collision
height is provisionally 0.20 m. Intake rectangle is robot-relative X 0.44–0.85 m,
Y ±0.40 m. Flywheels use a 0.15 s first-order response; hood and pivot use bounded
motion. These approximations test command flow, not measured mechanism dynamics.
Calibrate launch geometry, hood mapping, wheel transfer, capacity, and feed rate
before interpreting scores as predictions of the real robot. Existing shot tables
are left unchanged.

FuelSim is vendored from https://github.com/hammerheads5000/FuelSim at revision
`37eff8936e55fa4964b0c094ffd1093dff9a4519`, with its MIT license in the source.
Changes to the upstream file are limited to package, attribution, and formatting.
