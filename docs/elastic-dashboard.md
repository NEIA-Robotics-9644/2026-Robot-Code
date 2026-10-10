# Elastic driver dashboard

Open `layouts/elastic-layout.json` in Elastic, then connect to the robot or local
simulator. The Teleoperated tab includes the existing alerts and auto chooser plus:

| Widget | NetworkTables topic | Data |
| --- | --- | --- |
| Swerve Drive | `/SmartDashboard/Swerve Drive` | Measured module angles and speeds; estimated robot heading |
| Field | `/SmartDashboard/Field` | Estimated robot pose on the 2026 Rebuilt field |
| Match Time | `/SmartDashboard/Match Time` | Driver Station match time in seconds |
| FMS Info | `/FMSInfo` | WPILib's automatically published match, alliance, enable, and connection information |

`RobotDashboard` owns the widget publishers. `Robot.robotPeriodic()` updates them
through `RobotContainer.updateDashboard()` after the command scheduler has refreshed
the subsystem inputs, including while disabled. Swerve angles are CCW-positive
radians, speeds are meters per second, and module order is FL, FR, BL, BR. The field
uses the estimator's field coordinates without applying an alliance flip.

Match time preserves the Driver Station's `-1` value when unavailable. For a local
countdown, use the simulated Driver Station's match timing controls. The field shows
the estimated pose, which can differ from the simulated robot's ground truth.

To add the widgets manually, drag the topics above into Elastic and select their
corresponding widget types. Use Radians for Swerve Drive and Rebuilt for Field.

References: [Elastic widget properties](https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference)
and [swerve sendable example](https://frc-elastic.gitbook.io/docs/additional-features-and-references/custom-widget-examples).
