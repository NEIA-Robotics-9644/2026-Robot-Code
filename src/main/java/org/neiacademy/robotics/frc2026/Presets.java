package org.neiacademy.robotics.frc2026;

import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public final class Presets {

  public static class Spindexer {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/FeedVolts", 12.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/ExhaustVolts", -12.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/TuningVolts", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployTuckAngleDeg", -126);
    public static final LoggedTunableNumber EXTEND_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployExtendAngleDeg", 0);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployTuningAngleDeg", 0.0);

    public static final LoggedTunableNumber INTAKE_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerIntakeVolts", 7.5);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerTuningVolts", 0.0);
  }

  public static class Loader {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/FeedVolts", 12.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/ExhaustVolts", -12);
    public static final LoggedTunableNumber SLOW_EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/SlowExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/TuningVolts", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/CloseHubSpeedRadsPerSec", 300.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/TuningSpeedRadsPerSec", 0.0);
  }
}
