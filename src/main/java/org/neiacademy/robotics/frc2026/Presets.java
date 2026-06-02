package org.neiacademy.robotics.frc2026;

import org.neiacademy.robotics.frc2026.generated.TunerConstants;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;

public final class Presets {
  public static class Drive {
    public static final LoggedTunableNumber DRIVE_kP = TunerConstants.DRIVE_kP;
    public static final LoggedTunableNumber DRIVE_kI = TunerConstants.DRIVE_kI;
    public static final LoggedTunableNumber DRIVE_kD = TunerConstants.DRIVE_kD;
    public static final LoggedTunableNumber DRIVE_kS = TunerConstants.DRIVE_kS;
    public static final LoggedTunableNumber DRIVE_kV = TunerConstants.DRIVE_kV;
    public static final LoggedTunableNumber DRIVE_kA = TunerConstants.DRIVE_kA;

    public static final LoggedTunableNumber STEER_kP = TunerConstants.STEER_kP;
    public static final LoggedTunableNumber STEER_kI = TunerConstants.STEER_kI;
    public static final LoggedTunableNumber STEER_kD = TunerConstants.STEER_kD;
    public static final LoggedTunableNumber STEER_kS = TunerConstants.STEER_kS;
    public static final LoggedTunableNumber STEER_kV = TunerConstants.STEER_kV;
    public static final LoggedTunableNumber STEER_kA = TunerConstants.STEER_kA;
  }

  public static class Spindexer {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/FeedVolts", 12.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Spindexer/ExhaustVolts", -12.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Tuning/SpindexerTuningVolts", 0.0);
  }

  public static class Intake {
    public static final LoggedTunableNumber TUCK_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployTuckAngleDeg", 0.4);
    public static final LoggedTunableNumber EXTEND_ANGLE_DEG =
        new LoggedTunableNumber("Setpoints/Intake/DeployExtendAngleDeg", -0.2);
    public static final LoggedTunableNumber TUNING_ANGLE_DEG =
        new LoggedTunableNumber("Tuning/IntakeDeployTuningAngleDeg", 0.0);
    public static final LoggedTunableNumber PIVOT_MANUAL_MOVEMENT_TOTAL_TIME =
        new LoggedTunableNumber("Setpoints/Intake/PivotManualMovementTotalTime", 1.5);

    public static final LoggedTunableNumber INTAKE_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerIntakeVolts", 7.5);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Intake/RollerExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Tuning/IntakeRollerTuningVolts", 0.0);

    public static final LoggedTunableNumber SHOOTING_TOGGLE_SPEED_SEC =
        new LoggedTunableNumber("Setpoints/Intake/ShootingToggleSpeedSec", 0.1);
    public static final LoggedTunableNumber SHOOTING_TOGGLE_TIMEOUT_SPEED_SEC =
        new LoggedTunableNumber("Tuning/IntakeShootingToggleTimeoutSpeedSec", 0.4);
  }

  public static class Loader {
    public static final LoggedTunableNumber FEED_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/FeedVolts", 12.0);
    public static final LoggedTunableNumber EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/ExhaustVolts", -12);
    public static final LoggedTunableNumber SLOW_EXHAUST_VOLTS =
        new LoggedTunableNumber("Setpoints/Loader/SlowExhaustVolts", -6.0);
    public static final LoggedTunableNumber TUNING_VOLTS =
        new LoggedTunableNumber("Tuning/LoaderTuningVolts", 0.0);
  }

  public static class Shooter {
    public static final LoggedTunableNumber CLOSE_HUB_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/CloseHubSpeedRadsPerSec", 290.0);
    public static final LoggedTunableNumber TRENCH_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/TrenchSpeedRadsPerSec", 290.0);
    public static final LoggedTunableNumber CORNER_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/CornerSpeedRadsPerSec", 290.0);
    public static final LoggedTunableNumber TOWER_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/TowerSpeedRadsPerSec", 290.0);
    public static final LoggedTunableNumber HALF_SHUTTLE_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/HalfShuttleSpeedRadsPerSec", 425.0);
    public static final LoggedTunableNumber FULL_SHUTTLE_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/FullShuttleSpeedRadsPerSec", 550.0);
    public static final LoggedTunableNumber TUNING_SPEED =
        new LoggedTunableNumber("Setpoints/Shooter/TuningSpeedRadsPerSec", 0.0);
    public static final LoggedTunableNumber NO_SPEED =
        new LoggedTunableNumber("Tuning/ShooterTuningSpeedRadsPerSec", 0.0);
  }

  public static class Hood {
    public static final LoggedTunableNumber TUNING_POSITION =
        new LoggedTunableNumber("Tuning/HoodTuningPos", 0);
    public static final LoggedTunableNumber TUCK_POSITION =
        new LoggedTunableNumber("Setpoints/Hood/TuckPos", 0.49);
    public static final LoggedTunableNumber CLOSE_HUB_POSITION =
        new LoggedTunableNumber("Setpoints/Hood/CloseHubPos", 0.05);
  }
}
