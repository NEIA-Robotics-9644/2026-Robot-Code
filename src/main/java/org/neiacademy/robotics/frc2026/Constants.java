package org.neiacademy.robotics.frc2026;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Setter;
import org.neiacademy.robotics.frc2026.util.LoggedTunableNumber;
import org.neiacademy.robotics.frc2026.util.drivers.CANDeviceID;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  @Setter public static boolean tuningMode = true;

  @Setter public static boolean manualMode = false;

  public static final boolean disableHAL = false;

  public static final CANDeviceID kPDH_ID = new CANDeviceID(0, "rio");

  public static final double loopTime = 0.02;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Spindexer {
    public static final String CANBUS = "rio";
    public static final CANDeviceID MOTOR_ID = new CANDeviceID(30, "Drive");

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 40.0;
    public static final double SUPPLY_LIMIT = 40.0;

    public static final double GEAR_RATIO = 5.0;
  }

  public static class Intake {
    public static final String CANBUS = "Drive";
    public static final CANDeviceID DEPLOY_MOTOR_ID = new CANDeviceID(32, "Drive");
    public static final CANDeviceID ENCODER_ID = new CANDeviceID(35, "Drive");
    public static final CANDeviceID ROLLER_MOTOR_ID = new CANDeviceID(33, "Drive");

    public static final InvertedValue DEPLOY_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final SensorDirectionValue ENCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;
    public static final double ENCODER_OFFSET = 0.008;
    public static final double ENCODER_DISCONTINUITY_POINT = 0.5;

    public static final double DEPLOY_STATOR_LIMIT = 80.0;
    public static final double DEPLOY_SUPPLY_LIMIT = 40.0;

    public static final double ROLLER_STATOR_LIMIT = 80.0;
    public static final double ROLLER_SUPPLY_LIMIT = 60.0;

    public static final double SOFT_LIMIT_FORWARD = 0.277;
    public static final double SOFT_LIMIT_REVERSE = -0.2;

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("PID/Intake/kP", 80);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("PID/Intake/kD", 15);
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("PID/Intake/kS", 0.0);
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("PID/Intake/kG", 0);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("PID/Intake/kV", 0.0);
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("PID/Intake/kA", 0.0);

    public static final LoggedTunableNumber POSITION_TOLERANCE =
        new LoggedTunableNumber("PID/Intake/DeployToleranceDeg", 3.0);

    public static final double ROLLER_GEAR_RATIO = 18 / 24;
    public static final double DEPLOY_GEAR_RATIO = 20 * (40 / 18);

    public static final Rotation2d GRAVITY_POSTION_OFFSET = Rotation2d.fromDegrees(90.0);
  }

  public static class Loader {
    public static final String CANBUS = "rio";
    public static final CANDeviceID MOTOR_ID = new CANDeviceID(20, "rio");

    public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 80.0;
    public static final double SUPPLY_LIMIT = 80.0;

    public static final double GEAR_RATIO = 3.0;
  }

  public static class Shooter {
    public static final String CANBUS = "rio";
    public static final CANDeviceID LEFT_SHOOTER_LEADER_ID = new CANDeviceID(25, "rio");
    public static final CANDeviceID LEFT_SHOOTER_FOLLOWER_ID = new CANDeviceID(24, "rio");
    public static final CANDeviceID RIGHT_SHOOTER_LEADER_ID = new CANDeviceID(21, "rio");
    public static final CANDeviceID RIGHT_SHOOTER_FOLLOWER_ID = new CANDeviceID(22, "rio");

    public static final InvertedValue LEFT_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue RIGHT_INVERTED = InvertedValue.CounterClockwise_Positive;

    public static final double STATOR_LIMIT = 80.0;
    public static final double SUPPLY_LIMIT = 80.0;

    public static final LoggedTunableNumber LEFT_kP =
        new LoggedTunableNumber("PID/Shooter/Left/kP", 10.0);
    public static final LoggedTunableNumber LEFT_kD =
        new LoggedTunableNumber("PID/Shooter/Left/kD", 0.0);
    public static final LoggedTunableNumber LEFT_kS =
        new LoggedTunableNumber("PID/Shooter/Left/kS", 5.2);
    public static final LoggedTunableNumber LEFT_kV =
        new LoggedTunableNumber("PID/Shooter/Left/kV", 0.043);
    public static final LoggedTunableNumber LEFT_kA =
        new LoggedTunableNumber("PID/Shooter/Left/kA", 0.0);

    public static final LoggedTunableNumber RIGHT_kP =
        new LoggedTunableNumber("PID/Shooter/Right/kP", 10.0);
    public static final LoggedTunableNumber RIGHT_kD =
        new LoggedTunableNumber("PID/Shooter/Right/kD", 0.0);
    public static final LoggedTunableNumber RIGHT_kS =
        new LoggedTunableNumber("PID/Shooter/Right/kS", 5.2);
    public static final LoggedTunableNumber RIGHT_kV =
        new LoggedTunableNumber("PID/Shooter/Right/kV", 0.043);
    public static final LoggedTunableNumber RIGHT_kA =
        new LoggedTunableNumber("PID/Shooter/Right/kA", 0.0);

    public static final LoggedTunableNumber VELOCITY_TOLERANCE =
        new LoggedTunableNumber("PID/Shooter/ToleranceRadsPerSec", 35.0);

    public static final double GEAR_RATIO = 1;
  }

  public static class Hood {
    public static final int LEFT_CHANNEL = 0;
    public static final int RIGHT_CHANNEL = 1;

    public static final int LINEAR_ACTUATOR_MAX = 2000;
    public static final int LINEAR_ACTUATOR_DEADBAND_MAX = 1800;
    public static final int LINEAR_ACTUATOR_CENTER = 1500;
    public static final int LINEAR_ACTUATOR_DEADBAND_MIN = 1300;
    public static final int LINEAR_ACTUATOR_MIN = 1000;

    public static final Distance LENGTH_MILLIMETERS = Millimeters.of(140);
    public static final LinearVelocity MAX_SPEED =
        Millimeters.of(60)
            .per(Second); // Tripled from 20 — software position estimate only, does not limit
    // physical servo speed

    public static final double MIN_POSITION = 0.01;
    public static final double MAX_POSITION = 0.5;
    public static final double POSITION_TOLERANCE = 0.01;
    public static final double INITIAL_POSITION = 0;
  }
}
