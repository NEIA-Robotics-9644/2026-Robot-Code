package org.neiacademy.robotics.frc2026.sim;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Presets;
import org.neiacademy.robotics.frc2026.subsystems.hood.HoodIO;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIO;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIO;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIO;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIO;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIO;

/** Simulation-only mechanism response and fuel transport. Launch parameters require calibration. */
public final class PhoebeSim {
  public static final int CAPACITY = 40;
  // User-selected average feed rate; the simulator smooths burst/gap behavior.
  private static final double DEFAULT_FUELS_PER_SECOND = 8.0;
  private static final double DT = 0.02;
  private final FuelSim fuel;
  private final Supplier<Pose2d> pose;
  private int stored = 8;
  private int launched;
  private double feedTime;
  private double rollerVolts, loaderVolts, spindexerVolts;
  private double pivot = Presets.Intake.TUCK_ANGLE_DEG.get(), pivotTarget = pivot;
  private double hood, hoodTarget;
  public final SimShooter leftShooter = new SimShooter();
  public final SimShooter rightShooter = new SimShooter();

  public PhoebeSim(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> robotSpeeds) {
    this(pose, robotSpeeds, new FuelSim("FuelSim"));
  }

  PhoebeSim(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> robotSpeeds, FuelSim fuel) {
    this.pose = pose;
    this.fuel = fuel;
    fuel.registerRobot(
        0.879,
        0.879,
        0.20,
        pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds.get(), pose.get().getRotation()));
    // Intake and shooter are on robot +X. Rectangle extends beyond the front bumper.
    fuel.registerIntake(0.44, 0.85, -0.40, 0.40, this::canIntake, () -> stored++);
    SmartDashboard.setDefaultNumber("FuelSim/WheelRadiusMeters", 0.0508);
    SmartDashboard.setDefaultNumber("FuelSim/ExitSpeedRatio", 0.50);
    SmartDashboard.setDefaultNumber("FuelSim/LaunchHeightMeters", 0.65);
    SmartDashboard.setDefaultNumber("FuelSim/HoodRetractedDegrees", 70);
    SmartDashboard.setDefaultNumber("FuelSim/HoodExtendedDegrees", 45);
    SmartDashboard.setDefaultNumber("FuelSim/FuelsPerSecond", DEFAULT_FUELS_PER_SECOND);
    reset();
    fuel.start();
  }

  public void reset() {
    fuel.clearFuel();
    fuel.spawnStartingFuel();
    FuelSim.Hub.BLUE_HUB.resetScore();
    FuelSim.Hub.RED_HUB.resetScore();
    stored = 8;
    launched = 0;
    feedTime = 0;
  }

  public int getStoredFuel() {
    return stored;
  }

  public int getLaunchedFuel() {
    return launched;
  }

  private boolean canIntake() {
    return DriverStation.isEnabled()
        && stored < CAPACITY
        && rollerVolts > 1
        && Math.abs(pivot - Presets.Intake.EXTEND_ANGLE_DEG.get()) < 0.08;
  }

  public void update() {
    boolean enabled = DriverStation.isEnabled();
    if (enabled) {
      pivot += MathUtil.clamp(pivotTarget - pivot, -DT * 2, DT * 2);
      hood += MathUtil.clamp(hoodTarget - hood, -DT, DT);
    }
    leftShooter.update(enabled);
    rightShooter.update(enabled);
    // Only actual forward feeding launches fuel; reverse unjam cannot shoot.
    if (enabled
        && stored > 0
        && loaderVolts > 1
        && spindexerVolts > 1
        && leftShooter.velocity > 100
        && rightShooter.velocity > 100) {
      feedTime += DT;
      double interval =
          1
              / MathUtil.clamp(
                  SmartDashboard.getNumber("FuelSim/FuelsPerSecond", DEFAULT_FUELS_PER_SECOND),
                  1,
                  30);
      while (stored > 0 && feedTime >= interval) {
        feedTime -= interval;
        double speed =
            (leftShooter.velocity + rightShooter.velocity)
                / 2
                * SmartDashboard.getNumber("FuelSim/WheelRadiusMeters", 0.0508)
                * SmartDashboard.getNumber("FuelSim/ExitSpeedRatio", 0.50);
        double angle =
            MathUtil.interpolate(
                SmartDashboard.getNumber("FuelSim/HoodRetractedDegrees", 70),
                SmartDashboard.getNumber("FuelSim/HoodExtendedDegrees", 45),
                hood);
        fuel.launchFuel(
            MetersPerSecond.of(speed),
            Degrees.of(angle),
            Degrees.zero(),
            Meters.of(Math.max(0.3, SmartDashboard.getNumber("FuelSim/LaunchHeightMeters", 0.65))));
        stored--;
        launched++;
      }
    } else {
      feedTime = 0;
    }
    fuel.updateSim();
    Logger.recordOutput("FuelSim/Robot", new Pose3d(pose.get()));
    Logger.recordOutput("FuelSim/StoredFuel", stored);
    Logger.recordOutput("FuelSim/LaunchedFuel", launched);
    Logger.recordOutput("FuelSim/BlueScore", FuelSim.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("FuelSim/RedScore", FuelSim.Hub.RED_HUB.getScore());
  }

  /** Bounded first-order flywheel response, not a calibrated motor/inertia model. */
  public static final class SimShooter implements ShooterIO {
    private double target, velocity;
    private boolean idle;

    private void update(boolean enabled) {
      double requested = enabled ? target : 0;
      if (enabled && idle && velocity > requested) velocity *= Math.exp(-DT / 1.5);
      else velocity += (requested - velocity) * (1 - Math.exp(-DT / 0.15));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.leaderConnected = inputs.followerConnected = true;
      inputs.leaderVelocityRadsPerSec = inputs.followerVelocityRadsPerSec = velocity;
      inputs.leaderVelocitySetpointRadsPerSec = target;
      inputs.supplyVoltageVolts = 12;
    }

    @Override
    public void runVelocity(double value) {
      target = MathUtil.clamp(value, 0, 650);
      idle = false;
    }

    @Override
    public void runIdleVelocity(double value) {
      runVelocity(value);
      idle = true;
    }

    @Override
    public void runVoltage(double volts) {
      runVelocity(volts / 12 * 650);
    }

    @Override
    public void stop() {
      target = 0;
      idle = false;
    }
  }

  public final IntakeDeployIO intakeDeploy =
      new IntakeDeployIO() {
        @Override
        public void updateInputs(IntakeDeployIOInputs inputs) {
          inputs.motorConnected = inputs.cancoderConnected = true;
          inputs.rotorPositionRads = Units.rotationsToRadians(pivot);
          inputs.positionSetpointRads = Units.rotationsToRadians(pivotTarget);
        }

        @Override
        public void runPosition(double rotations) {
          pivotTarget = rotations;
        }

        @Override
        public void runVoltage(double volts) {
          if (volts > 0) pivotTarget = Presets.Intake.EXTEND_ANGLE_DEG.get();
          else if (volts < 0) pivotTarget = Presets.Intake.TUCK_ANGLE_DEG.get();
        }

        @Override
        public void stop() {
          pivotTarget = pivot;
        }
      };
  public final HoodIO hoodIO =
      new HoodIO() {
        @Override
        public void updateInputs(HoodIOInputs inputs) {
          inputs.leftNormalizedPosition = inputs.rightNormalizedPosition = hood;
          inputs.leftSetpointPosition = inputs.rightSetpointPosition = hoodTarget;
        }

        @Override
        public void setPositionNormalized(double value) {
          hoodTarget = MathUtil.clamp(value, 0, 1);
        }

        @Override
        public void setSpeedNormalized(double value) {
          setPositionNormalized(hoodTarget + value * DT);
        }

        @Override
        public void resetPosition() {
          hoodTarget = hood = 0;
        }

        @Override
        public void zeroPosition() {
          resetPosition();
        }

        @Override
        public boolean isPositionWithinTolerance() {
          return Math.abs(hood - hoodTarget) < 0.02;
        }

        @Override
        public void stop() {
          hoodTarget = hood;
        }
      };
  public final IntakeRollerIO intakeRoller =
      new IntakeRollerIO() {
        @Override
        public void runVoltage(double value) {
          rollerVolts = value;
        }

        @Override
        public void stop() {
          rollerVolts = 0;
        }

        @Override
        public void updateInputs(IntakeRollerIOInputs inputs) {
          inputs.connected = true;
          inputs.appliedVolts = DriverStation.isEnabled() ? rollerVolts : 0;
        }
      };
  public final LoaderIO loader =
      new LoaderIO() {
        @Override
        public void runVoltage(double value) {
          loaderVolts = value;
        }

        @Override
        public void stop() {
          loaderVolts = 0;
        }

        @Override
        public void updateInputs(LoaderIOInputs inputs) {
          inputs.connected = true;
          inputs.appliedVolts = DriverStation.isEnabled() ? loaderVolts : 0;
        }
      };
  public final SpindexerIO spindexer =
      new SpindexerIO() {
        @Override
        public void runVoltage(double value) {
          spindexerVolts = value;
        }

        @Override
        public void stop() {
          spindexerVolts = 0;
        }

        @Override
        public void updateInputs(SpindexerIOInputs inputs) {
          inputs.connected = true;
          inputs.appliedVolts = DriverStation.isEnabled() ? spindexerVolts : 0;
        }
      };
}
