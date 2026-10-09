package org.neiacademy.robotics.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.FieldConstants;
import org.neiacademy.robotics.frc2026.Presets;
import org.neiacademy.robotics.frc2026.subsystems.drive.*;
import org.neiacademy.robotics.frc2026.subsystems.hood.*;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.*;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.*;
import org.neiacademy.robotics.frc2026.subsystems.loader.*;
import org.neiacademy.robotics.frc2026.subsystems.shooter.*;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.*;

class SuperstructureFlywheelsTest {
  private static final CommandScheduler scheduler = CommandScheduler.getInstance();
  private static final RecordingShooterIO leftIO = new RecordingShooterIO();
  private static final RecordingShooterIO rightIO = new RecordingShooterIO();
  private static final double[] rollerVolts = new double[3];
  private static IntakeRoller intake;
  private static Spindexer spindexer;
  private static Loader loader;
  private static Drive drive;
  private static Shooter left;
  private static Superstructure superstructure;

  @BeforeAll
  static void setupRobot() {
    assertTrue(HAL.initialize(500, 0));
    Logger.AdvancedHooks.disableRobotBaseCheck();
    Logger.disableConsoleCapture();
    Logger.start();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    left = new Shooter(leftIO, true);
    Shooter right = new Shooter(rightIO, false);
    intake =
        new IntakeRoller(
            new IntakeRollerIO() {
              @Override
              public void runVoltage(double volts) {
                rollerVolts[0] = volts;
              }

              @Override
              public void stop() {
                rollerVolts[0] = 0;
              }
            });
    spindexer =
        new Spindexer(
            new SpindexerIO() {
              @Override
              public void runVoltage(double volts) {
                rollerVolts[1] = volts;
              }

              @Override
              public void stop() {
                rollerVolts[1] = 0;
              }
            });
    loader =
        new Loader(
            new LoaderIO() {
              @Override
              public void runVoltage(double volts) {
                rollerVolts[2] = volts;
              }

              @Override
              public void stop() {
                rollerVolts[2] = 0;
              }
            });
    superstructure =
        new Superstructure(
            drive,
            spindexer,
            new IntakeDeploy(new IntakeDeployIO() {}),
            intake,
            loader,
            left,
            right,
            new Hood(new HoodIO() {}));
  }

  @BeforeEach
  void reset() {
    scheduler.cancelAll();
    Presets.Shooter.IDLE_SPEED_RPM.set(300);
    leftIO.measuredVelocity = 0;
    rightIO.measuredVelocity = 0;
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    SmartDashboard.putBoolean("ConstantFlywheelsMode", false);
    SmartDashboard.putBoolean("FixedShooterMode", true);
    setHubDistance(2.0);
    runCycles();
    leftIO.velocityCalls = 0;
    rightIO.velocityCalls = 0;
  }

  @AfterAll
  static void cleanup() {
    scheduler.cancelAll();
    scheduler.unregisterAllSubsystems();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SmartDashboard.putBoolean("ConstantFlywheelsMode", false);
    SmartDashboard.putBoolean("FixedShooterMode", true);
    Constants.constantFlywheelsMode = false;
    Constants.fixedShooterMode = true;
    Logger.end();
  }

  @Test
  void dashboardEnablesBothFlywheelsFromRestAndDisablesThemAgain() {
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    runCycles();
    assertTrue(Constants.constantFlywheelsMode);
    assertTrue(leftIO.velocityCalls > 0);
    assertTrue(rightIO.velocityCalls > 0);
    assertEquals(Units.rotationsPerMinuteToRadiansPerSecond(300), leftIO.target, 1e-6);
    assertEquals(leftIO.target, rightIO.target, 1e-6);
    SmartDashboard.putBoolean("ConstantFlywheelsMode", false);
    runCycles();
    assertFalse(Constants.constantFlywheelsMode);
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
  }

  @Test
  void idleTargetIgnoresDistanceZoneAndShotTrimAndTracksIdlePreset() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    runCycles();
    double idle = leftIO.target;
    setHubDistance(3.0);
    runCycles();
    assertEquals(idle, leftIO.target, 1e-6);
    drive.setPose(
        new Pose2d(FieldConstants.LinesVertical.allianceZone + 1.0, 2.0, Rotation2d.kZero));
    runCycles();
    assertEquals(idle, leftIO.target, 1e-6);
    scheduler.schedule(superstructure.fudgeShooterSpeedShuttle(9));
    runCycles();
    assertEquals(idle, leftIO.target, 1e-6);
    scheduler.schedule(superstructure.resetShooterSpeedAdjustmentCommand());
    Presets.Shooter.IDLE_SPEED_RPM.set(500);
    runCycles();
    assertEquals(Units.rotationsPerMinuteToRadiansPerSecond(500), leftIO.target, 1e-6);
    assertEquals(leftIO.target, rightIO.target, 1e-6);
  }

  @Test
  void idleCoastsAboveFloorAndInvalidOrZeroTargetsStopOutput() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    leftIO.measuredVelocity = 290;
    rightIO.measuredVelocity = 290;
    runCycles();
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
    assertEquals(0, leftIO.velocityCalls);
    leftIO.measuredVelocity = Units.rotationsPerMinuteToRadiansPerSecond(299);
    rightIO.measuredVelocity = leftIO.measuredVelocity;
    runCycles();
    assertFalse(leftIO.stopped);
    assertTrue(leftIO.idleRequest);
    assertEquals(Units.rotationsPerMinuteToRadiansPerSecond(300), leftIO.target, 1e-6);
    Presets.Shooter.IDLE_SPEED_RPM.set(250);
    runCycles();
    assertTrue(leftIO.stopped);
    for (double invalid : new double[] {0, -100, Double.NaN, Double.POSITIVE_INFINITY}) {
      Presets.Shooter.IDLE_SPEED_RPM.set(invalid);
      runCycles();
      assertTrue(leftIO.stopped);
      assertTrue(rightIO.stopped);
    }
  }

  @Test
  void fixedShooterDashboardToggleChangesTheShotSolution() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    runCycles();
    double fixedTarget = superstructure.getHubShootingSetpointShooterSpeed();
    assertEquals(0.0, superstructure.getHubShootingSetpointHoodAngle(), 1e-6);
    SmartDashboard.putBoolean("FixedShooterMode", false);
    runCycles();
    assertFalse(Constants.fixedShooterMode);
    assertNotEquals(fixedTarget, superstructure.getHubShootingSetpointShooterSpeed());
    assertTrue(superstructure.getHubShootingSetpointHoodAngle() > 0.0);
    SmartDashboard.putBoolean("FixedShooterMode", true);
    runCycles();
    assertTrue(Constants.fixedShooterMode);
    assertEquals(fixedTarget, superstructure.getHubShootingSetpointShooterSpeed(), 1e-6);
  }

  @Test
  void shootingTakesOwnershipThenIdleResumesAndRobotDisableStopsOutput() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    runCycles();
    Command shot = left.runTrackedVelocityCommand(() -> 425.0);
    scheduler.schedule(shot);
    runCycles();
    assertTrue(shot.isScheduled());
    assertEquals(425.0, leftIO.target, 1e-6);
    assertFalse(leftIO.idleRequest);
    leftIO.measuredVelocity = 425;
    shot.cancel();
    runCycles();
    assertTrue(leftIO.stopped);
    leftIO.measuredVelocity = 30;
    runCycles();
    assertTrue(leftIO.idleRequest);
    assertEquals(Units.rotationsPerMinuteToRadiansPerSecond(300), leftIO.target, 1e-6);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    runCycles();
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
  }

  @Test
  void xLockDoesNotCancelHubOrShuttleShooterTracking() {
    boolean[] locked = {false};
    for (boolean shuttle : new boolean[] {false, true}) {
      Command aim =
          shuttle
              ? superstructure.shuttleAimCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> locked[0])
              : superstructure.hubAimCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> locked[0]);
      scheduler.schedule(aim);
      runCycles();
      assertTrue(aim.isScheduled());
      for (boolean lock : new boolean[] {true, false, true}) {
        int calls = leftIO.velocityCalls;
        locked[0] = lock;
        runCycles();
        assertTrue(aim.isScheduled());
        assertSame(aim, drive.getCurrentCommand());
        assertSame(aim, left.getCurrentCommand());
        assertTrue(leftIO.velocityCalls > calls);
        assertEquals(
            shuttle
                ? superstructure.getShuttleShootingSetpointShooterSpeed()
                : superstructure.getHubShootingSetpointShooterSpeed(),
            leftIO.target,
            1e-6);
      }
      aim.cancel();
      assertFalse(org.neiacademy.robotics.frc2026.commands.DriveCommands.atAngleSetpoint());
    }
  }

  @Test
  void lockedHeadingReadinessUsesLiveWrappedError() {
    var target = Rotation2d.fromDegrees(-179);
    Command aim =
        org.neiacademy.robotics.frc2026.commands.DriveCommands.joystickDriveAtAngle(
            drive, () -> 1.0, () -> 1.0, () -> target, () -> Rotation2d.kZero, () -> true);
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.fromDegrees(179)));
    scheduler.schedule(aim);
    runCycles();
    assertTrue(org.neiacademy.robotics.frc2026.commands.DriveCommands.atAngleSetpoint());
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
    runCycles();
    assertFalse(org.neiacademy.robotics.frc2026.commands.DriveCommands.atAngleSetpoint());
    aim.cancel();
  }

  @Test
  void heldStopOverridesActiveMotorsRejectsNewShotsAndReleasesToIdle() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    Command rollers =
        edu.wpi.first.wpilibj2.command.Commands.parallel(
            intake.runVoltageCommand(() -> 8),
            spindexer.runVoltageCommand(() -> 8),
            loader.runVoltageCommand(() -> 8));
    scheduler.schedule(rollers);
    runCycles();
    assertArrayEquals(new double[] {8, 8, 8}, rollerVolts);
    assertFalse(leftIO.stopped);
    assertFalse(rightIO.stopped);
    Command stop = superstructure.holdBallHandlingStoppedCommand();
    scheduler.schedule(stop);
    assertFalse(rollers.isScheduled());
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
    assertArrayEquals(new double[] {0, 0, 0}, rollerVolts);
    Command shot = left.runTrackedVelocityCommand(() -> 425);
    scheduler.schedule(shot);
    scheduler.schedule(rollers);
    runCycles();
    assertTrue(stop.isScheduled());
    assertFalse(shot.isScheduled());
    assertFalse(rollers.isScheduled());
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
    assertArrayEquals(new double[] {0, 0, 0}, rollerVolts);
    stop.cancel();
    runCycles();
    assertFalse(leftIO.stopped);
    assertFalse(rightIO.stopped);
    scheduler.schedule(rollers);
    runCycles();
    assertArrayEquals(new double[] {8, 8, 8}, rollerVolts);
  }

  @Test
  void resetRestoresOnlyCurrentZonesStartupAdjustment() {
    double hub = superstructure.getHubShootingSetpointShooterSpeed();
    double shuttle = superstructure.getShuttleShootingSetpointShooterSpeed();
    scheduler.schedule(superstructure.fudgeShooterSpeedShoot(5));
    scheduler.schedule(superstructure.fudgeShooterSpeedShuttle(9));
    runCycles();
    assertEquals(hub + 5, superstructure.getHubShootingSetpointShooterSpeed(), 1e-6);
    assertEquals(shuttle + 9, superstructure.getShuttleShootingSetpointShooterSpeed(), 1e-6);
    scheduler.schedule(superstructure.resetShooterSpeedAdjustmentCommand());
    runCycles();
    assertEquals(hub, superstructure.getHubShootingSetpointShooterSpeed(), 1e-6);
    assertEquals(shuttle + 9, superstructure.getShuttleShootingSetpointShooterSpeed(), 1e-6);
    drive.setPose(
        new Pose2d(FieldConstants.LinesVertical.allianceZone + 1.0, 2.0, Rotation2d.kZero));
    runCycles();
    double adjustedShuttle = superstructure.getShuttleShootingSetpointShooterSpeed();
    scheduler.schedule(superstructure.resetShooterSpeedAdjustmentCommand());
    runCycles();
    assertEquals(
        adjustedShuttle - 9, superstructure.getShuttleShootingSetpointShooterSpeed(), 1e-6);
    setHubDistance(2.0);
    runCycles();
    assertEquals(hub, superstructure.getHubShootingSetpointShooterSpeed(), 1e-6);
  }

  @Test
  void idleSpeedCannotSatisfyShootingReadiness() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    leftIO.measuredVelocity = Units.rotationsPerMinuteToRadiansPerSecond(300);
    runCycles();
    assertFalse(left.atSetpoint());
    Command spinUp = left.runVelocityCommand(() -> 290);
    scheduler.schedule(spinUp);
    assertFalse(left.atSetpoint());
    runCycles();
    assertTrue(spinUp.isScheduled());
    leftIO.measuredVelocity = 290;
    runCycles();
    assertFalse(spinUp.isScheduled());
  }

  private static void setHubDistance(double distance) {
    drive.setPose(
        new Pose2d(
            FieldConstants.Hub.innerCenterPoint.getX() - distance,
            FieldConstants.Hub.innerCenterPoint.getY(),
            Rotation2d.kZero));
  }

  private static void runCycles() {
    for (int i = 0; i < 3; i++) {
      Logger.AdvancedHooks.invokePeriodicBeforeUser();
      scheduler.run();
    }
  }

  private static class RecordingShooterIO implements ShooterIO {
    double target;
    double measuredVelocity;
    boolean idleRequest;
    int velocityCalls;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      inputs.leaderVelocityRadsPerSec = measuredVelocity;
    }

    @Override
    public void runIdleVelocity(double velocityRadsPerSec) {
      runVelocity(velocityRadsPerSec);
      idleRequest = true;
    }

    boolean stopped = true;

    @Override
    public void runVelocity(double velocityRadsPerSec) {
      target = velocityRadsPerSec;
      idleRequest = false;
      velocityCalls++;
      stopped = false;
    }

    @Override
    public void stop() {
      stopped = true;
    }
  }
}
