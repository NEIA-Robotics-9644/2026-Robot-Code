package org.neiacademy.robotics.frc2026.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.FieldConstants;
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
  private static Drive drive;
  private static Shooter left;
  private static Superstructure superstructure;

  @BeforeAll
  static void setupRobot() {
    assertTrue(HAL.initialize(500, 0));
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
    superstructure =
        new Superstructure(
            drive,
            new Spindexer(new SpindexerIO() {}),
            new IntakeDeploy(new IntakeDeployIO() {}),
            new IntakeRoller(new IntakeRollerIO() {}),
            new Loader(new LoaderIO() {}),
            left,
            right,
            new Hood(new HoodIO() {}));
  }

  @BeforeEach
  void reset() {
    scheduler.cancelAll();
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
    assertEquals(284.0, leftIO.target, 1e-6); // 290 rad/s table value, minus 6 adjustment.
    assertEquals(leftIO.target, rightIO.target, 1e-6);
    SmartDashboard.putBoolean("ConstantFlywheelsMode", false);
    runCycles();
    assertFalse(Constants.constantFlywheelsMode);
    assertTrue(leftIO.stopped);
    assertTrue(rightIO.stopped);
  }

  @Test
  void idleTargetTracksDistanceAndSwitchesToShuttleOutsideAllianceZone() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    runCycles();
    double closeTarget = leftIO.target;
    setHubDistance(3.0);
    runCycles();
    assertEquals(315.0, leftIO.target, 1e-6);
    assertNotEquals(closeTarget, leftIO.target);
    drive.setPose(
        new Pose2d(FieldConstants.LinesVertical.allianceZone + 1.0, 2.0, Rotation2d.kZero));
    runCycles();
    assertEquals(superstructure.getShuttleShootingSetpointShooterSpeed(), leftIO.target, 1e-6);
    assertEquals(leftIO.target, rightIO.target, 1e-6);
  }

  @Test
  void fixedShooterDashboardToggleChangesTheShotSolution() {
    SmartDashboard.putBoolean("ConstantFlywheelsMode", true);
    runCycles();
    double fixedTarget = leftIO.target;
    assertEquals(0.0, superstructure.getHubShootingSetpointHoodAngle(), 1e-6);
    SmartDashboard.putBoolean("FixedShooterMode", false);
    runCycles();
    assertFalse(Constants.fixedShooterMode);
    assertNotEquals(fixedTarget, leftIO.target);
    assertTrue(superstructure.getHubShootingSetpointHoodAngle() > 0.0);
    SmartDashboard.putBoolean("FixedShooterMode", true);
    runCycles();
    assertTrue(Constants.fixedShooterMode);
    assertEquals(fixedTarget, leftIO.target, 1e-6);
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
    shot.cancel();
    runCycles();
    assertEquals(superstructure.getHubShootingSetpointShooterSpeed(), leftIO.target, 1e-6);
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

  private static void setHubDistance(double distance) {
    drive.setPose(
        new Pose2d(
            FieldConstants.Hub.innerCenterPoint.getX() - distance,
            FieldConstants.Hub.innerCenterPoint.getY(),
            Rotation2d.kZero));
  }

  private static void runCycles() {
    for (int i = 0; i < 3; i++) scheduler.run();
  }

  private static class RecordingShooterIO implements ShooterIO {
    double target;
    int velocityCalls;
    boolean stopped = true;

    @Override
    public void runVelocity(double velocityRadsPerSec) {
      target = velocityRadsPerSec;
      velocityCalls++;
      stopped = false;
    }

    @Override
    public void stop() {
      stopped = true;
    }
  }
}
