package org.neiacademy.robotics.frc2026.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.FieldConstants;
import org.neiacademy.robotics.frc2026.Presets;
import org.neiacademy.robotics.frc2026.commands.DriveCommands;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeploy;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRoller;
import org.neiacademy.robotics.frc2026.subsystems.loader.Loader;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.Spindexer;
import org.neiacademy.robotics.frc2026.util.AllianceFlipUtil;
import org.neiacademy.robotics.frc2026.util.ShootingUtil;
import org.neiacademy.robotics.frc2026.util.ShootingUtil.ShooterSetpoint;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;

  private ShooterSetpoint hubShootingSetpoint;
  private ShooterSetpoint shuttleShootingSetpoint;

  public Superstructure(
      Drive drive,
      Spindexer spindexer,
      IntakeDeploy intakeDeploy,
      IntakeRoller intakeRoller,
      Loader loader,
      Shooter leftShooter,
      Shooter rightShooter) {
    this.drive = drive;
    this.spindexer = spindexer;
    this.intakeDeploy = intakeDeploy;
    this.intakeRoller = intakeRoller;
    this.loader = loader;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;

    hubShootingSetpoint =
        ShootingUtil.makeHubSetpoint(
            drive,
            AllianceFlipUtil.apply(
                new Pose2d(
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.kZero)));
    shuttleShootingSetpoint = ShootingUtil.makeShuttleSetpoint(drive, getShuttleTargetPose());
    leftShooter.setDefaultCommand(leftShooter.stopCommand());
    rightShooter.setDefaultCommand(rightShooter.stopCommand());
  }

  @Override
  public void periodic() {
    hubShootingSetpoint =
        ShootingUtil.makeHubSetpoint(
            drive,
            AllianceFlipUtil.apply(
                new Pose2d(
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.kZero)));
    shuttleShootingSetpoint = ShootingUtil.makeShuttleSetpoint(drive, getShuttleTargetPose());

    Logger.recordOutput("DriveCommands/atAngleSetpoint", DriveCommands.atAngleSetpoint());
    Logger.recordOutput(
        "DriveCommands/atDriveToPoseSetpoint", DriveCommands.atDriveToPoseSetpoint());
  }

  public Command hubAimCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return new ParallelCommandGroup(
        DriveCommands.joystickDriveAtAngle(
            drive,
            driveXSupplier,
            driveYSupplier,
            this::getHubShootingSetpointDriveAngle,
            this::getHubShootingSetpointDriveVelocity),
        leftShooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed),
        rightShooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed));
  }

  public Command shuttleAimCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return new ParallelCommandGroup(
        DriveCommands.joystickDriveAtAngle(
            drive,
            driveXSupplier,
            driveYSupplier,
            this::getShuttleShootingSetpointDriveAngle,
            this::getShuttleShootingSetpointDriveVelocity),
        leftShooter.runTrackedVelocityCommand(this::getShuttleShootingSetpointShooterSpeed),
        rightShooter.runTrackedVelocityCommand(this::getShuttleShootingSetpointShooterSpeed));
  }

  public Command shootCommand() {
    return new ParallelCommandGroup(
        spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
        loader.runVoltageCommand(Presets.Loader.FEED_VOLTS));
  }

  public Command endShootCommand() {
    return new SequentialCommandGroup(
        loader.runVoltageCommand(Presets.Loader.SLOW_EXHAUST_VOLTS).withTimeout(0.5),
        new ParallelCommandGroup(spindexer.stopCommand(), loader.stopCommand()));
  }

  public Command deployIntake() {
    return intakeDeploy.runPositionCommand(Presets.Intake.EXTEND_ANGLE_DEG);
  }

  public Command retractIntake() {
    return intakeDeploy.runPositionCommand(Presets.Intake.TUCK_ANGLE_DEG);
  }

  public Command stopAllRollersCommand() {
    return new ParallelCommandGroup(
        intakeRoller.stopCommand(),
        spindexer.stopCommand(),
        loader.stopCommand(),
        leftShooter.stopCommand(),
        rightShooter.stopCommand());
  }

  public Pose2d getShuttleTargetPose() {
    Pose2d robotPose = AllianceFlipUtil.apply(drive.getPose());
    return AllianceFlipUtil.apply(
        new Pose2d(
            (robotPose.getY() <= FieldConstants.fieldWidth / 2)
                ? FieldConstants.RightBump.centerPoint
                : FieldConstants.LeftBump.centerPoint,
            Rotation2d.kZero));
  }

  public Command autoShoot() {
    return new ParallelCommandGroup(
        hubAimCommand(() -> 0.0, () -> 0.0),
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> leftShooter.atSetpoint() && rightShooter.atSetpoint()),
            new ParallelCommandGroup(
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS))));
  }

  public Rotation2d getHubShootingSetpointDriveAngle() {
    return hubShootingSetpoint.driveAngleRads();
  }

  public Rotation2d getHubShootingSetpointDriveVelocity() {
    return hubShootingSetpoint.driveVelocityRadsPerSec();
  }

  public double getHubShootingSetpointShooterSpeed() {
    return hubShootingSetpoint.shooterSpeedRadsPerSec();
  }

  public Rotation2d getShuttleShootingSetpointDriveAngle() {
    return shuttleShootingSetpoint.driveAngleRads();
  }

  public Rotation2d getShuttleShootingSetpointDriveVelocity() {
    return shuttleShootingSetpoint.driveVelocityRadsPerSec();
  }

  public double getShuttleShootingSetpointShooterSpeed() {
    return shuttleShootingSetpoint.shooterSpeedRadsPerSec();
  }
}
