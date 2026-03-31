package org.neiacademy.robotics.frc2026.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import org.neiacademy.robotics.frc2026.subsystems.hood.Hood;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeploy;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRoller;
import org.neiacademy.robotics.frc2026.subsystems.loader.Loader;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.Spindexer;
import org.neiacademy.robotics.frc2026.util.AllianceFlipUtil;
import org.neiacademy.robotics.frc2026.util.ShootingUtilNew;

public class SuperstructureNew extends SubsystemBase {
  private final Drive drive;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Hood hood;

  public SuperstructureNew(
      Drive drive,
      Spindexer spindexer,
      IntakeDeploy intakeDeploy,
      IntakeRoller intakeRoller,
      Loader loader,
      Shooter leftShooter,
      Shooter rightShooter,
      Hood hood) {
    this.drive = drive;
    this.spindexer = spindexer;
    this.intakeDeploy = intakeDeploy;
    this.intakeRoller = intakeRoller;
    this.loader = loader;
    this.leftShooter = leftShooter;
    this.rightShooter = rightShooter;
    this.hood = hood;

    hood.setDefaultCommand(hood.tuckCommand(hood));
    leftShooter.setDefaultCommand(leftShooter.stopCommand());
    rightShooter.setDefaultCommand(rightShooter.stopCommand());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("DriveCommands/atAngleSetpoint", DriveCommands.atAngleSetpoint());
    Logger.recordOutput(
        "DriveCommands/atDriveToPoseSetpoint", DriveCommands.atDriveToPoseSetpoint());
  }

  public Command aimCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return new ParallelCommandGroup(
        DriveCommands.joystickDriveAtAngle(
            drive,
            driveXSupplier,
            driveYSupplier,
            () -> ShootingUtilNew.makeSetpoint(drive).driveAngleRads(),
            () -> ShootingUtilNew.makeSetpoint(drive).driveVelocityRadsPerSec()),
        hood.runTrackedPositionCommand(() -> ShootingUtilNew.makeSetpoint(drive).hoodAngleRads()),
        leftShooter.runTrackedVelocityCommand(
            () -> ShootingUtilNew.makeSetpoint(drive).shooterSpeedRadsPerSec()),
        rightShooter.runTrackedVelocityCommand(
            () -> ShootingUtilNew.makeSetpoint(drive).shooterSpeedRadsPerSec()));
  }

  public Command shootCommand() {
    return new ParallelCommandGroup(
        spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
        loader.runVoltageCommand(Presets.Loader.FEED_VOLTS));
  }

  public Command endShootCommand() {
    return new SequentialCommandGroup(
        // leftShooter.runTrackedVelocityCommand(Presets.Shooter.NO_SPEED),
        // rightShooter.runTrackedVelocityCommand(Presets.Shooter.NO_SPEED),
        loader.runVoltageCommand(Presets.Loader.SLOW_EXHAUST_VOLTS).withTimeout(0.5),
        new ParallelCommandGroup(spindexer.stopCommand(), loader.stopCommand()));
  }

  public Command deployIntake() {
    return intakeDeploy.runTrackedPositionCommand(
        () -> Presets.Intake.EXTEND_ANGLE_DEG.getAsDouble());
  }

  public Command retractIntake() {
    return intakeDeploy.runTrackedPositionCommand(
        () -> Presets.Intake.TUCK_ANGLE_DEG.getAsDouble());
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
        aimCommand(() -> 0.0, () -> 0.0),
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> leftShooter.atSetpoint() && rightShooter.atSetpoint()),
            new ParallelCommandGroup(
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS))));
  }

  public Command closeHubShoot() {
    return new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> leftShooter.atSetpoint() && rightShooter.atSetpoint()),
                new ParallelCommandGroup(
                    loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                    spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS))))
        .andThen(autoEndShootCommand());
  }

  public Command autoEndShootCommand() {
    return new ParallelCommandGroup(
        spindexer.stopCommand(),
        new SequentialCommandGroup(
            loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS).withTimeout(0.5),
            loader.stopCommand()),
        leftShooter.stopCommand(),
        rightShooter.stopCommand());
  }
}
