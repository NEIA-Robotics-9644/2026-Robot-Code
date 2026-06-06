package org.neiacademy.robotics.frc2026.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
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
import org.neiacademy.robotics.frc2026.util.HubShiftUtil;
import org.neiacademy.robotics.frc2026.util.ShootingUtil;
import org.neiacademy.robotics.frc2026.util.ShootingUtil.ShooterSetpoint;

public class Superstructure extends SubsystemBase {
  private static final double TURN_ADJUSTMENT_DEADBAND = 0.15;
  private static final double TURN_ADJUSTMENT_RATE_RAD_PER_SEC = Units.degreesToRadians(3.0);

  private final Drive drive;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Hood hood;

  private ShooterSetpoint hubShootingSetpoint;
  private ShooterSetpoint shuttleShootingSetpoint;

  @AutoLogOutput(key = "Overrides/ShooterRadFudgeFactorShuttle")
  private double shooterRadFudgeFactorShuttle = 0;

  @AutoLogOutput(key = "Overrides/ShooterRadFudgeFactorShoot")
  private double shooterRadFudgeFactorShoot = -3;

  @AutoLogOutput(key = "Overrides/ShiftOverride")
  private boolean shiftOverride = false;

  @AutoLogOutput(key = "Overrides/AutoShootTurnAdjustmentDeg")
  private double autoShootTurnAdjustmentDeg = 0.0;

  private Rotation2d autoShootTurnAdjustment = Rotation2d.kZero;

  private Alert shiftOverrideAlert = new Alert("Shift Override", AlertType.kInfo);

  private final GenericEntry isFixedEntry =
      Shuffleboard.getTab("Shooter")
          .add("Fixed?", true)
          .withWidget(BuiltInWidgets.kToggleSwitch)
          .getEntry();
  BooleanSupplier isFixed = () -> isFixedEntry.getBoolean(false);

  public Superstructure(
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

    hubShootingSetpoint =
        ShootingUtil.makeHubSetpoint(
            drive,
            AllianceFlipUtil.apply(
                new Pose2d(
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.kZero)),
            isFixed,
            this::getShooterRadFudgeFactorShoot);
    shuttleShootingSetpoint =
        ShootingUtil.makeShuttleSetpoint(
            drive, getShuttleTargetPose(), isFixed, this::getShooterRadFudgeFactorShuttle);

    hood.setDefaultCommand(hood.tuckCommand(Presets.Hood.TUCK_POSITION));
    leftShooter.setDefaultCommand(leftShooter.stopCommand());
    rightShooter.setDefaultCommand(rightShooter.stopCommand());
    SmartDashboard.putData("Overrides/Shift", enableShiftOverride());
    SmartDashboard.putData("Overrides/ShooterFudgePlus1", fudgeShooterSpeedShoot(1));
    SmartDashboard.putData("Overrides/ShooterFudgeMinus1", fudgeShooterSpeedShoot(-1));
    SmartDashboard.putData("Overrides/ShuttleFudgePlus1", fudgeShooterSpeedShuttle(1));
    SmartDashboard.putData("Overrides/ShuttleFudgeMinus1", fudgeShooterSpeedShuttle(-1));
  }

  @Override
  public void periodic() {
    hubShootingSetpoint =
        ShootingUtil.makeHubSetpoint(
            drive,
            AllianceFlipUtil.apply(
                new Pose2d(
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.kZero)),
            isFixed,
            this::getShooterRadFudgeFactorShoot);
    shuttleShootingSetpoint =
        ShootingUtil.makeShuttleSetpoint(
            drive, getShuttleTargetPose(), isFixed, this::getShooterRadFudgeFactorShuttle);

    Logger.recordOutput("Shooter/isFixed", isFixed.getAsBoolean());

    Logger.recordOutput("DriveCommands/atAngleSetpoint", DriveCommands.atAngleSetpoint());
    Logger.recordOutput(
        "DriveCommands/atDriveToPoseSetpoint", DriveCommands.atDriveToPoseSetpoint());
    Logger.recordOutput(
        "Tuning/HubDistance",
        AllianceFlipUtil.apply(
                new Pose2d(FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.kZero))
            .getTranslation()
            .getDistance(drive.getPose().getTranslation()));

    Logger.recordOutput(
        "Superstructure/Shift Time", HubShiftUtil.getOfficialShiftInfo().remainingTime());
  }

  public Command enableShiftOverride() {
    return Commands.startEnd(
            () -> {
              shiftOverride = true;
              shiftOverrideAlert.set(true);
            },
            () -> {
              shiftOverride = false;
              shiftOverrideAlert.set(false);
            })
        .ignoringDisable(true)
        .withName("Override Shift");
  }

  public Command fudgeShooterSpeedShuttle(double fudgeFactor) {
    return Commands.runOnce(
        () -> shooterRadFudgeFactorShuttle = shooterRadFudgeFactorShuttle + fudgeFactor);
  }

  public Command fudgeShooterSpeedShoot(double fudgeFactor) {
    return Commands.runOnce(
        () -> shooterRadFudgeFactorShoot = shooterRadFudgeFactorShoot + fudgeFactor);
  }

  public Command hubAimCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return hubAimCommand(driveXSupplier, driveYSupplier, () -> 0.0);
  }

  public Command hubAimCommand(
      DoubleSupplier driveXSupplier,
      DoubleSupplier driveYSupplier,
      DoubleSupplier turnAdjustmentSupplier) {
    return new ParallelCommandGroup(
            Commands.run(() -> updateAutoShootTurnAdjustment(turnAdjustmentSupplier)),
            DriveCommands.joystickDriveAtAngle(
                drive,
                driveXSupplier,
                driveYSupplier,
                this::getAdjustedHubShootingSetpointDriveAngle,
                this::getHubShootingSetpointDriveVelocity),
            hood.runTrackedPositionCommand(this::getHubShootingSetpointHoodAngle),
            leftShooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed),
            rightShooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed))
        .beforeStarting(this::resetAutoShootTurnAdjustment)
        .finallyDo(() -> resetAutoShootTurnAdjustment());
  }

  public Command shuttleAimCommand(DoubleSupplier driveXSupplier, DoubleSupplier driveYSupplier) {
    return shuttleAimCommand(driveXSupplier, driveYSupplier, () -> 0.0);
  }

  public Command shuttleAimCommand(
      DoubleSupplier driveXSupplier,
      DoubleSupplier driveYSupplier,
      DoubleSupplier turnAdjustmentSupplier) {
    return new ParallelCommandGroup(
            Commands.run(() -> updateAutoShootTurnAdjustment(turnAdjustmentSupplier)),
            DriveCommands.joystickDriveAtAngle(
                drive,
                driveXSupplier,
                driveYSupplier,
                this::getAdjustedShuttleShootingSetpointDriveAngle,
                this::getShuttleShootingSetpointDriveVelocity),
            hood.runTrackedPositionCommand(this::getShuttleShootingSetpointHoodAngle),
            leftShooter.runTrackedVelocityCommand(this::getShuttleShootingSetpointShooterSpeed),
            rightShooter.runTrackedVelocityCommand(this::getShuttleShootingSetpointShooterSpeed))
        .beforeStarting(this::resetAutoShootTurnAdjustment)
        .finallyDo(() -> resetAutoShootTurnAdjustment());
  }

  public Command hubSpinFlywheelsCommand() {
    return new ParallelCommandGroup(
        hood.runTrackedPositionCommand(this::getHubShootingSetpointHoodAngle),
        leftShooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed),
        rightShooter.runTrackedVelocityCommand(this::getHubShootingSetpointShooterSpeed));
  }

  public Command shuttleSpinFlywheelsCommand() {
    return new ParallelCommandGroup(
        hood.runTrackedPositionCommand(this::getShuttleShootingSetpointHoodAngle),
        leftShooter.runTrackedVelocityCommand(this::getShuttleShootingSetpointShooterSpeed),
        rightShooter.runTrackedVelocityCommand(this::getShuttleShootingSetpointShooterSpeed));
  }

  public Command shootCommand() {
    return new SequentialCommandGroup(
        Commands.waitSeconds(0.1),
        new ParallelCommandGroup(
            spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
            loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));
  }

  public Command endShootCommand() {
    return new SequentialCommandGroup(
        // leftShooter.runTrackedVelocityCommand(Presets.Shooter.NO_SPEED),
        // rightShooter.runTrackedVelocityCommand(Presets.Shooter.NO_SPEED),
        // loader.runVoltageCommand(Presets.Loader.SLOW_EXHAUST_VOLTS).withTimeout(0.5),
        new ParallelCommandGroup(
            spindexer.stopCommand()
            // , loader.stopCommand()
            ));
  }

  public Command deployIntake() {
    return intakeDeploy.runTrackedPositionCommand(
        () -> Presets.Intake.EXTEND_ANGLE_DEG.getAsDouble());
  }

  public Command retractIntake() {
    return intakeDeploy.runTrackedPositionCommand(
        () -> Presets.Intake.TUCK_ANGLE_DEG.getAsDouble());
  }

  public Command toggleIntake() {
    return new SequentialCommandGroup(
            retractIntake().withTimeout(0.25), deployIntake().withTimeout(0.25))
        .repeatedly()
        .withTimeout(2.25)
        .andThen(retractIntake());
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
                    spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS))))
        .withTimeout(6)
        .andThen(autoEndShootCommand());
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
            loader.stopCommand(),
            leftShooter.stopCommand(),
            rightShooter.stopCommand())
        .withTimeout(0.1);
  }

  public Rotation2d getHubShootingSetpointDriveAngle() {
    return hubShootingSetpoint.driveAngleRads();
  }

  public Rotation2d getAdjustedHubShootingSetpointDriveAngle() {
    return hubShootingSetpoint.driveAngleRads().plus(autoShootTurnAdjustment);
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

  public Rotation2d getAdjustedShuttleShootingSetpointDriveAngle() {
    return shuttleShootingSetpoint.driveAngleRads().plus(autoShootTurnAdjustment);
  }

  public Rotation2d getShuttleShootingSetpointDriveVelocity() {
    return shuttleShootingSetpoint.driveVelocityRadsPerSec();
  }

  public double getShuttleShootingSetpointShooterSpeed() {
    return shuttleShootingSetpoint.shooterSpeedRadsPerSec();
  }

  public double getHubShootingSetpointHoodAngle() {
    return hubShootingSetpoint.hoodPosition();
  }

  public double getShuttleShootingSetpointHoodAngle() {
    return shuttleShootingSetpoint.hoodPosition();
  }

  private void updateAutoShootTurnAdjustment(DoubleSupplier turnAdjustmentSupplier) {
    double turnInput =
        MathUtil.applyDeadband(turnAdjustmentSupplier.getAsDouble(), TURN_ADJUSTMENT_DEADBAND);
    autoShootTurnAdjustment =
        autoShootTurnAdjustment.plus(
            Rotation2d.fromRadians(
                turnInput * TURN_ADJUSTMENT_RATE_RAD_PER_SEC * Constants.loopTime));
    autoShootTurnAdjustmentDeg = autoShootTurnAdjustment.getDegrees();
  }

  private void resetAutoShootTurnAdjustment() {
    autoShootTurnAdjustment = Rotation2d.kZero;
    autoShootTurnAdjustmentDeg = 0.0;
  }

  private double getShooterRadFudgeFactorShoot() {
    return shooterRadFudgeFactorShoot;
  }

  private double getShooterRadFudgeFactorShuttle() {
    return shooterRadFudgeFactorShuttle;
  }
}
