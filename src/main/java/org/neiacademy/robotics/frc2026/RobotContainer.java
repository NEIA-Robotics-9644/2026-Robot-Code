// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.neiacademy.robotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.neiacademy.robotics.frc2026.Constants.*;
import org.neiacademy.robotics.frc2026.commands.DriveCommands;
import org.neiacademy.robotics.frc2026.generated.TunerConstants;
import org.neiacademy.robotics.frc2026.subsystems.Superstructure;
import org.neiacademy.robotics.frc2026.subsystems.Superstructure.INTAKE_DEPLOY_SETPOINT;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIOPigeon2;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOSim;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeploy;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIO;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRoller;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIO;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.loader.Loader;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIO;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.misc.LED.LEDSubsystem;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIO;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.Spindexer;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIO;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.vision.Vision;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionConstants;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.neiacademy.robotics.frc2026.util.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Spindexer spindexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;
  private final Loader loader;
  private final Shooter leftShooter;
  private final Shooter rightShooter;
  private final Superstructure superstructure;

  private final LEDSubsystem led;

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert noAutoAlert = new Alert("Please select an auto routine!!!", AlertType.kError);

  private Command noAuto = Commands.none();

  private Trigger inAllianceZone;

  // Controller
  private final CommandXboxController driverCon = new CommandXboxController(0);
  private final CommandXboxController operatorCon = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        led = new LEDSubsystem(9);
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));

        spindexer = new Spindexer(new SpindexerIOTalonFX());
        intakeDeploy = new IntakeDeploy(new IntakeDeployIOTalonFX());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
        loader = new Loader(new LoaderIOTalonFX());
        leftShooter =
            new Shooter(
                new ShooterIOTalonFX(
                    true,
                    Constants.Shooter.LEFT_SHOOTER_LEADER_ID,
                    Constants.Shooter.LEFT_SHOOTER_FOLLOWER_ID,
                    true),
                true);
        rightShooter =
            new Shooter(
                new ShooterIOTalonFX(
                    false,
                    Constants.Shooter.RIGHT_SHOOTER_LEADER_ID,
                    Constants.Shooter.RIGHT_SHOOTER_FOLLOWER_ID,
                    false),
                false);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        led = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);

        break;

      default:
        // Replayed robot, disable IO implementations
        led = null;
        vision = null;

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);

        break;
    }

    superstructure =
        new Superstructure(
            drive, spindexer, intakeDeploy, intakeRoller, loader, leftShooter, rightShooter);

    inAllianceZone =
        new Trigger(
            () -> {
              Pose2d robotPose = AllianceFlipUtil.apply(drive.getPose());
              return (robotPose.getX() <= FieldConstants.LinesVertical.allianceZone + 0.40);
            });

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addDefaultOption("No Auto!", noAuto);

    NamedCommands.registerCommand("shoot", superstructure.shootCommand().withTimeout(5));
    NamedCommands.registerCommand("autoShoot", superstructure.autoShoot().withTimeout(5));
    NamedCommands.registerCommand(
        "runIntakeRoller",
        intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS).withTimeout(5));
    NamedCommands.registerCommand("intakeDeploy", superstructure.deployIntake());
    NamedCommands.registerCommand("intakeRetract", superstructure.retractIntake());
    NamedCommands.registerCommand(
        "runSpindexer", spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS).withTimeout(5));
    NamedCommands.registerCommand(
        "shuttleAimAndShoot",
        Commands.run(
            () ->
                inAllianceZone
                    .negate()
                    .whileTrue(
                        superstructure.shuttleAimCommand(
                            () -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
                    .and(leftShooter::atSetpoint)
                    .and(rightShooter::atSetpoint)
                    .and(DriveCommands::atAngleSetpoint)
                    .whileTrue(superstructure.shootCommand())
                    .onFalse(superstructure.endShootCommand()),
            leftShooter,
            rightShooter).withTimeout(4));
    NamedCommands.registerCommand(
        "hubAimAndShoot",
        Commands.run(
            () ->
                inAllianceZone
                    .whileTrue(
                        superstructure.hubAimCommand(
                            () -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
                    .and(leftShooter::atSetpoint)
                    .and(rightShooter::atSetpoint)
                    .and(DriveCommands::atAngleSetpoint)
                    .whileTrue(superstructure.shootCommand())
                    .onFalse(superstructure.endShootCommand())).withTimeout(4));

    NamedCommands.registerCommand(
        "spinShooterFlywheels",
        Commands.run(
            () -> {
              leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED);
              rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED);
            }).withTimeout(5));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData(
        "RunEverythingForTuning",
        new ParallelCommandGroup(
            loader.runVoltageCommand(Presets.Loader.TUNING_VOLTS),
            spindexer.runVoltageCommand(Presets.Spindexer.TUNING_VOLTS),
            intakeRoller.runVoltageCommand(Presets.Intake.TUNING_VOLTS),
            intakeDeploy.runTrackedPositionCommand(
                () -> Units.degreesToRadians(Presets.Intake.TUNING_ANGLE_DEG.getAsDouble())),
            leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
            rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED)));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Trigger isManualMode =
        new Trigger(
            () -> {
              return Constants.manualMode;
            });

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverCon.getLeftY(),
            () -> -driverCon.getLeftX(),
            () -> -driverCon.getRightX()));

    // Switch to X pattern when X button is pressed
    driverCon.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0 when povdown button is pressed
    driverCon
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    driverCon
        .rightTrigger()
        .and(inAllianceZone)
        .whileTrue(
            superstructure.hubAimCommand(() -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .and(DriveCommands::atAngleSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());
    driverCon
        .rightTrigger()
        .and(inAllianceZone.negate())
        .whileTrue(
            superstructure.shuttleAimCommand(
                () -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .and(DriveCommands::atAngleSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    driverCon
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)));

    //driverCon.leftTrigger().onTrue(superstructure.deployIntake());
    driverCon.leftTrigger().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));

    driverCon.leftTrigger().whileTrue(intakeDeploy.setVolts(() -> -7));

    //driverCon.leftBumper().onTrue(superstructure.retractIntake());

    driverCon.leftBumper().whileTrue(intakeDeploy.setVolts(() -> 7));

    driverCon.b().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS));
    driverCon.b().whileTrue(loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS));

    operatorCon
        .start()
        .onTrue(Commands.runOnce(() -> Constants.setManualMode(!Constants.manualMode)));
    operatorCon
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Presets.Shooter.CLOSE_HUB_SPEED.setDefault(
                      Presets.Shooter.CLOSE_HUB_SPEED.getAsDouble());
                  if (superstructure.getCurrentIntakeDeploySetpoint()
                      == INTAKE_DEPLOY_SETPOINT.DEPLOYED) {
                    Presets.Intake.EXTEND_ANGLE_DEG.setDefault(
                        Units.radiansToDegrees(intakeDeploy.getAngleRads()));
                  } else if (superstructure.getCurrentIntakeDeploySetpoint()
                      == INTAKE_DEPLOY_SETPOINT.RETRACTED) {
                    Presets.Intake.TUCK_ANGLE_DEG.setDefault(
                        Units.radiansToDegrees(intakeDeploy.getAngleRads()));
                  }
                }));

    operatorCon
        .povUp()
        .and(isManualMode)
        .onTrue(Commands.runOnce(() -> Presets.Shooter.CLOSE_HUB_SPEED.adjustSetValue(1)));
    operatorCon
        .povDown()
        .and(isManualMode)
        .onTrue(Commands.runOnce(() -> Presets.Shooter.CLOSE_HUB_SPEED.adjustSetValue(-1)));
    operatorCon
        .povLeft()
        .and(isManualMode)
        .onTrue(
            Commands.runOnce(
                () ->
                    intakeDeploy.runPositionCommand(
                        () -> intakeDeploy.getAngleRads() + Units.degreesToRadians(-1))));
    operatorCon
        .povRight()
        .and(isManualMode)
        .onTrue(
            Commands.runOnce(
                () ->
                    intakeDeploy.runPositionCommand(
                        () -> intakeDeploy.getAngleRads() + Units.degreesToRadians(1))));

    operatorCon
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)));

    operatorCon
        .rightTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)));

    driverCon.b().whileTrue(spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(driverCon.getHID().getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operatorCon.getHID().getPort()));

    // Auto alert
    noAutoAlert.set(
        DriverStation.isAutonomous() && !DriverStation.isEnabled() && autoChooser.get() == noAuto);
  }
}
