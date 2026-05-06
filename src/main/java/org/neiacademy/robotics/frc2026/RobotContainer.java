// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.neiacademy.robotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.neiacademy.robotics.frc2026.Constants.*;
import org.neiacademy.robotics.frc2026.commands.DriveCommands;
import org.neiacademy.robotics.frc2026.generated.TunerConstants;
import org.neiacademy.robotics.frc2026.subsystems.Superstructure;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIOPigeon2;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOSim;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.hood.Hood;
import org.neiacademy.robotics.frc2026.subsystems.hood.HoodIO;
import org.neiacademy.robotics.frc2026.subsystems.hood.HoodIOLinearActuator;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeploy;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIO;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.IntakeDeployIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRoller;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIO;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.IntakeRollerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.loader.Loader;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIO;
import org.neiacademy.robotics.frc2026.subsystems.loader.LoaderIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.shooter.Shooter;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIO;
import org.neiacademy.robotics.frc2026.subsystems.shooter.ShooterIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.Spindexer;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIO;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.SpindexerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.vision.Vision;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionConstants;
import org.neiacademy.robotics.frc2026.subsystems.vision.VisionIO;
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
  private final Hood hood;
  private final Superstructure superstructure;

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert noAutoAlert = new Alert("Select an auto routine!!!", AlertType.kError);

  private Command noAuto = Commands.none();

  private Trigger inAllianceZone;

  private Trigger isManualMode;

  // Controller
  private final CommandXboxController driverCon = new CommandXboxController(0);
  private final CommandXboxController operatorCon = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  @AutoLogOutput boolean test = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));

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

        hood = new Hood(new HoodIOLinearActuator());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
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
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));

        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);
        hood = new Hood(new HoodIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        intakeDeploy = new IntakeDeploy(new IntakeDeployIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        loader = new Loader(new LoaderIO() {});
        leftShooter = new Shooter(new ShooterIO() {}, true);
        rightShooter = new Shooter(new ShooterIO() {}, false);
        hood = new Hood(new HoodIO() {});

        break;
    }

    superstructure =
        new Superstructure(
            drive, spindexer, intakeDeploy, intakeRoller, loader, leftShooter, rightShooter, hood);

    inAllianceZone =
        new Trigger(
            () -> {
              Pose2d robotPose = AllianceFlipUtil.apply(drive.getPose());
              return (robotPose.getX() <= FieldConstants.LinesVertical.allianceZone + 0.40);
            });

    isManualMode =
        new Trigger(
            () -> {
              return Constants.manualMode;
            });

    // manual shoot (close hub, center only auto)
    NamedCommands.registerCommand("shoot", superstructure.shootCommand().withTimeout(5));
    NamedCommands.registerCommand(
        "spinShooterFlywheels",
        Commands.parallel(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED))
            .withTimeout(5));

    NamedCommands.registerCommand("autoShoot", superstructure.autoShoot());
    NamedCommands.registerCommand(
        "intakeRoller", intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS).withTimeout(5));
    NamedCommands.registerCommand("intakeDeploy", superstructure.deployIntake().withTimeout(0.75));
    NamedCommands.registerCommand(
        "intakeRetract", superstructure.retractIntake().withTimeout(0.75));
    NamedCommands.registerCommand(
        "runSpindexer", spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS));
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
                        .onFalse(superstructure.endShootCommand()))
            .withTimeout(4));

    NamedCommands.registerCommand(
        "closeHubShoot",
        Commands.run(
            () -> {
              spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS);
              loader.runVoltageCommand(Presets.Loader.FEED_VOLTS);
              leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED);
              rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED);
            }));

    NamedCommands.registerCommand(
        "toggleIntakeDeploy", new RepeatCommand(superstructure.toggleIntake()).withTimeout(6));

    NamedCommands.registerCommand(
        "xLock", new ParallelCommandGroup(Commands.run(drive::stopWithX, drive)));

    NamedCommands.registerCommand(
        "forceShoot",
        new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS))
            .withTimeout(0.5));

    NamedCommands.registerCommand(
        "unjam",
        new ParallelCommandGroup(
                intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS))
            .withTimeout(0.25));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addDefaultOption("No Auto!", noAuto);

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
    autoChooser.addOption(
        "Left NZ Steal And Shoot Auto", new PathPlannerAuto("Right NZ Steal And Shoot Auto", true));
    autoChooser.addOption(
        "Left NZ Trench No Cross Wait Steal and Shoot Auto",
        new PathPlannerAuto("Right NZ Trench No Cross Wait Steal and Shoot Auto", true));
    autoChooser.addOption(
        "Left NZ Bump No Cross Wait Steal and Shoot Auto",
        new PathPlannerAuto("Right NZ Bump No Cross Wait Steal and Shoot Auto", true));

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
    hood.runTrackedPositionCommand(Presets.Hood.TUNING_POSITION);

    SmartDashboard.putBoolean("ManualMode", Constants.manualMode);
    SmartDashboard.putBoolean("TuningMode", Constants.tuningMode);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverCon.getLeftY(),
            () -> -driverCon.getLeftX(),
            () -> -driverCon.getRightX()));

    // Switch to X pattern when X button is pressed
    driverCon.x().whileTrue(Commands.run(drive::stopWithX, drive));

    // fall back
    driverCon
        .y()
        .whileTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    hood.positionCommand(Presets.Hood.CLOSE_HUB_POSITION.getAsDouble()),
                    leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                    rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)),
                new ParallelCommandGroup(
                    spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                    loader.runVoltageCommand(Presets.Loader.FEED_VOLTS),
                    leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                    rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED))))
        .onFalse(superstructure.endShootCommand());

    driverCon
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS)));

    // Lock to a parallel angle to shove balls
    driverCon
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverCon.getLeftY(),
                () -> -driverCon.getLeftX(),
                () -> DriveCommands.closestNormalAngle(drive.getPose()),
                () -> new Rotation2d(0, 0)));

    // auto shoot
    driverCon
        .rightTrigger()
        .debounce(0.1, DebounceType.kRising)
        .whileTrue(loader.runVoltageCommand(Presets.Loader.FEED_VOLTS));
    driverCon
        .rightTrigger()
        .and(inAllianceZone)
        .whileTrue(
            superstructure.hubAimCommand(() -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .and(DriveCommands::atAngleSetpoint)
        .and(hood::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());
    // x lock shoot
    driverCon
        .rightTrigger()
        .and(driverCon.x())
        .and(inAllianceZone)
        .whileTrue(
            superstructure.hubAimCommand(() -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .and(DriveCommands::atAngleSetpoint)
        .and(hood::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .whileTrue(Commands.run(drive::stopWithX, drive))
        .onFalse(superstructure.endShootCommand());
    // shuttle
    driverCon
        .rightTrigger()
        .and(inAllianceZone.negate())
        .whileTrue(
            superstructure.shuttleAimCommand(
                () -> -driverCon.getLeftY(), () -> -driverCon.getLeftX()))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        // .and(DriveCommands::atAngleSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // force shoot even if the tolerances aren't being met
    driverCon
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

    // force shoot fall back even if flywheels aren't fully spun up
    driverCon
        .povUp()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

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
        .start()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.HALF_SHUTTLE_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.HALF_SHUTTLE_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    driverCon
        .back()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.FULL_SHUTTLE_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.FULL_SHUTTLE_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // Tune shot
    driverCon
        .povLeft()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.TUNING_SPEED),
                hood.runTrackedPositionCommand(Presets.Hood.TUNING_POSITION),
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));

    driverCon.leftTrigger().onTrue(superstructure.deployIntake());
    driverCon.leftTrigger().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.INTAKE_VOLTS));

    driverCon.leftBumper().onTrue(superstructure.retractIntake());

    operatorCon
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS)));

    operatorCon.a().whileTrue(intakeRoller.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS));
    operatorCon.x().whileTrue(loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS));
    operatorCon.y().whileTrue(spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS));

    operatorCon
        .leftBumper()
        .and(operatorCon.leftStick())
        .whileTrue(
            intakeDeploy.runTrackedPositionCommand(
                () ->
                    clamp(
                        intakeDeploy.getAngleRads()
                            + (((Math.abs(Presets.Intake.TUCK_ANGLE_DEG.get())
                                        + Math.abs(Presets.Intake.EXTEND_ANGLE_DEG.get()))
                                    / (Presets.Intake.PIVOT_MANUAL_MOVEMENT_TOTAL_TIME.get() * 50))
                                * operatorCon.getLeftY()),
                        Presets.Intake.TUCK_ANGLE_DEG.get(),
                        Presets.Intake.EXTEND_ANGLE_DEG.get())));

    // fall back
    operatorCon
        .povUp()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.CLOSE_HUB_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    operatorCon
        .povRight()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.TRENCH_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.TRENCH_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    operatorCon
        .povLeft()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.CORNER_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.CORNER_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    operatorCon
        .povDown()
        .whileTrue(
            new ParallelCommandGroup(
                leftShooter.runTrackedVelocityCommand(Presets.Shooter.TOWER_SPEED),
                rightShooter.runTrackedVelocityCommand(Presets.Shooter.TOWER_SPEED)))
        .and(leftShooter::atSetpoint)
        .and(rightShooter::atSetpoint)
        .whileTrue(superstructure.shootCommand())
        .onFalse(superstructure.endShootCommand());

    // force shoot even if the tolerances aren't being met
    operatorCon
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                spindexer.runVoltageCommand(Presets.Spindexer.FEED_VOLTS),
                loader.runVoltageCommand(Presets.Loader.FEED_VOLTS)));
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
  // will stay until java gets updated for this
  private double clamp(double value, double min, double max) {
    return Math.max(min >= max ? min : max, Math.min(max >= min ? max : min, value));
  }
}
