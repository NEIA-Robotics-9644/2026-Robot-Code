// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.neiacademy.robotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.neiacademy.robotics.frc2026.commands.DriveCommands;
import org.neiacademy.robotics.frc2026.generated.TunerConstants;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.GyroIOPigeon2;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIO;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOSim;
import org.neiacademy.robotics.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.intake.Intake;
import org.neiacademy.robotics.frc2026.subsystems.intake.IntakeIO;
import org.neiacademy.robotics.frc2026.subsystems.intake.IntakeIOSim;
import org.neiacademy.robotics.frc2026.subsystems.intake.IntakeIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.indexer.Indexer;
import org.neiacademy.robotics.frc2026.subsystems.indexer.IndexerIO;
import org.neiacademy.robotics.frc2026.subsystems.indexer.IndexerIOSim;
import org.neiacademy.robotics.frc2026.subsystems.indexer.IndexerIOTalonFX;
import org.neiacademy.robotics.frc2026.subsystems.misc.LED.LEDSubsystem;
import org.neiacademy.robotics.frc2026.subsystems.test.HallEffect.TestHallEffect;
import org.neiacademy.robotics.frc2026.subsystems.test.HallEffect.TestHallEffectIOReal;
import org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN.TestLaserCAN;
import org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN.TestLaserCANIO;
import org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN.TestLaserCANIOReal;
import org.neiacademy.robotics.frc2026.subsystems.test.LaserCAN.TestLaserCANIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  private final LEDSubsystem led;

  private final Drive drive;

  private final Intake intake;
  private final Indexer indexer;

  // private final Intake intake

  //   private final Vision vision;

  private final TestLaserCAN testlaserCAN;

  private final TestHallEffect testhalleffect;

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert noAutoAlert =
      new Alert("Please select an auto routine!!! 😳", AlertType.kError);

  private Command noAuto = Commands.none();

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
        testhalleffect = new TestHallEffect(new TestHallEffectIOReal());
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOTalonFX(0, false), new IntakeIOTalonFX(0, false));

        // set CAN later
        indexer = new Indexer(new IndexerIOTalonFX(0, false));
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0),
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1),
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera2Name, VisionConstants.robotToCamera2),
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera3Name, VisionConstants.robotToCamera3));
        testlaserCAN = new TestLaserCAN(new TestLaserCANIOReal());
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
        intake = new Intake(new IntakeIOSim(), new IntakeIOSim());

        indexer = new Indexer(new IndexerIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera3Name, VisionConstants.robotToCamera3,
        // drive::getPose));
        testlaserCAN = new TestLaserCAN(new TestLaserCANIOSim());
        testhalleffect = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        led = null;
        testhalleffect = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {}, new IntakeIO() {});

        indexer = new Indexer(new IndexerIO() {});

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIO() {},
        //         new VisionIO() {},
        //         new VisionIO() {},
        //         new VisionIO() {});
        testlaserCAN = new TestLaserCAN(new TestLaserCANIO() {});
        break;
    }

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverCon.getLeftY(),
            () -> -driverCon.getLeftX(),
            () -> -driverCon.getRightX()));

    // Lock to 0° when A button is held
    driverCon
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverCon.getLeftY(),
                () -> -driverCon.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverCon.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverCon
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
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
