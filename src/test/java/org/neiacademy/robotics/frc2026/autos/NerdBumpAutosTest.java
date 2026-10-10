package org.neiacademy.robotics.frc2026.autos;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import org.junit.jupiter.api.Test;
import org.neiacademy.robotics.frc2026.Presets;
import org.neiacademy.robotics.frc2026.commands.DriveCommands;
import org.neiacademy.robotics.frc2026.subsystems.Superstructure;
import org.neiacademy.robotics.frc2026.subsystems.drive.*;
import org.neiacademy.robotics.frc2026.subsystems.hood.*;
import org.neiacademy.robotics.frc2026.subsystems.intakedeploy.*;
import org.neiacademy.robotics.frc2026.subsystems.intakeroller.*;
import org.neiacademy.robotics.frc2026.subsystems.loader.*;
import org.neiacademy.robotics.frc2026.subsystems.shooter.*;
import org.neiacademy.robotics.frc2026.subsystems.spindexer.*;

class NerdBumpAutosTest {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private void tick() {
    SimHooks.stepTiming(0.02);
    DriverStationSim.notifyNewData();
    scheduler.run();
  }

  @Test
  void allTwelveVariantsRunBothAlliancesAndDepartAtSeventeenSeconds() {
    runAutos(18.0);
  }

  @Test
  void allVariantsAtProductionFortyFiveKilograms() {
    runAutos(45.0);
  }

  @Test
  void reportAllVariantsAtFiftyKilograms() {
    runAutos(50.0);
  }

  private static RobotConfig configAtMass(double mass) {
    try {
      var field = Drive.class.getDeclaredField("PP_CONFIG");
      field.setAccessible(true);
      var current = (RobotConfig) field.get(null);
      return new RobotConfig(mass, current.MOI, current.moduleConfig, current.moduleLocations);
    } catch (ReflectiveOperationException e) {
      throw new AssertionError(e);
    }
  }

  @Test
  void compareGeneratedSpeedsAtEighteenAndFiftyKilograms() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    try (var files = Files.list(Path.of("src/main/deploy/pathplanner/paths"))) {
      for (var file :
          files
              .filter(p -> p.getFileName().toString().startsWith("Right NERD "))
              .sorted()
              .toList()) {
        String name = file.getFileName().toString().replace(".path", "");
        for (double mass : new double[] {18, 50}) {
          PathPlannerPath.clearCache();
          var path = PathPlannerPath.fromPathFile(name);
          var trajectory = path.getIdealTrajectory(configAtMass(mass)).orElseThrow();
          double peak =
              trajectory.getStates().stream()
                  .mapToDouble(s -> s.linearVelocity)
                  .max()
                  .orElseThrow();
          double entry = Double.NaN;
          if (name.endsWith("Safe_Bump")) {
            entry =
                trajectory.getStates().stream()
                    .min(Comparator.comparingDouble(s -> Math.abs(s.pose.getX() - 5.7)))
                    .orElseThrow()
                    .linearVelocity;
          }
          System.out.printf(
              "MASS_SPEED,%s,%.0f,%.4f,%.4f,%.4f%n",
              name, mass, trajectory.getTotalTimeSeconds(), peak, entry);
          assertTrue(Double.isFinite(trajectory.getTotalTimeSeconds()));
        }
      }
    } finally {
      PathPlannerPath.clearCache();
    }
  }

  private void runAutos(double mass) {
    assertTrue(HAL.initialize(500, 0));
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    scheduler.unregisterAllSubsystems();
    SimHooks.pauseTiming();
    RobotController.setTimeSource(RobotController::getFPGATime);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    PathPlannerPath.clearCache();
    KinematicDrive drive = new KinematicDrive();
    AutoBuilder.configure(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
        configAtMass(mass),
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red,
        drive);
    double[] volts = new double[3];
    ReadyShooterIO left = new ReadyShooterIO();
    ReadyShooterIO right = new ReadyShooterIO();
    Spindexer spindexer =
        new Spindexer(
            new SpindexerIO() {
              public void runVoltage(double value) {
                volts[0] = value;
              }

              public void stop() {
                volts[0] = 0;
              }
            });
    IntakeRoller intake =
        new IntakeRoller(
            new IntakeRollerIO() {
              public void runVoltage(double value) {
                volts[1] = value;
              }

              public void stop() {
                volts[1] = 0;
              }
            });
    Loader loader =
        new Loader(
            new LoaderIO() {
              public void runVoltage(double value) {
                volts[2] = value;
              }

              public void stop() {
                volts[2] = 0;
              }
            });
    Superstructure mechanisms =
        new Superstructure(
            drive,
            spindexer,
            new IntakeDeploy(new IntakeDeployIO() {}),
            intake,
            loader,
            new Shooter(left, true),
            new Shooter(right, false),
            new Hood(new HoodIO() {}));
    // The same public mechanism commands and limits registered in RobotContainer.
    NamedCommands.registerCommand("autoShoot", mechanisms.autoShoot());
    NamedCommands.registerCommand("intakeDeploy", mechanisms.deployIntake().withTimeout(0.75));
    NamedCommands.registerCommand("intakeRetract", mechanisms.retractIntake().withTimeout(0.75));
    NamedCommands.registerCommand("autoEndShootCommand", mechanisms.autoEndShootCommand());
    NamedCommands.registerCommand(
        "intakeRoller", intake.runVoltageCommand(Presets.Intake.INTAKE_VOLTS).withTimeout(5));
    NamedCommands.registerCommand(
        "unjam",
        Commands.parallel(
                intake.runVoltageCommand(Presets.Intake.EXHAUST_VOLTS),
                loader.runVoltageCommand(Presets.Loader.EXHAUST_VOLTS),
                spindexer.runVoltageCommand(Presets.Spindexer.EXHAUST_VOLTS))
            .withTimeout(0.25));
    NerdBumpAutos.registerCommands(drive, mechanisms);
    try {
      for (boolean red : new boolean[] {false, true}) {
        DriverStationSim.setAllianceStationId(
            red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
        tick();
        for (String side : new String[] {"Right", "Left"}) {
          for (String type : new String[] {"Safe", "Steal", "Super Safe"}) {
            for (boolean greedy : new boolean[] {false, true}) {
              String name = "Right NERD " + type + " Bump" + (greedy ? " Greedy" : "");
              PathPlannerAuto auto = new PathPlannerAuto(name, side.equals("Left"));
              Pose2d expectedStart =
                  red ? FlippingUtil.flipFieldPose(auto.getStartingPose()) : auto.getStartingPose();
              List<String> paths = new ArrayList<>();
              List<Double> pathStarts = new ArrayList<>();
              Set<Integer> shotWindows = new HashSet<>();
              Set<String> findings = new HashSet<>();
              Set<Integer> unjamWindows = new HashSet<>();
              Set<Integer> fastCrossings = new HashSet<>();
              Set<Integer> movingDepartures = new HashSet<>();
              Rotation2d crossingHeading;
              try {
                var crossing = PathPlannerPath.fromPathFile("Right NERD Safe_Bump");
                if (side.equals("Left")) crossing = crossing.mirrorPath();
                var pose = crossing.getStartingHolonomicPose().orElseThrow();
                crossingHeading = (red ? FlippingUtil.flipFieldPose(pose) : pose).getRotation();
              } catch (Exception e) {
                throw new AssertionError(e);
              }
              double started = Timer.getTimestamp();
              scheduler.schedule(auto);
              assertEquals(expectedStart.getX(), drive.getPose().getX(), 0.01);
              assertEquals(expectedStart.getY(), drive.getPose().getY(), 0.01);
              String previous = "";
              for (int i = 0; i < 1010 && auto.isScheduled(); i++) {
                tick();
                String path = PathPlannerAuto.currentPathName;
                if (!path.isEmpty() && !path.equals(previous)) {
                  paths.add(path);
                  pathStarts.add(Timer.getTimestamp() - started);
                }
                if (previous.endsWith("Safe_Bump") && path.isEmpty()) {
                  // Check BEFORE unjam/autoShoot can perform any aiming correction.
                  var blueEnd =
                      new Pose2d(2.83425, 2.55305, Rotation2d.fromDegrees(39.60214190455108));
                  if (side.equals("Left"))
                    blueEnd =
                        new Pose2d(
                            blueEnd.getX(),
                            FlippingUtil.fieldSizeY - blueEnd.getY(),
                            blueEnd.getRotation().unaryMinus());
                  var expected = red ? FlippingUtil.flipFieldPose(blueEnd) : blueEnd;
                  if (mass != 50.0)
                    assertEquals(
                        0,
                        expected.getRotation().minus(drive.getRotation()).getRadians(),
                        Math.toRadians(7),
                        "Path finishes turn before shooting");
                }
                previous = path;
                if (path.endsWith("Second_T_NZ_B") || path.endsWith("End_T_NZ")) {
                  Pose2d bluePose =
                      red ? FlippingUtil.flipFieldPose(drive.getPose()) : drive.getPose();
                  double speed =
                      Math.hypot(drive.speeds.vxMetersPerSecond, drive.speeds.vyMetersPerSecond);
                  if (bluePose.getY() > 1.8 && bluePose.getX() < 3.5 && speed > 0.2) {
                    if (mass != 50.0)
                      assertTrue(
                          drive.speeds.vxMetersPerSecond > 0, "Leave intake-first, not backwards");
                    if (Math.abs(drive.speeds.omegaRadiansPerSecond) > 0.25)
                      movingDepartures.add(paths.size());
                  }
                }
                if (path.endsWith("Safe_Bump")) {
                  Pose2d bluePose =
                      red ? FlippingUtil.flipFieldPose(drive.getPose()) : drive.getPose();
                  double x = bluePose.getX();
                  // Includes the bumper footprint, from first contact through complete landing.
                  if (x < 5.8 && x > 3.5) {
                    assertEquals(
                        0,
                        crossingHeading.minus(drive.getRotation()).getRadians(),
                        Math.toRadians(5),
                        "Hold crossing heading");
                    assertTrue(
                        Math.abs(drive.speeds.omegaRadiansPerSecond) < 0.35,
                        "Do not spin on the bump");
                  }
                  if (x < 5.8 && x > 5.6) {
                    assertTrue(
                        Math.hypot(drive.speeds.vxMetersPerSecond, drive.speeds.vyMetersPerSecond)
                            >= 2.8,
                        "Enter ramp at speed");
                    fastCrossings.add(paths.size());
                  }
                }
                if (path.isEmpty()
                    && (paths.size() == 2 || paths.size() == 4)
                    && Math.abs(drive.speeds.omegaRadiansPerSecond) > 0.35) {
                  Pose2d bluePose =
                      red ? FlippingUtil.flipFieldPose(drive.getPose()) : drive.getPose();
                  assertTrue(bluePose.getX() < 3.05, "Turn only at the landing spot");
                  if (mass != 50.0) {
                    assertEquals(
                        0,
                        Math.hypot(drive.speeds.vxMetersPerSecond, drive.speeds.vyMetersPerSecond),
                        1e-9,
                        "Turn stationary");
                  } else if (Math.hypot(
                          drive.speeds.vxMetersPerSecond, drive.speeds.vyMetersPerSecond)
                      > 1e-9) {
                    findings.add("return interrupted before stopping");
                  }
                }
                if (volts[2] == Presets.Loader.EXHAUST_VOLTS.get()) unjamWindows.add(paths.size());
                if (volts[2] == Presets.Loader.FEED_VOLTS.get()) {
                  if (mass != 50.0)
                    assertTrue(DriveCommands.atAngleSetpoint(), "Never feed facing away from hub");
                  shotWindows.add(paths.size());
                }
              }
              assertFalse(auto.isScheduled(), name);
              System.out.println(
                  "NATIVE SIM "
                      + mass
                      + "kg "
                      + side
                      + " "
                      + (red ? "Red " : "Blue ")
                      + name
                      + " "
                      + paths
                      + " "
                      + pathStarts
                      + " fed="
                      + shotWindows
                      + " findings="
                      + findings);
              assertEquals(
                  5, paths.size(), name + " must follow two sweeps, two returns, final run");
              assertTrue(paths.get(4).endsWith("End_T_NZ"));
              assertEquals(17.0, pathStarts.get(4), 0.10, name);
              // 50 kg is an exploratory report: missed windows are printed in fed=[...].
              if (mass != 50.0)
                assertEquals(Set.of(2, 4), shotWindows, name + " must shoot in both windows");
              if (mass != 50.0)
                assertEquals(Set.of(2, 4), unjamWindows, name + " must unjam before each shot");
              assertEquals(Set.of(2, 4), fastCrossings, name + " fast bump entry on both returns");
              if (mass != 50.0)
                assertEquals(
                    Set.of(3, 5), movingDepartures, "Turn while moving on both departures");
              assertEquals(20.0, Timer.getTimestamp() - started, 0.10);
              assertEquals(0, drive.speeds.vxMetersPerSecond, 1e-9);
              assertEquals(0, drive.speeds.vyMetersPerSecond, 1e-9);
              assertArrayEquals(new double[3], volts, 1e-9);
              assertEquals(0, left.target, 1e-9);
              assertEquals(0, right.target, 1e-9);
            }
          }
        }
      }
      left.ready = false;
      right.ready = false;
      double started = Timer.getTimestamp();
      double finalStart = -1;
      Command stalled = new PathPlannerAuto("Right NERD Safe Bump");
      scheduler.schedule(stalled);
      for (int i = 0; i < 910; i++) {
        tick();
        assertNotEquals(
            Presets.Loader.FEED_VOLTS.get(),
            volts[2],
            "Must not feed stalled flywheels (unjamming is allowed)");
        if (finalStart < 0 && PathPlannerAuto.currentPathName.endsWith("End_T_NZ")) {
          finalStart = Timer.getTimestamp() - started;
        }
      }
      assertEquals(17.0, finalStart, 0.10);
      stalled.cancel();
      assertArrayEquals(new double[3], volts, 1e-9);
      assertEquals(0, drive.speeds.vxMetersPerSecond, 1e-9);
      assertEquals(0, left.target, 1e-9);
    } finally {
      scheduler.cancelAll();
      scheduler.unregisterAllSubsystems();
      scheduler.getDefaultButtonLoop().clear();
      DriverStationSim.setEnabled(false);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      PathPlannerPath.clearCache();
      SimHooks.resumeTiming();
    }
  }

  @Test
  void importsRotateChassisAndMirrorSideWithoutChangingTravelGeometry() {
    PathPlannerPath right;
    PathPlannerPath left;
    try {
      right = PathPlannerPath.fromPathFile("Right NERD T_NZSafe_B");
      left = right.mirrorPath();
    } catch (Exception e) {
      throw new AssertionError(e);
    }
    assertFalse(right.isChoreoPath());
    assertFalse(left.isChoreoPath());
    var r = right.getStartingHolonomicPose().orElseThrow();
    var l = left.getStartingHolonomicPose().orElseThrow();
    assertEquals(4.54837, r.getX(), 1e-4);
    assertEquals(0.57921, r.getY(), 1e-4);
    assertEquals(Math.PI / 4, r.getRotation().getRadians(), 1e-4);
    assertEquals(r.getX(), l.getX(), 1e-9);
    assertEquals(FlippingUtil.fieldSizeY - r.getY(), l.getY(), 1e-9);
    assertEquals(-r.getRotation().getRadians(), l.getRotation().getRadians(), 1e-9);
  }

  // Ideal velocity integration tests scheduler, actual follower, heading control and IO requests.
  // It does not model wheel slip, bump impacts, motor limits, or fuel motion.
  private static class KinematicDrive extends Drive {
    private Pose2d pose = Pose2d.kZero;
    ChassisSpeeds speeds = new ChassisSpeeds();

    KinematicDrive() {
      super(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    }

    @Override
    public void periodic() {
      pose =
          pose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * 0.02,
                  speeds.vyMetersPerSecond * 0.02,
                  speeds.omegaRadiansPerSecond * 0.02));
    }

    @Override
    public Pose2d getPose() {
      return pose;
    }

    @Override
    public void setPose(Pose2d value) {
      pose = value;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
      return speeds;
    }

    @Override
    public void runVelocity(ChassisSpeeds value) {
      speeds = value;
    }
  }

  private static class ReadyShooterIO implements ShooterIO {
    double target;
    boolean ready = true;

    public void runVelocity(double velocity) {
      target = velocity;
    }

    public void stop() {
      target = 0;
    }

    public void updateInputs(ShooterIOInputs inputs) {
      inputs.leaderVelocityRadsPerSec = ready ? target : 0;
    }
  }
}
