package org.neiacademy.robotics.frc2026.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;

/** Publishes the driver widgets used by layouts/elastic-layout.json. */
public final class RobotDashboard {
  private final Drive drive;
  private final Field2d field = new Field2d();
  private final SwerveWidget swerve = new SwerveWidget();

  public RobotDashboard(Drive drive) {
    this.drive = drive;
    update();
    SmartDashboard.putData("Swerve Drive", swerve);
    SmartDashboard.putData("Field", field);
    // WPILib publishes /FMSInfo automatically; the layout uses that table directly.
  }

  /** Call after the command scheduler so widgets use the latest subsystem inputs. */
  public void update() {
    var pose = drive.getPose();
    field.setRobotPose(pose);
    swerve.states = drive.getModuleStates();
    swerve.robotAngle = pose.getRotation().getRadians();
    // Preserve the Driver Station's -1 sentinel when no match time is available.
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /** Read-only measured states, in Elastic's expected CCW-positive radians and meters/second. */
  private static final class SwerveWidget implements Sendable {
    private SwerveModuleState[] states;
    private double robotAngle;

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("SwerveDrive");
      String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
      for (int i = 0; i < moduleNames.length; i++) {
        final int index = i;
        builder.addDoubleProperty(
            moduleNames[i] + " Angle", () -> states[index].angle.getRadians(), null);
        builder.addDoubleProperty(
            moduleNames[i] + " Velocity", () -> states[index].speedMetersPerSecond, null);
      }
      builder.addDoubleProperty("Robot Angle", () -> robotAngle, null);
    }
  }
}
