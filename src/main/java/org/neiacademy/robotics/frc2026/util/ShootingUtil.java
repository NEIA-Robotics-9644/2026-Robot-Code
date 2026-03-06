package org.neiacademy.robotics.frc2026.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;

public class ShootingUtil {

  private static final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopTime));

  private static double lastHubDriveAngleRads;

  private static double lastShuttleDriveAngleRads;

  public record ShooterSetpoint(
      Rotation2d driveAngleRads,
      Rotation2d driveVelocityRadsPerSec,
      double shooterSpeedRadsPerSec) {}

  private static final InterpolatingDoubleTreeMap hubDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hubDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap shuttleDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();

  static {
    hubDistanceShooterVelocityMap.put(0.0, 280.0);
    hubDistanceShooterVelocityMap.put(1.0, 280.0);
    hubDistanceShooterVelocityMap.put(1.5, 285.0);
    hubDistanceShooterVelocityMap.put(2.0, 290.0);
    hubDistanceShooterVelocityMap.put(2.5, 295.0);
    hubDistanceShooterVelocityMap.put(3.0, 305.0);
    hubDistanceShooterVelocityMap.put(3.5, 315.0);
    hubDistanceShooterVelocityMap.put(4.0, 325.0);
    hubDistanceShooterVelocityMap.put(4.5, 335.0);
    hubDistanceShooterVelocityMap.put(5.0, 345.0);

    hubDistanceTimeOfFlightMap.put(0.0, 1.0);
    hubDistanceTimeOfFlightMap.put(1.0, 1.0);
    hubDistanceTimeOfFlightMap.put(3.0, 1.05);
    hubDistanceTimeOfFlightMap.put(5.0, 1.05);

    shuttleDistanceShooterVelocityMap.put(0.0, 250.0);
    shuttleDistanceShooterVelocityMap.put(1.5, 250.0);
    shuttleDistanceShooterVelocityMap.put(2.5, 270.0);
    shuttleDistanceShooterVelocityMap.put(3.5, 290.0);
    shuttleDistanceShooterVelocityMap.put(4.5, 310.0);
    shuttleDistanceShooterVelocityMap.put(5.5, 350.0);
    shuttleDistanceShooterVelocityMap.put(6.5, 390.0);
    shuttleDistanceShooterVelocityMap.put(7.5, 430.0);
    shuttleDistanceShooterVelocityMap.put(8.5, 470.0);
    shuttleDistanceShooterVelocityMap.put(9.5, 510.0);
    shuttleDistanceShooterVelocityMap.put(10.5, 540.0);
  }

  public static ShooterSetpoint makeHubSetpoint(Drive drive, Pose2d target) {

    double driveAngleRads = Double.NaN;
    double shooterSpeedRadsPerSec;
    double driveVelocityRadsPerSec;

    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());

    double timeOfFlight;
    Pose2d futurePose = drive.getPose();
    double futurePosetoTargetDistance =
        target.getTranslation().getDistance(drive.getPose().getTranslation());

    // iterate over timeOfFlight for each new future pose because it would be slightly different
    for (int i = 0; i < 25; i++) {
      timeOfFlight = hubDistanceTimeOfFlightMap.get(futurePosetoTargetDistance);
      futurePose =
          new Pose2d(
              drive.getPose().getX() + fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight,
              drive.getPose().getY() + fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight,
              drive.getPose().getRotation());
      futurePosetoTargetDistance = target.getTranslation().getDistance(futurePose.getTranslation());
    }

    driveAngleRads =
        target.getTranslation().minus(futurePose.getTranslation()).getAngle().getRadians();
    shooterSpeedRadsPerSec = hubDistanceShooterVelocityMap.get(futurePosetoTargetDistance);

    if (Double.isNaN(lastHubDriveAngleRads)) lastHubDriveAngleRads = driveAngleRads;

    // drive angular speed wraparound
    double deltaAngleRads = driveAngleRads - lastHubDriveAngleRads;
    if (deltaAngleRads > Math.PI) deltaAngleRads -= (2 * Math.PI);
    else if (deltaAngleRads < -Math.PI) deltaAngleRads += (2 * Math.PI);

    // stops rapid switches in futurePose target velocity when near the goal
    driveVelocityRadsPerSec =
        (futurePosetoTargetDistance >= 1.0)
            ? driveAngleFilter.calculate(
                MathUtil.clamp(
                    (deltaAngleRads) / Constants.loopTime,
                    -drive.getMaxAngularSpeedRadPerSec(),
                    drive.getMaxAngularSpeedRadPerSec()))
            : 0;

    lastHubDriveAngleRads = driveAngleRads;

    Logger.recordOutput("ShootingUtil/HubTargetPose", target);
    Logger.recordOutput("ShootingUtil/HubFutureRobotPose", futurePose);
    Logger.recordOutput("ShootingUtil/HubFuturePosetoTargetDistance", futurePosetoTargetDistance);

    return new ShooterSetpoint(
        Rotation2d.fromRadians(driveAngleRads),
        Rotation2d.fromRadians(driveVelocityRadsPerSec),
        shooterSpeedRadsPerSec);
  }

  // doesn't use SOTM for shuttling
  public static ShooterSetpoint makeShuttleSetpoint(Drive drive, Pose2d target) {
    double driveAngleRads = Double.NaN;
    double hoodAngleRads = Double.NaN;
    double shooterSpeedRadsPerSec;
    double driveVelocityRadsPerSec;
    double hoodVelocityRadsPerSec;

    double robotPosetoTargetDistance =
        target.getTranslation().getDistance(drive.getPose().getTranslation());

    driveAngleRads =
        target.getTranslation().minus(drive.getPose().getTranslation()).getAngle().getRadians();
    shooterSpeedRadsPerSec = shuttleDistanceShooterVelocityMap.get(robotPosetoTargetDistance);

    if (Double.isNaN(lastShuttleDriveAngleRads)) lastShuttleDriveAngleRads = driveAngleRads;

    // drive angular speed wraparound
    double deltaAngleRads = driveAngleRads - lastShuttleDriveAngleRads;
    if (deltaAngleRads > Math.PI) deltaAngleRads -= (2 * Math.PI);
    else if (deltaAngleRads < -Math.PI) deltaAngleRads += (2 * Math.PI);

    // stops rapid switches in pose target velocity when near the goal
    driveVelocityRadsPerSec =
        (robotPosetoTargetDistance >= 1.0)
            ? driveAngleFilter.calculate(
                MathUtil.clamp(
                    (deltaAngleRads) / Constants.loopTime,
                    -drive.getMaxAngularSpeedRadPerSec(),
                    drive.getMaxAngularSpeedRadPerSec()))
            : 0;

    lastShuttleDriveAngleRads = driveAngleRads;

    Logger.recordOutput("ShootingUtil/ShuttleTargetPose", target);
    Logger.recordOutput("ShootingUtil/ShuttleRobotPosetoTargetDistance", robotPosetoTargetDistance);

    return new ShooterSetpoint(
        Rotation2d.fromRadians(driveAngleRads),
        Rotation2d.fromRadians(driveVelocityRadsPerSec),
        shooterSpeedRadsPerSec);
  }
}
