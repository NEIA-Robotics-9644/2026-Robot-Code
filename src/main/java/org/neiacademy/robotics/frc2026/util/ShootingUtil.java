package org.neiacademy.robotics.frc2026.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.neiacademy.robotics.frc2026.Constants;
import org.neiacademy.robotics.frc2026.subsystems.drive.Drive;

public class ShootingUtil {

  private static final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopTime));

  private static double lastHubDriveAngleRads;
  private static double lastHubhoodPosition;

  private static double lastShuttleDriveAngleRads;
  private static double lastShuttlehoodPosition;

  private static double hubFudgeFactor = -17;
  private static double shuttleFudgeFactor = +25;

  private static double fixedShuttleFudgeFactor = 0;

  public record ShooterSetpoint(
      Rotation2d driveAngleRads,
      double hoodPosition,
      Rotation2d driveVelocityRadsPerSec,
      double shooterSpeedRadsPerSec) {}

  private static final InterpolatingDoubleTreeMap hubDistanceHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hubDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hubDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap shuttleDistanceHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shuttleDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shuttleDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap fixedHubDistanceHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap fixedHubDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap fixedHubDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap fixedShuttleDistanceHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap fixedShuttleDistanceShooterVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap fixedShuttleDistanceTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    hubDistanceHoodAngleMap.put(0.0, 0.05);
    hubDistanceHoodAngleMap.put(1.0, 0.2);
    hubDistanceHoodAngleMap.put(1.53, 0.3);
    hubDistanceHoodAngleMap.put(2.0, 0.4);
    hubDistanceHoodAngleMap.put(2.49, 0.4);
    hubDistanceHoodAngleMap.put(2.99, 0.5);
    hubDistanceHoodAngleMap.put(3.52, 0.6);
    hubDistanceHoodAngleMap.put(4.0, 0.7);
    hubDistanceHoodAngleMap.put(4.5, 0.9);
    hubDistanceHoodAngleMap.put(5.0, 0.9);
    hubDistanceHoodAngleMap.put(5.5, 1.0);

    hubDistanceShooterVelocityMap.put(0.00, 278.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(1.0, 283.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(1.53, 298.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(2.01, 313.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(2.49, 333.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(2.99, 353.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(3.48, 360.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(4.00, 365.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(4.50, 375.0 + hubFudgeFactor);
    hubDistanceShooterVelocityMap.put(5.00, 385.0 + hubFudgeFactor);

    hubDistanceTimeOfFlightMap.put(0.0, 1.10);
    hubDistanceTimeOfFlightMap.put(1.0, 1.10);
    hubDistanceTimeOfFlightMap.put(3.0, 1.15);
    hubDistanceTimeOfFlightMap.put(5.0, 1.20);

    shuttleDistanceHoodAngleMap.put(0.0, 0.8);
    shuttleDistanceHoodAngleMap.put(2.5, 1.0);
    shuttleDistanceHoodAngleMap.put(3.5, 1.0);
    shuttleDistanceHoodAngleMap.put(4.5, 1.0);

    shuttleDistanceShooterVelocityMap.put(0.0, 300.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(1.5, 300.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(2.5, 300.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(3.5, 300.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(4.5, 335.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(5.5, 385.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(6.5, 425.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(7.5, 465.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(8.5, 480.0 + shuttleFudgeFactor);
    shuttleDistanceShooterVelocityMap.put(9.5, 480.0 + shuttleFudgeFactor);

    shuttleDistanceTimeOfFlightMap.put(0.0, 0.75);
    shuttleDistanceTimeOfFlightMap.put(3.0, 0.75);
    shuttleDistanceTimeOfFlightMap.put(5.0, 1.00);
    shuttleDistanceTimeOfFlightMap.put(10.0, 1.5);

    fixedHubDistanceHoodAngleMap.put(0.0, 0.0);
    fixedHubDistanceHoodAngleMap.put(1.0, 0.0);

    fixedHubDistanceShooterVelocityMap.put(0.0, 250.0);
    fixedHubDistanceShooterVelocityMap.put(1.0, 260.0);
    fixedHubDistanceShooterVelocityMap.put(1.5, 275.0);
    fixedHubDistanceShooterVelocityMap.put(2.0, 390.0);
    fixedHubDistanceShooterVelocityMap.put(2.5, 303.0);
    fixedHubDistanceShooterVelocityMap.put(3.0, 321.0);
    fixedHubDistanceShooterVelocityMap.put(3.5, 338.0);
    fixedHubDistanceShooterVelocityMap.put(4.0, 350.0);
    fixedHubDistanceShooterVelocityMap.put(4.5, 375.0);
    fixedHubDistanceShooterVelocityMap.put(5.0, 403.0);

    fixedHubDistanceTimeOfFlightMap.put(0.0, 1.10);
    fixedHubDistanceTimeOfFlightMap.put(1.0, 1.10);
    fixedHubDistanceTimeOfFlightMap.put(3.0, 1.15);
    fixedHubDistanceTimeOfFlightMap.put(5.0, 1.20);

    fixedShuttleDistanceHoodAngleMap.put(0.0, 0.0);
    fixedShuttleDistanceHoodAngleMap.put(1.0, 0.0);

    fixedShuttleDistanceShooterVelocityMap.put(0.0, 300.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(1.5, 300.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(2.5, 300.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(3.5, 300.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(4.5, 335.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(5.5, 385.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(6.5, 425.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(7.5, 465.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(8.5, 480.0 + fixedShuttleFudgeFactor);
    fixedShuttleDistanceShooterVelocityMap.put(9.5, 480.0 + fixedShuttleFudgeFactor);

    fixedShuttleDistanceTimeOfFlightMap.put(0.0, 0.75);
    fixedShuttleDistanceTimeOfFlightMap.put(3.0, 0.75);
    fixedShuttleDistanceTimeOfFlightMap.put(5.0, 1.00);
    fixedShuttleDistanceTimeOfFlightMap.put(10.0, 1.5);
  }

  public static ShooterSetpoint makeHubSetpoint(
      Drive drive, Pose2d target, BooleanSupplier isFixed) {

    double driveAngleRads = Double.NaN;
    double hoodPosition = Double.NaN;
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
      timeOfFlight =
          isFixed.getAsBoolean()
              ? fixedHubDistanceTimeOfFlightMap.get(futurePosetoTargetDistance)
              : hubDistanceTimeOfFlightMap.get(futurePosetoTargetDistance);
      futurePose =
          new Pose2d(
              drive.getPose().getX() + fieldRelativeVelocity.vxMetersPerSecond * timeOfFlight,
              drive.getPose().getY() + fieldRelativeVelocity.vyMetersPerSecond * timeOfFlight,
              drive.getPose().getRotation());
      futurePosetoTargetDistance = target.getTranslation().getDistance(futurePose.getTranslation());
    }

    driveAngleRads =
        target.getTranslation().minus(futurePose.getTranslation()).getAngle().getRadians();
    hoodPosition =
        isFixed.getAsBoolean()
            ? fixedHubDistanceHoodAngleMap.get(futurePosetoTargetDistance)
            : hubDistanceHoodAngleMap.get(futurePosetoTargetDistance);
    shooterSpeedRadsPerSec =
        isFixed.getAsBoolean()
            ? fixedHubDistanceShooterVelocityMap.get(futurePosetoTargetDistance)
            : hubDistanceShooterVelocityMap.get(futurePosetoTargetDistance);

    if (Double.isNaN(lastHubDriveAngleRads)) lastHubDriveAngleRads = driveAngleRads;
    if (Double.isNaN(lastHubhoodPosition)) lastHubhoodPosition = hoodPosition;

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
    lastHubhoodPosition = hoodPosition;

    Logger.recordOutput("ShootingUtil/HubTargetPose", target);
    Logger.recordOutput("ShootingUtil/HubFutureRobotPose", futurePose);
    Logger.recordOutput("ShootingUtil/HubFuturePosetoTargetDistance", futurePosetoTargetDistance);

    return new ShooterSetpoint(
        Rotation2d.fromRadians(driveAngleRads),
        hoodPosition,
        Rotation2d.fromRadians(driveVelocityRadsPerSec),
        shooterSpeedRadsPerSec);
  }

  // doesn't use SOTM for shuttling
  public static ShooterSetpoint makeShuttleSetpoint(
      Drive drive, Pose2d target, BooleanSupplier isFixed) {
    double driveAngleRads = Double.NaN;
    double hoodPosition = Double.NaN;
    double shooterSpeedRadsPerSec;
    double driveVelocityRadsPerSec;

    double robotPosetoTargetDistance =
        target.getTranslation().getDistance(drive.getPose().getTranslation());

    driveAngleRads =
        target.getTranslation().minus(drive.getPose().getTranslation()).getAngle().getRadians();
    hoodPosition =
        isFixed.getAsBoolean()
            ? fixedShuttleDistanceHoodAngleMap.get(robotPosetoTargetDistance)
            : shuttleDistanceHoodAngleMap.get(robotPosetoTargetDistance);
    shooterSpeedRadsPerSec =
        isFixed.getAsBoolean()
            ? fixedShuttleDistanceShooterVelocityMap.get(robotPosetoTargetDistance)
            : shuttleDistanceShooterVelocityMap.get(robotPosetoTargetDistance);

    if (Double.isNaN(lastShuttleDriveAngleRads)) lastShuttleDriveAngleRads = driveAngleRads;
    if (Double.isNaN(lastShuttlehoodPosition)) lastShuttlehoodPosition = hoodPosition;

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
    lastShuttlehoodPosition = hoodPosition;

    Logger.recordOutput("ShootingUtil/ShuttleTargetPose", target);
    Logger.recordOutput("ShootingUtil/ShuttleRobotPosetoTargetDistance", robotPosetoTargetDistance);

    return new ShooterSetpoint(
        Rotation2d.fromRadians(driveAngleRads),
        hoodPosition,
        Rotation2d.fromRadians(driveVelocityRadsPerSec),
        shooterSpeedRadsPerSec);
  }
}
