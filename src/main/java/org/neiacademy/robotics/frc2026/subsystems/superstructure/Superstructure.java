package org.neiacademy.robotics.frc2026.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.neiacademy.robotics.frc2026.FieldConstants.KeyFieldPoints;
import org.neiacademy.robotics.frc2026.GlobalRobotState;

public class Superstructure extends SubsystemBase {

  @Getter
  private final SuperstructurePhaseShiftTracker phaseShiftTracker =
      new SuperstructurePhaseShiftTracker();

  @Getter private TrackingTarget trackingTarget = TrackingTarget.BLUE_HUB;

  @Getter private Translation2d trackingTargetCoordinates;

  private boolean useTracker;

  public Superstructure() {}

  @Override
  public void periodic() {
    phaseShiftTracker.periodic();

    if (useTracker) {
      trackKeyPoints();
    }
  }

  private void trackKeyPoints() {
    if (GlobalRobotState.getInstance().isMatch()) {
      var alliance = GlobalRobotState.getInstance().getAlliance();
      var pose = GlobalRobotState.getInstance().getEstimatedPose();

      Translation2d hub = KeyFieldPoints.ALLIANCE_HUB.get(alliance);
      double zoneLineX = KeyFieldPoints.ALLIANCE_ZONE_LINE_FROM_TRENCH.get(alliance);
      Translation2d passingTop = KeyFieldPoints.ALLIANCE_PASSING_POINT_TOP.get(alliance);
      Translation2d passingBottom = KeyFieldPoints.ALLIANCE_PASSING_POINT_BOTTOM.get(alliance);

      if ((alliance == Alliance.Red && pose.getX() >= zoneLineX)
          || (alliance == Alliance.Blue && pose.getX() <= zoneLineX)) {

        if (phaseShiftTracker.getCurrentActiveAlliance() == alliance) {
          trackingTarget =
              (alliance == Alliance.Red) ? TrackingTarget.RED_HUB : TrackingTarget.BLUE_HUB;
          trackingTargetCoordinates = hub;
        } else {
          trackingTarget = TrackingTarget.NONE;
        }
      } else {
        boolean topHalf = pose.getY() <= KeyFieldPoints.FIELD_CENTER.getY();
        if (alliance == Alliance.Red) {
          trackingTarget =
              topHalf
                  ? TrackingTarget.RED_ALLIANCE_ZONE_TOP
                  : TrackingTarget.RED_ALLIANCE_ZONE_BOTTOM;
          trackingTargetCoordinates = topHalf ? passingTop : passingBottom;
        } else {
          trackingTarget =
              topHalf
                  ? TrackingTarget.BLUE_ALLIANCE_ZONE_TOP
                  : TrackingTarget.BLUE_ALLIANCE_ZONE_BOTTOM;
          trackingTargetCoordinates = topHalf ? passingTop : passingBottom;
        }
      }
    }
  }
  /*
     Allows disabling or enabling tracking key positions on
     the field using the Phase Shift Tracker
  */
  public void doKeyPointTracking() {
    useTracker = !useTracker;
  }

  public enum TrackingTarget {
    RED_ALLIANCE_ZONE_TOP,
    RED_ALLIANCE_ZONE_BOTTOM,
    RED_HUB,
    BLUE_ALLIANCE_ZONE_TOP,
    BLUE_ALLIANCE_ZONE_BOTTOM,
    BLUE_HUB,
    NONE
  }
}
