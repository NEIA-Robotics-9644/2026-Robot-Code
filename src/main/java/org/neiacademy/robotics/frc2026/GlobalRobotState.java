package org.neiacademy.robotics.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class GlobalRobotState {

  private static GlobalRobotState instance;

  @Getter @Setter @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @Getter @Setter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  @Getter @Setter private boolean isMatch = false;

  @Getter @Setter private Alliance alliance;

  private GlobalRobotState() {}

  public static GlobalRobotState getInstance() {
    if (instance == null) instance = new GlobalRobotState();
    return instance;
  }
}
