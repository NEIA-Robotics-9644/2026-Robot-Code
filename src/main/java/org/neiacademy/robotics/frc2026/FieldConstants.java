// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.neiacademy.robotics.frc2026;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Constants.disableHAL
                    ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
                    : Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }

  // This should be replaced in the future. All in meters
  public static class KeyFieldPoints {

    public static final Translation2d FIELD_CENTER = new Translation2d(8.2565, 4.021);

    public static final Map<Alliance, Translation2d> ALLIANCE_HUB =
        Map.of(
            Alliance.Red, new Translation2d(11.9, 4.021),
            Alliance.Blue, new Translation2d(4.61, 4.021));

    public static final Map<Alliance, Double> ALLIANCE_ZONE_LINE_FROM_TRENCH =
        Map.of(
            Alliance.Red, 11.8984,
            Alliance.Blue, 4.6116);

    public static final Map<Alliance, Translation2d> ALLIANCE_TRENCH_TOP =
        Map.of(
            Alliance.Red, new Translation2d(11.8984, 0.626),
            Alliance.Blue, new Translation2d(4.6116, 0.626));

    public static final Map<Alliance, Translation2d> ALLIANCE_TRENCH_BOTTOM =
        Map.of(
            Alliance.Red, new Translation2d(11.8984, 7.416),
            Alliance.Blue, new Translation2d(4.6116, 7.416));

    public static final Map<Alliance, Translation2d> ALLIANCE_PASSING_POINT_TOP =
        Map.of(
            Alliance.Red, new Translation2d(10.08, 3.01),
            Alliance.Blue, new Translation2d(6.43, 3.01));

    public static final Map<Alliance, Translation2d> ALLIANCE_PASSING_POINT_BOTTOM =
        Map.of(
            Alliance.Red, new Translation2d(10.08, 5.00),
            Alliance.Blue, new Translation2d(6.43, 5.00));
  }
}
