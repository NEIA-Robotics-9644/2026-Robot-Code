// Copyright (c) 2026

package org.neiacademy.robotics.frc2026.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import java.util.Set;
import java.util.TreeSet;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoMirroringUtil {
  private static final String AUTO_EXTENSION = ".auto";
  private static final Path PATHPLANNER_AUTOS_PATH = Path.of("pathplanner", "autos");

  private AutoMirroringUtil() {}

  public static void addMirroredAutos(LoggedDashboardChooser<Command> autoChooser) {
    Path autosDirectory = Filesystem.getDeployDirectory().toPath().resolve(PATHPLANNER_AUTOS_PATH);

    Set<String> autoNames = getAutoNames(autosDirectory);
    Set<String> normalizedAutoNames = new TreeSet<>();
    autoNames.stream().map(AutoMirroringUtil::normalizeAutoName).forEach(normalizedAutoNames::add);

    for (String autoName : autoNames) {
      String mirroredAutoName = getMirroredAutoName(autoName);
      if (mirroredAutoName == null
          || normalizedAutoNames.contains(normalizeAutoName(mirroredAutoName))) {
        continue;
      }

      autoChooser.addOption(mirroredAutoName, new PathPlannerAuto(autoName, true));
    }
  }

  private static Set<String> getAutoNames(Path autosDirectory) {
    Set<String> autoNames = new TreeSet<>();
    if (!Files.isDirectory(autosDirectory)) {
      DriverStation.reportWarning(
          "PathPlanner autos directory does not exist: " + autosDirectory, false);
      return autoNames;
    }

    try (var files = Files.list(autosDirectory)) {
      files
          .filter(Files::isRegularFile)
          .map(path -> path.getFileName().toString())
          .filter(AutoMirroringUtil::isAutoFile)
          .map(fileName -> fileName.substring(0, fileName.length() - AUTO_EXTENSION.length()))
          .forEach(autoNames::add);
    } catch (IOException e) {
      DriverStation.reportWarning(
          "Failed to read PathPlanner autos directory: " + autosDirectory, e.getStackTrace());
    }

    return autoNames;
  }

  private static boolean isAutoFile(String fileName) {
    return fileName.toLowerCase(Locale.ROOT).endsWith(AUTO_EXTENSION);
  }

  private static String normalizeAutoName(String autoName) {
    return autoName.toLowerCase(Locale.ROOT);
  }

  private static String getMirroredAutoName(String autoName) {
    if (startsWithIgnoreCase(autoName, "right")) {
      return "Left" + autoName.substring("right".length());
    }
    if (startsWithIgnoreCase(autoName, "left")) {
      return "Right" + autoName.substring("left".length());
    }
    return null;
  }

  private static boolean startsWithIgnoreCase(String value, String prefix) {
    return value.regionMatches(true, 0, prefix, 0, prefix.length());
  }
}
