// Copyright (c) 2026

package org.neiacademy.robotics.frc2026.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.HashSet;
import java.util.Set;
import org.neiacademy.robotics.frc2026.util.HubShiftUtil.ShiftEnum;

public class ControllerAlertRumble {
  private static final double ALERT_RUMBLE_DURATION_SECS = 0.5;
  private static final double ALERT_RUMBLE_VALUE = 1.0;
  private static final double SHIFT_RUMBLE_VALUE = 1.0;
  private static final String SHIFT_RUMBLE_ENABLED_KEY = "ControllerRumble/ShiftRumbleEnabled";

  private final CommandXboxController[] controllers;
  private final StringArraySubscriber alertErrorsSubscriber =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getSubTable("Alerts")
          .getStringArrayTopic("errors")
          .subscribe(new String[] {});
  private final StringArraySubscriber alertWarningsSubscriber =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getSubTable("Alerts")
          .getStringArrayTopic("warnings")
          .subscribe(new String[] {});
  private final StringArraySubscriber alertInfosSubscriber =
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getSubTable("Alerts")
          .getStringArrayTopic("infos")
          .subscribe(new String[] {});

  private Set<String> previousActiveAlerts = new HashSet<>();
  private double alertRumbleEndTimestamp = 0.0;
  private boolean shiftTrackingInitialized = false;
  private boolean tenSecondCuePlayed = false;
  private boolean fiveSecondCuePlayed = false;
  private boolean wasTeleopEnabled = false;
  private ShiftEnum previousShift = ShiftEnum.DISABLED;
  private double shiftPatternStartTimestamp = -1.0;
  private ShiftPattern shiftPattern = ShiftPattern.NONE;

  public ControllerAlertRumble(CommandXboxController... controllers) {
    this.controllers = controllers;
    SmartDashboard.putBoolean(SHIFT_RUMBLE_ENABLED_KEY, true);
    SmartDashboard.putData(
        "ControllerRumble/ResetShiftRumbleTimer",
        Commands.runOnce(this::resetShiftTracking)
            .ignoringDisable(true)
            .withName("Reset Shift Rumble Timer"));
  }

  public void periodic() {
    updateFmsShiftTrackingInitialization();
    updateShiftPattern();

    RumbleValues alertRumble = getAlertRumble();
    RumbleValues shiftRumble = getShiftRumble();
    setRumble(
        Math.max(alertRumble.left(), shiftRumble.left()),
        Math.max(alertRumble.right(), shiftRumble.right()));
  }

  private RumbleValues getAlertRumble() {
    Set<String> activeAlerts = getActiveAlerts();
    if (activeAlerts.stream().anyMatch(alert -> !previousActiveAlerts.contains(alert))) {
      alertRumbleEndTimestamp =
          Math.max(alertRumbleEndTimestamp, Timer.getTimestamp() + ALERT_RUMBLE_DURATION_SECS);
    }
    previousActiveAlerts = activeAlerts;

    return Timer.getTimestamp() < alertRumbleEndTimestamp
        ? new RumbleValues(ALERT_RUMBLE_VALUE, ALERT_RUMBLE_VALUE)
        : RumbleValues.NONE;
  }

  private void updateFmsShiftTrackingInitialization() {
    boolean teleopEnabled = DriverStation.isTeleopEnabled();
    if (DriverStation.isFMSAttached() && teleopEnabled && !wasTeleopEnabled) {
      resetShiftTracking();
    }
    wasTeleopEnabled = teleopEnabled;
  }

  private void resetShiftTracking() {
    HubShiftUtil.initialize();
    shiftTrackingInitialized = true;
    tenSecondCuePlayed = false;
    fiveSecondCuePlayed = false;
    previousShift = HubShiftUtil.getOfficialShiftInfo().currentShift();
    shiftPattern = ShiftPattern.NONE;
    shiftPatternStartTimestamp = -1.0;
  }

  private void updateShiftPattern() {
    if (!isShiftRumbleEnabled() || !shiftTrackingInitialized || !DriverStation.isTeleopEnabled()) {
      shiftPattern = ShiftPattern.NONE;
      shiftPatternStartTimestamp = -1.0;
      return;
    }

    var shiftInfo = HubShiftUtil.getOfficialShiftInfo();
    if (shiftInfo.currentShift() != previousShift) {
      previousShift = shiftInfo.currentShift();
      tenSecondCuePlayed = false;
      fiveSecondCuePlayed = false;
      shiftPattern = ShiftPattern.NONE;
      shiftPatternStartTimestamp = -1.0;
    }

    double remainingTime = shiftInfo.remainingTime();
    if (!fiveSecondCuePlayed && remainingTime <= 5.0 && remainingTime > 0.0) {
      startShiftPattern(ShiftPattern.FIVE_SECONDS);
      fiveSecondCuePlayed = true;
      tenSecondCuePlayed = true;
    } else if (!tenSecondCuePlayed && remainingTime <= 10.0 && remainingTime > 5.0) {
      startShiftPattern(ShiftPattern.TEN_SECONDS);
      tenSecondCuePlayed = true;
    }
  }

  private boolean isShiftRumbleEnabled() {
    return SmartDashboard.getBoolean(SHIFT_RUMBLE_ENABLED_KEY, true);
  }

  private void startShiftPattern(ShiftPattern pattern) {
    shiftPattern = pattern;
    shiftPatternStartTimestamp = Timer.getTimestamp();
  }

  private RumbleValues getShiftRumble() {
    if (shiftPattern == ShiftPattern.NONE) {
      return RumbleValues.NONE;
    }

    double elapsedTime = Timer.getTimestamp() - shiftPatternStartTimestamp;
    RumbleValues values =
        switch (shiftPattern) {
          case TEN_SECONDS -> getTenSecondShiftRumble(elapsedTime);
          case FIVE_SECONDS -> getFiveSecondShiftRumble(elapsedTime);
          case NONE -> RumbleValues.NONE;
        };

    if (values == RumbleValues.NONE) {
      shiftPattern = ShiftPattern.NONE;
      shiftPatternStartTimestamp = -1.0;
    }
    return values;
  }

  private RumbleValues getTenSecondShiftRumble(double elapsedTime) {
    if (elapsedTime < 0.2) {
      return RumbleValues.LEFT;
    }
    if (elapsedTime < 0.4) {
      return RumbleValues.RIGHT;
    }
    return RumbleValues.NONE;
  }

  private RumbleValues getFiveSecondShiftRumble(double elapsedTime) {
    if (elapsedTime < 1.0) {
      return RumbleValues.LEFT;
    }
    if (elapsedTime < 2.0) {
      return RumbleValues.RIGHT;
    }
    if (elapsedTime < 3.0) {
      return RumbleValues.LEFT;
    }
    if (elapsedTime < 4.0) {
      return RumbleValues.RIGHT;
    }
    if (elapsedTime < 6.0) {
      return RumbleValues.BOTH;
    }
    return RumbleValues.NONE;
  }

  private Set<String> getActiveAlerts() {
    Set<String> activeAlerts = new HashSet<>();
    addAlerts(activeAlerts, "error", alertErrorsSubscriber.get());
    addAlerts(activeAlerts, "warning", alertWarningsSubscriber.get());
    addAlerts(activeAlerts, "info", alertInfosSubscriber.get());
    return activeAlerts;
  }

  private void addAlerts(Set<String> activeAlerts, String type, String[] alerts) {
    for (String alert : alerts) {
      activeAlerts.add(type + ": " + alert);
    }
  }

  private void setRumble(double leftValue, double rightValue) {
    for (CommandXboxController controller : controllers) {
      controller.getHID().setRumble(RumbleType.kLeftRumble, leftValue);
      controller.getHID().setRumble(RumbleType.kRightRumble, rightValue);
    }
  }

  private record RumbleValues(double left, double right) {
    private static final RumbleValues NONE = new RumbleValues(0.0, 0.0);
    private static final RumbleValues LEFT = new RumbleValues(SHIFT_RUMBLE_VALUE, 0.0);
    private static final RumbleValues RIGHT = new RumbleValues(0.0, SHIFT_RUMBLE_VALUE);
    private static final RumbleValues BOTH =
        new RumbleValues(SHIFT_RUMBLE_VALUE, SHIFT_RUMBLE_VALUE);
  }

  private enum ShiftPattern {
    NONE,
    TEN_SECONDS,
    FIVE_SECONDS
  }
}
