// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.neiacademy.robotics.frc2026.util.drivers;

public class CANDeviceID {
  private final int deviceNumber;
  private final String bus;

  public CANDeviceID(int deviceNumber, String bus) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
  }

  // Use the default bus name "rio".
  public CANDeviceID(int deviceNumber) {
    this(deviceNumber, "rio");
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBus() {
    return bus;
  }

  @SuppressWarnings("NonOverridingEquals")
  public boolean equals(CANDeviceID other) {
    return other.deviceNumber == deviceNumber && other.bus.equals(bus);
  }

  @Override
  public String toString() {
    return "CanDeviceId(" + deviceNumber + ", " + bus + ")";
  }
}
