// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aimbot;

import edu.wpi.first.math.util.Units;

/** Holds shared aimbot state that persists across command instances. */
public final class StationaryAimbotCommandData {
  private static double offset = 0; // fudge factor to adjust trajectory mid-match

  private StationaryAimbotCommandData() {}

  /** Adds six inches to the aimbot fudge factor. */
  public static void addSixInches() {
    offset += Units.inchesToMeters(6);
  }

  /** Subtracts six inches from the aimbot fudge factor. */
  public static void minusSixInches() {
    offset -= Units.inchesToMeters(6);
  }

  /** Resets the aimbot fudge factor to zero. */
  public static void resetOffset() {
    offset = 0;
  }

  /** Returns the current fudge factor in meters. */
  public static double getOffsetMeters() {
    return offset;
  }
}
