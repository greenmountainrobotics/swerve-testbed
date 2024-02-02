// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class SwerveConstants {
    public static final int FrontLeftDriveId = 12;
    public static final int FrontLeftTurnId = 10;
    public static final int FrontLeftEncoderId = 11;

    public static final int FrontRightDriveId = 3;
    public static final int FrontRightTurnId = 1;
    public static final int FrontRightEncoderId = 2;

    public static final int BackLeftDriveId = 9;
    public static final int BackLeftTurnId = 7;
    public static final int BackLeftEncoderId = 8;

    public static final int BackRightDriveId = 6;
    public static final int BackRightTurnId = 4;
    public static final int BackRightEncoderId = 5;
  }


  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
