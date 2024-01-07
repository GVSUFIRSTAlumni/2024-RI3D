// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int pigeonID = 0;

    public static final class DrivetrainConstants {
        public static final double maxSpeed = Units.feetToMeters(15.1);

        public static final int frontLeftDriveID = 0;
        public static final int frontLeftSteerID = 0;
        public static final int frontLeftCANCoderID = 0;

        public static final int frontRightDriveID = 0;
        public static final int frontRightSteerID = 0;
        public static final int frontRightCANCoderID = 0;

        public static final int backLeftDriveID = 0;
        public static final int backLeftSteerID = 0;
        public static final int backLeftCANCoderID = 0;

        public static final int backRightDriveID = 0;
        public static final int backRightSteerID = 0;
        public static final int backRightCANCoderID = 0;

        public static final double xOffsetMeters = 0d;
        public static final double yOffsetMeters = 0d;
    }
}
