// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

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
    public static final int kDriveController = 1;
    public static final double kDeadzone = 0.05;

    public static final class DrivetrainConstants {
        // public static final double maxSpeed = Units.feetToMeters(15.1);
        public static final double maxSpeed = Units.feetToMeters(30);
        public static final double maxTurningSpeed = 4.5;

        public static final int frontLeftDriveID = 4;
        public static final int frontLeftSteerID = 3;
        public static final int frontLeftCANCoderID = 20;
        public static final double frontLeftEncoderOffset = 96.328;

        public static final int frontRightDriveID = 7;
        public static final int frontRightSteerID = 1;
        public static final int frontRightCANCoderID = 21;
        public static final double frontRightEncoderOffset = 64.424;

        public static final int backLeftDriveID = 5;
        public static final int backLeftSteerID = 6;
        public static final int backLeftCANCoderID = 22;
        public static final double backLeftEncoderOffset = 11.338;

        public static final int backRightDriveID = 2;
        public static final int backRightSteerID = 8;
        public static final int backRightCANCoderID = 23;
        public static final double backRightEncoderOffset = 288.896;

        public static final double xOffsetMeters = Units.inchesToMeters(12.5);
        public static final double yOffsetMeters = Units.inchesToMeters(12.5);

        public static final class DriveParams {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 1;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }

        public static final class SteerParams {
            public static final double kP = 0.01;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kFF = 0;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
        }
    }
}
