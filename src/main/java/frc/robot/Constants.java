// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class kOperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class kDrivetrain {
        public static final class kCANID {
            public static final int kMotDriveFL = 12;
            public static final int kMotDriveFR = 9;
            public static final int kMotDriveBL = 15;
            public static final int kMotDriveBR = 18;

            public static final int kMotTurnFL = 11;
            public static final int kMotTurnFR = 8;
            public static final int kMotTurnBL = 14;
            public static final int kMotTurnBR = 17;

            public static final int kCANcoderFL = 13;
            public static final int kCANcoderFR = 10;
            public static final int kCANcoderBL = 16;
            public static final int kCANcoderBR = 19;

            public static final int kPigeon = 3;
        }

        public static final class kCurrentLimit {
            public static final int kMotDriveCurrentLimit = 40;
            public static final int kMotTurnCurrentLimit = 30;
        }

        public static final class kPID {
            public static final double kDriveP = 0.04;
            public static final double kDriveI = 0;
            public static final double kDriveD = 0;
            public static final double kDriveFF = 0.22;

            public static final double kTurnP = 3;
            public static final double kTurnI = 0;
            public static final double kTurnD = 0;
            public static final double kTurnFF = 0;

            public static final double kHeadingP = 3;
            public static final double kHeadingI = 0;
            public static final double kHeadingD = 0;
        }

        public static final class kCANcoder {
            public static final int kCountsPerRev = 4096;

            public static final double kMountPoseYaw = -90; // -Y axis faces forward

            public static final double kAbsOffsetFL = 0.725098;
            public static final double kAbsOffsetFR = 0.692139;
            public static final double kAbsOffsetBL = 0.994873;
            public static final double kAbsOffsetBR = 0.875244;
        }

        public static enum Location {
            FRONT_LEFT,
            FRONT_RIGHT,
            BACK_LEFT,
            BACK_RIGHT
        }

        public static final class kRobot {
            public static final double kLength = 0.64;
            public static final double kWidth = 0.64;
        }

        public static final class kAutonomous {
            public static final String[] kPaths = {"Path1"}; // all available trajectories from PathPlanner
    
            public static final double kMaxDriveVelocity = 1;
            public static final double kMaxDriveAcceleration = 1;
        }

        public static final double kWheelRadius = 0.05; // meters
        public static final double kWheelDiameter = 2 * kWheelRadius;
        public static final double kWheelCircumference = Math.PI * kWheelDiameter;

        public static final double kDriveGearRatio = 6.75;
        public static final double kTurnGearRatio = 150.0 / 7.0;

        public static final double kMaxDriveVelocity = 4.56; // meters per second
        public static final double kMaxTurnAngularVelocity = 10; // rotation per second
        public static final double kMaxTurnAngularAcceleration = 2 * Math.toRadians(360); // rotation per second squared

        public static final int kCountsPerRev = 42; // NEO built-in relative encoder resolution
        public static final double kDriveEncoderCoefficient = kWheelCircumference / kDriveGearRatio; // meters
        public static final double kTurnEncoderCoefficient = 2 * Math.PI / kTurnGearRatio; // radians

        public static final double kDriveRampRate = 0.6;
    }
}
