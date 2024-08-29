// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER = 0;  
        public static final int OPERATOR_CONTROLLER = 1;  
    }

    public static class DrivetrainConstants {
        public static final int MOTOR_LEFT_ID = 2;
        public static final int MOTOR_RIGHT_ID = 4;
        public static final double GEAR_RATIO = 1/12.75;
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI;
        public static final double ENCODER_CONVERSION_FACTOR = GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.875);

        public static final double DRIVE_P = 0.9; // +x speed [-1, 1] = DRIVE_P * vision distance METERS
        public static final double ANGULAR_P = 0.025; // rotation rate from [-1, 1] = ANGULAR_P * vision rotation DEGREES
    }

    public static class PhotonVisionConstants {
        //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System.
        public static final Transform3d ROBOT_TO_CAMERA =
                new Transform3d(Units.inchesToMeters(18), 0, Units.inchesToMeters(25.5), new Rotation3d());
        public static final double CAMERA_HEIGHT = Units.inchesToMeters(25.5);
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);
        // We need the height of the april tag (target). I assume this is to prevent slight errors being compounded when solving for height
        public static final double TARGET_HEIGHT = Units.inchesToMeters(57.5);
    }
}