// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public final class RobotConstants {

    public static class MotorIdConstants {
        public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 21;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 31;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 41;

        public static final int FRONT_LEFT_TURNING_CAN_ID = 12;
        public static final int REAR_LEFT_TURNING_CAN_ID = 22;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 32;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 42;

        public static final int INTAKE_ROLLER_ALPHA_CAN_ID = 6;
        public static final int SHOOTER_INDEXER_CAN_ID = 1;
        public static final int GYRO_CAN_ID = 3;
        public static final int SHOOTER_BOTTOM_CAN_ID = 3;
        public static final int SHOOTER_TOP_CAN_ID = 21;

        public static final int INTAKE_ROLLER_BETA_AND_COMP_CAN_ID = 7;
        public static final int INTAKE_ARM_BETA_CAN_ID = 8;
        public static final int SHOOTER_BOTTOM_BETA_CAN_ID = 9;
        public static final int SHOOTER_TOP_BETA_CAN_ID = 10;
        public static final int SHOOTER_INDEXER_BETA_CAN_ID = 13;
        public static final int SPINDEXER_CAN_ID = 14;
    }

    public static class SensorIdConstants {}

    public static class MotorConstants {
        public static final Current NEO550_CURRENT_LIMIT = Amps.of(20);
        public static final Current NEO_CURRENT_LIMIT = Amps.of(50);
        public static final Current VORTEX_CURRENT_LIMIT = Amps.of(60);
        public static final AngularVelocity NEO550_FREE_SPEED = RPM.of(11000);
        public static final AngularVelocity NEO_FREE_SPEED = RPM.of(5676);
        public static final AngularVelocity VORTEX_FREE_SPEED = RPM.of(6784);
    }

    public static class SpeedConstants {
        public static final double MAIN_LOOP_FREQUENCY_HZ = 50;
        public static final Time MAIN_LOOP_FREQUENCY =
                Milliseconds.of(1000 / MAIN_LOOP_FREQUENCY_HZ);
        public static final double LOGGING_FREQUENCY_HZ = 10;
        public static final int LOGGING_FREQUENCY_MS = (int) (1000 / LOGGING_FREQUENCY_HZ);
    }

    public static class DriveControlConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int TUNING_CONTROLLER_PORT = 1;
        public static final double DRIVE_DEADBAND = 0.1;
        public static final boolean FIELD_ORIENTED_DRIVE = true;
        public static final double LOW_BATTERY_VOLTAGE = 11.0;
    }

    public static class TransformConstants {
        public static final Distance ROBOT_TO_SHOOTER_X = Inches.of(10.425726);
        public static final Distance ROBOT_TO_SHOOTER_Y = Inches.of(8.5);
        // these values should be edited once we have our robot to shooter transform
        // DO NOT LET THESE CHANGES GO TO MAIN THEY ARE FOR TESTING
        public static final Transform2d ROBOT_TO_SHOOTER_TRANSFORM =
                new Transform2d(0, 0, new Rotation2d(0));
    }

    public static class GearRatios {
        public static final int BETA_SPINDEXER_GEAR_RATIO = 6; // 6 : 1 gear ratio
        public static final int COMP_SPINDEXER_GEAR_RATIO = 9; // 9 : 1 gear ratio
    }

    public static class DeadbandConstants {
        public static final Angle BETA_INTAKE_ARM_DEADBAND = Degrees.of(3.0);
        public static final Angle COMP_INTAKE_ARM_DEADBAND = Degrees.of(2.0);
    }
}
