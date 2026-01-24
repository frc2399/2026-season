// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public final class Constants {

  public static class MotorIdConstants {
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
    public static final int REAR_LEFT_DRIVING_CAN_ID = 21;
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 31;
    public static final int REAR_RIGHT_DRIVING_CAN_ID = 41;

    public static final int FRONT_LEFT_TURNING_CAN_ID = 12;
    public static final int REAR_LEFT_TURNING_CAN_ID = 22;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 32;
    public static final int REAR_RIGHT_TURNING_CAN_ID = 42;

    public static final int GYRO_CAN_ID = 3;
  }

  public static class SensorIdConstants {
  }

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
    public static final int MAIN_LOOP_FREQUENCY_MS = (int) (1000 / MAIN_LOOP_FREQUENCY_HZ);
    public static final double LOGGING_FREQUENCY_HZ = 10;
    public static final int LOGGING_FREQUENCY_MS = (int) (1000 / LOGGING_FREQUENCY_HZ);
    public static final AngularVelocity ALGAE_INTAKE_SPEED = MotorConstants.NEO550_FREE_SPEED.times(1);
    public static final AngularVelocity ALGAE_OUTAKE_SPEED = MotorConstants.NEO550_FREE_SPEED.times(-0.50);
    public static final double DRIVETRAIN_MAX_SPEED_MPS = 4.4;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED_RPS = 2 * Math.PI;
  }

  public static class DriveControlConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVE_DEADBAND = 0.1;
    public static final boolean FIELD_ORIENTED_DRIVE = true;

    public static final double SLOW_DRIVE_FACTOR = 0.25;
    public static final double DRIVE_FACTOR = 1.0;

    public static final Distance ALPHA_TRACK_WIDTH = Meters.of(0.4954);
    public static final Distance MOZART_TRACK_WIDTH = Inches.of(26 - (2 * 1.75));
    // replace if needed

    // we got the 2 below numbers from blake at 6:57 bon 03/13
    public static final Distance BETA_XTRACK_WIDTH = Inches.of(24.5);
    public static final Distance BETA_YTRACK_WIDTH = Inches.of(26.5);

  }
}