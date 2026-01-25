package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveSubsystemConfigurations {

    // constants for different robots, separated into classes for organization
    // not named following [ROBOTNAME]_KS template because every call will be
    // [ROBOTNAME]Constants.KS
    // since the constant will never be called inside its own class
    // thus making putting the robot name before every constant redundant
    private static final class MozartConstants {
        private static final double KS = 0;
        private static final double KV = 0;
        private static final double DRIVE_P = 0;
        private static final double DRIVE_D = 0;
        private static final double TURN_P = 0;
        private static final double TURN_D = 0;
        private static final double HEADING_P = 0;
        private static final double HEADING_D = 0;
        private static final Distance TRACK_WIDTH = Inches.of(0);
        private static final Distance TRACK_LENGTH = Inches.of(0);
        private static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
        private static final LinearVelocity MAX_SPEED = MetersPerSecond.of(0);
        private static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(0);
    }

    private static final class BubblesConstants {
        private static final double KS = 0;
        private static final double KV = 0;
        private static final double DRIVE_P = 0;
        private static final double DRIVE_D = 0;
        private static final double TURN_P = 0;
        private static final double TURN_D = 0;
        private static final double HEADING_P = 0;
        private static final double HEADING_D = 0;
        private static final Distance TRACK_WIDTH = Inches.of(0);
        private static final Distance TRACK_LENGTH = Inches.of(0);
        private static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
        private static final LinearVelocity MAX_SPEED = MetersPerSecond.of(0);
        private static final AngularAcceleration MAX_ANGULAR_ACCELERATION = RadiansPerSecondPerSecond.of(0);
    }

    // architecture for creating a DriveConfig
    /*
     * A record is a shorthand way of making a class that's only really meant to
     * store data.
     * The Java compiler will auto-generate a constructor and instance variables
     * that match the parameters.
     * As a result, the code is shorter, cleaner, and easier to read!
     * 
     * Any field in the class can be called via
     * [driveconfigobjectname].[fieldname]();
     * (e.g. mozartDriveConfig.kS();)
     */
    public record DriveConfig(
            double kS,
            double kV,
            double driveP,
            double driveD,
            double turnP,
            double turnD,
            double headingP,
            double headingD,
            Distance trackWidth,
            Distance trackLength,
            LinearAcceleration maxAccel,
            LinearVelocity maxlLinearSpeed,
            AngularAcceleration maxAngularAccel) {
    }

    public static final DriveConfig mozartDriveConfig = new DriveConfig(MozartConstants.KS, MozartConstants.KV,
            MozartConstants.DRIVE_P, MozartConstants.DRIVE_D, MozartConstants.TURN_P, MozartConstants.TURN_D,
            MozartConstants.HEADING_P, MozartConstants.HEADING_D, MozartConstants.TRACK_WIDTH,
            MozartConstants.TRACK_LENGTH, MozartConstants.MAX_ACCELERATION, MozartConstants.MAX_SPEED,
            MozartConstants.MAX_ANGULAR_ACCELERATION);

    public static final DriveConfig bubblesDriveConfig = new DriveConfig(BubblesConstants.KS, BubblesConstants.KV,
            BubblesConstants.DRIVE_P, BubblesConstants.DRIVE_D, BubblesConstants.TURN_P, BubblesConstants.TURN_D,
            BubblesConstants.HEADING_P, BubblesConstants.HEADING_D, BubblesConstants.TRACK_WIDTH,
            BubblesConstants.TRACK_LENGTH, BubblesConstants.MAX_ACCELERATION, BubblesConstants.MAX_SPEED,
            BubblesConstants.MAX_ANGULAR_ACCELERATION);
}
