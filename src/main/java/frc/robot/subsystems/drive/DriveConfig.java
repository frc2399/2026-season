package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.SubsystemFactory;

public final class DriveConfig {
        public static final double kS;
        public static final double kV;
        public static final double DRIVE_P;
        public static final double DRIVE_D;
        public static final double TURN_P;
        public static final double TURN_D;
        public static final double HEADING_P;
        public static final double HEADING_D;
        public static final int PINION_TEETH;
        public static final Distance TRACK_WIDTH;
        public static final Distance TRACK_LENGTH;
        public static final LinearAcceleration MAX_ACCELERATION;
        public static final LinearVelocity MAX_SPEED;
        public static final AngularVelocity MAX_ANGULAR_VELOCITY;

        static {
                switch(SubsystemFactory.getRobotType()) {
                        case BUBBLES:
                                kS = 0.1;
                                kV = 2.29;
                                DRIVE_P = 0.4;
                                DRIVE_D = 0;
                                TURN_P = 1.0;
                                TURN_D = 0.001;
                                HEADING_P = 1.1;
                                HEADING_D = 0.05;
                                PINION_TEETH = 12;
                                TRACK_WIDTH = Inches.of(26 - (2 * 1.75));
                                TRACK_LENGTH = Inches.of(26 - (2 * 1.75));
                                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                                MAX_SPEED = MetersPerSecond.of(0);
                                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                                break;
                        case MOZART:
                                kS = 0.1;
                                kV = 2.29;
                                DRIVE_P = 0.4;
                                DRIVE_D = 0;
                                TURN_P = 1.0;
                                TURN_D = 0.001;
                                HEADING_P = 1.1;
                                HEADING_D = 0.05;
                                PINION_TEETH = 12;
                                TRACK_WIDTH = Inches.of(26 - (2 * 1.75));
                                TRACK_LENGTH = Inches.of(26 - (2 * 1.75));
                                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                                MAX_SPEED = MetersPerSecond.of(0);
                                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                                break;
                        case BETA:
                                kS = 0;
                                kV = 0;
                                DRIVE_P = 0;
                                DRIVE_D = 0;
                                TURN_P = 0;
                                TURN_D = 0;
                                HEADING_P = 0;
                                HEADING_D = 0;
                                PINION_TEETH = 0;
                                TRACK_WIDTH = Inches.of(0);
                                TRACK_LENGTH = Inches.of(0);
                                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                                MAX_SPEED = MetersPerSecond.of(0);
                                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                                break;
                        case COMP:
                                kS = 0;
                                kV = 0;
                                DRIVE_P = 0;
                                DRIVE_D = 0;
                                TURN_P = 0;
                                TURN_D = 0;
                                HEADING_P = 0;
                                HEADING_D = 0;
                                PINION_TEETH = 0;
                                TRACK_WIDTH = Inches.of(0);
                                TRACK_LENGTH = Inches.of(0);
                                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                                MAX_SPEED = MetersPerSecond.of(0);
                                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                                break;
                        case SIM:
                                kS = 0;
                                kV = 0;
                                DRIVE_P = 0;
                                DRIVE_D = 0;
                                TURN_P = 0;
                                TURN_D = 0;
                                HEADING_P = 0;
                                HEADING_D = 0;
                                PINION_TEETH = 0;
                                TRACK_WIDTH = Inches.of(0);
                                TRACK_LENGTH = Inches.of(0);
                                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                                MAX_SPEED = MetersPerSecond.of(0);
                                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                                break;
                        default:
                                kS = 0;
                                kV = 0;
                                DRIVE_P = 0;
                                DRIVE_D = 0;
                                TURN_P = 0;
                                TURN_D = 0;
                                HEADING_P = 0;
                                HEADING_D = 0;
                                PINION_TEETH = 0;
                                TRACK_WIDTH = Inches.of(0);
                                TRACK_LENGTH = Inches.of(0);
                                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                                MAX_SPEED = MetersPerSecond.of(0);
                                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                                break;
                }
        }
}