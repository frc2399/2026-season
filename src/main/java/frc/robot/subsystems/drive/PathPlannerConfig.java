package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.SubsystemFactory;

public final class PathPlannerConfig {
    public static final Mass ROBOT_MASS;
    public static final MomentOfInertia ROBOT_MOI;
    public static final double WHEEL_RADIUS; //meters
    public static final double MAX_SPEED; //meters per second
    public static final double WHEEL_COF;
    public static final DCMotor DRIVE_MOTOR;
    public static final double DRIVE_CURRENT_LIMIT; //amps
    public static final int NUM_MOTORS;
    public static final Translation2d MODULE_OFFSETS;


    static {
        switch (SubsystemFactory.getRobotType()) {
            case BUBBLES:
                ROBOT_MASS = 
                ROBOT_MOI = KilogramSquareMeters.of(6.883);
                WHEEL_RADIUS = 
                MAX_SPEED = 4.3;
                WHEEL_COF = 
                DRIVE_MOTOR = 
                DRIVE_CURRENT_LIMIT = 
                NUM_MOTORS = 
                MODULE_OFFSETS = 
                break;
            case MOZART:
                ROBOT_MASS = 
                ROBOT_MOI = 
                WHEEL_RADIUS = 
                MAX_SPEED = 0;
                WHEEL_COF = 
                DRIVE_MOTOR = 
                DRIVE_CURRENT_LIMIT = 
                NUM_MOTORS = 
                MODULE_OFFSETS = 
                break;
            case BETA:
                ROBOT_MASS = 
                ROBOT_MOI = 
                WHEEL_RADIUS = 
                MAX_SPEED = 0;
                WHEEL_COF = 
                DRIVE_MOTOR = 
                DRIVE_CURRENT_LIMIT = 
                NUM_MOTORS = 
                MODULE_OFFSETS = 
                break;
            case COMP:
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(0);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 0;
                WHEEL_COF = 0;
                DRIVE_MOTOR = 0;
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = 0;
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
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(0);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 0;
                WHEEL_COF = 0;
                DRIVE_MOTOR = 0; 
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = 0;
                break;
        }
    }
}
