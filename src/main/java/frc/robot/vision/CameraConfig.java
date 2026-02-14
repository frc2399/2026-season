package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.SubsystemFactory;

public class CameraConfig {
    public static final String LIMELIGHT_NAME;
    public static final Angle CAMERA_PITCH; // 0 = horizontal; positive = leaning back
    public static final Angle CAMERA_YAW;
    public static final Distance X_ROBOT_TO_CAMERA_OFFSET; // positive = in front of robot center
    public static final Distance Y_ROBOT_TO_CAMERA_OFFSET; // positive = left of robot centerline
    public static final Distance Z_ROBOT_TO_CAMERA_OFFSET; // GROUND plane = 0 (not bellypan)
    public static final boolean HAS_SECOND_CAMERA;
    public static final String SECOND_LIMELIGHT_NAME;
    public static final Angle SECOND_CAMERA_PITCH; // 0 = horizontal; positive = leaning back
    public static final Angle SECOND_CAMERA_YAW;
    public static final Distance SECOND_X_ROBOT_TO_CAMERA_OFFSET; // positive = in front of robot center
    public static final Distance SECOND_Y_ROBOT_TO_CAMERA_OFFSET; // positive = left of robot centerline
    public static final Distance SECOND_Z_ROBOT_TO_CAMERA_OFFSET; // GROUND plane = 0 (not bellypan)

    static {
        switch (SubsystemFactory.getRobotType()) {
            case MOZART:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(15);
                CAMERA_YAW = Degrees.of(153);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(15);
                SECOND_CAMERA_YAW = Degrees.of(153);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                break;
            case BUBBLES:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(15);
                CAMERA_YAW = Degrees.of(153);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(15);
                SECOND_CAMERA_YAW = Degrees.of(153);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                break;
            case BETA:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(15);
                CAMERA_YAW = Degrees.of(153);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(15);
                SECOND_CAMERA_YAW = Degrees.of(153);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                break;
            default:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(15);
                CAMERA_YAW = Degrees.of(153);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(15);
                SECOND_CAMERA_YAW = Degrees.of(153);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
        }
    }
}
