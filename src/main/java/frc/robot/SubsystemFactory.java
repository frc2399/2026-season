package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorIdConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveConfig;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardwareVortex;
import frc.robot.subsystems.drive.SwerveModulePlacebo;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroHardware;
import frc.robot.subsystems.gyro.GyroPlacebo;

public class SubsystemFactory {
    private static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    private static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    private static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    private static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // we may need more, depending on how many subsystem + rio we have
    private static final String MOZART_SERIAL_NUMBER = "030ee8c8";
    private static final String BUBBLES_SERIAL_NUMBER = "030fc267";
    private static final String BETA_SERIAL_NUMBER = "";
    private static final String COMP_SERIAL_NUMBER = "";

    public enum RobotType {
        MOZART,
        BUBBLES,
        SIM,
        BETA,
        COMP
    }

    private static RobotType robotType;

    private String serialNum = System.getenv("serialnum");

    public SubsystemFactory() {
        if (serialNum.equals(BETA_SERIAL_NUMBER)) {
            robotType = RobotType.BETA;
        } else if (serialNum.equals(COMP_SERIAL_NUMBER)) {
            robotType = RobotType.COMP;
        } else if (serialNum.equals(BUBBLES_SERIAL_NUMBER)) {
            robotType = RobotType.BUBBLES;
        } else if (serialNum.equals(MOZART_SERIAL_NUMBER)) {
            robotType = RobotType.MOZART;
        } else {
            robotType = RobotType.SIM;
        }
    }

    public static RobotType getRobotType() {
        return robotType;
    }

    public DriveSubsystem buildDriveSubsystem(Gyro gyro) {
        SwerveModule frontLeft;
        SwerveModule rearLeft;
        SwerveModule frontRight;
        SwerveModule rearRight;

        // if (robotType == RobotType.ALPHA) {
        //     frontLeft = new SwerveModule(new SwerveModuleHardwareNEO(
        //             MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
        //             MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
        //             FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
        //     frontRight = new SwerveModule(new SwerveModuleHardwareNEO(
        //             MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
        //             MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
        //             FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
        //     rearLeft = new SwerveModule(new SwerveModuleHardwareNEO(
        //             MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
        //             MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
        //             REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
        //     rearRight = new SwerveModule(new SwerveModuleHardwareNEO(
        //             MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
        //             MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
        //             REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
        //     return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro,
        //             Constants.DriveControlConstants.ALPHA_TRACK_WIDTH,
        //             Constants.DriveControlConstants.ALPHA_TRACK_WIDTH);
        // } else
        if (robotType == RobotType.BETA || robotType == RobotType.BUBBLES) {
            frontLeft = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        } else if (robotType == RobotType.MOZART) {
            frontLeft = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        } else {
            frontLeft = new SwerveModule(new SwerveModulePlacebo());
            frontRight = new SwerveModule(new SwerveModulePlacebo());
            rearLeft = new SwerveModule(new SwerveModulePlacebo());
            rearRight = new SwerveModule(new SwerveModulePlacebo());
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        }
    }

    public Gyro buildGyro() {
        if (robotType == RobotType.COMP || robotType == RobotType.BETA || robotType == RobotType.BUBBLES
                || robotType == RobotType.MOZART) {
            return new Gyro(new GyroHardware());
        } else {
            return new Gyro(new GyroPlacebo());
        }
    }
}