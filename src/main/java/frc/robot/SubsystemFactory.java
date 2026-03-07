package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.GearRatios;
import frc.robot.constants.RobotConstants.MotorIdConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardwareVortex;
import frc.robot.subsystems.drive.SwerveModulePlacebo;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroHardware;
import frc.robot.subsystems.drive.gyro.GyroPlacebo;
import frc.robot.subsystems.intake.IntakeArmHardwareBeta;
import frc.robot.subsystems.intake.IntakeArmPlacebo;
import frc.robot.subsystems.intake.IntakeRollerHardware;
import frc.robot.subsystems.intake.IntakeRollerPlacebo;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterHardwareBeta;
import frc.robot.subsystems.shooter.ShooterHardwarePrototype;
import frc.robot.subsystems.shooter.ShooterPlacebo;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerHardwareBeta;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerHardwarePrototype;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerPlacebo;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerHardwareBetaAndComp;
import frc.robot.subsystems.spindexer.SpindexerPlacebo;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class SubsystemFactory {
    private static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    private static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    private static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    private static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // we may need more, depending on how many subsystem + rio we have
    private static final String MOZART_SERIAL_NUMBER = "030ee8c8";
    private static final String BUBBLES_SERIAL_NUMBER = "030fc267";
    private static final String BETA_SERIAL_NUMBER = "030589d5";
    private static final String COMP_SERIAL_NUMBER = "";
    private static final String PROTOTYPE_SERIAL_NUMBER = "03260A64";

    public enum RobotType {
        MOZART,
        BUBBLES,
        SIM,
        BETA,
        COMP,
        PROTOTYPE
    }

    private static RobotType robotType;

    private String serialNum = System.getenv("serialnum");

    public SubsystemFactory() {
        if (RobotBase.isSimulation()) {
            robotType = RobotType.SIM;
            serialNum = "simulation";
        } else if (serialNum == null) {
            robotType = null;
            throw new RuntimeException("NO SERIAL NUMBER (cannot identify robot based on rio)");
        } else if (serialNum.equals(BETA_SERIAL_NUMBER)) {
            robotType = RobotType.BETA;
        } else if (serialNum.equals(COMP_SERIAL_NUMBER)) {
            robotType = RobotType.COMP;
        } else if (serialNum.equals(BUBBLES_SERIAL_NUMBER)) {
            robotType = RobotType.BUBBLES;
        } else if (serialNum.equals(MOZART_SERIAL_NUMBER)) {
            robotType = RobotType.MOZART;
        } else if (serialNum.equals(PROTOTYPE_SERIAL_NUMBER)) {
            robotType = RobotType.PROTOTYPE;
        } else {
            throw new RuntimeException(
                    "UNKNOWN SERIAL NUMBER (cannot identify robot based on rio) \nserial number of current rio: "
                            + serialNum);
        }
        SmartDashboard.putString("robot/serialNum", serialNum);
        SmartDashboard.putString("robot/robotType", robotType.name());
    }

    public static RobotType getRobotType() {
        return robotType;
    }

    public DriveSubsystem buildDriveSubsystem(Gyro gyro) {
        SwerveModule frontLeft;
        SwerveModule rearLeft;
        SwerveModule frontRight;
        SwerveModule rearRight;
        if (robotType == RobotType.BETA
                || robotType == RobotType.BUBBLES
                || robotType == RobotType.MOZART) {
            frontLeft =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
                                    "front left"));
            frontRight =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
                                    "front right"));
            rearLeft =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET,
                                    "rear left"));
            rearRight =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET,
                                    "rear right"));
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
        if (robotType == RobotType.COMP
                || robotType == RobotType.BETA
                || robotType == RobotType.BUBBLES
                || robotType == RobotType.MOZART) {
            return new Gyro(new GyroHardware());
        } else {
            return new Gyro(new GyroPlacebo());
        }
    }

    public IntakeSubsystem buildIntake() {
        if (robotType == RobotType.MOZART) {
            return new IntakeSubsystem(
                    new IntakeRollerHardware(
                            RobotConstants.MotorIdConstants.INTAKE_ROLLER_ALPHA_CAN_ID),
                    new IntakeArmPlacebo());
        } else if (robotType == RobotType.BETA) {
            return new IntakeSubsystem(
                    new IntakeRollerHardware(
                            RobotConstants.MotorIdConstants.INTAKE_ROLLER_BETA_CAN_ID),
                    new IntakeArmHardwareBeta());
        } else {
            return new IntakeSubsystem(new IntakeRollerPlacebo(), new IntakeArmPlacebo());
        }
    }

    public ShooterSubsystem buildShooter() {
        if (robotType == RobotType.PROTOTYPE) {
            return new ShooterSubsystem(new ShooterHardwarePrototype());
        } else if (robotType == RobotType.BETA) {
            return new ShooterSubsystem(new ShooterHardwareBeta());
        } else {
            return new ShooterSubsystem(new ShooterPlacebo());
        }
    }

    public SpindexerSubsystem buildSpindexer() {
        if (robotType == RobotType.BETA) {
            return new SpindexerSubsystem(
                    new SpindexerHardwareBetaAndComp(GearRatios.BETA_SPINDEXER_GEAR_RATIO));
        } else if (robotType == RobotType.COMP) {
            return new SpindexerSubsystem(
                    new SpindexerHardwareBetaAndComp(GearRatios.COMP_SPINDEXER_GEAR_RATIO));
        } else {
            return new SpindexerSubsystem(new SpindexerPlacebo());
        }
    }

    public ShooterIndexerSubsystem buildShooterIndexer() {
        if (robotType == RobotType.PROTOTYPE) {
            return new ShooterIndexerSubsystem(new ShooterIndexerHardwarePrototype());
        } else if (robotType == RobotType.BETA) {
            return new ShooterIndexerSubsystem(new ShooterIndexerHardwareBeta());
        } else {
            return new ShooterIndexerSubsystem(new ShooterIndexerPlacebo());
        }
    }
}
