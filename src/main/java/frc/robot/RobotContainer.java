// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;
import frc.robot.vision.VisionPoseEstimator;

public class RobotContainer {
    private SubsystemFactory subsystemFactory = new SubsystemFactory();
    private final Alert driverDisconnected =
            new Alert("Driver controller disconnected!", AlertType.kWarning);
    private final Alert noAutonSelectedAlert =
            new Alert("Auton is not selected!", AlertType.kWarning);
    private Gyro gyro = subsystemFactory.buildGyro();
    private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
    private IntakeSubsystem intakeSubsystem = subsystemFactory.buildIntake();
    private IndexerSubsystem indexerSubsystem = subsystemFactory.buildIndexer();
    private IntakeArmSubsystem intakeArmSubsystem = subsystemFactory.buildIntakeArm();
    // this is public because we need to run the visionPoseEstimator periodic from
    // Robot
    public VisionPoseEstimator visionPoseEstimator =
            new VisionPoseEstimator(drive, subsystemFactory.getRobotType());
    public CommandFactory commandFactory =
            new CommandFactory(drive, gyro, intakeSubsystem, intakeArmSubsystem);

    private static SendableChooser<Command> autoChooser = new SendableChooser<>();
    private Command defaultCommand = Commands.none();

    private static final CommandXboxController driverController =
            new CommandXboxController(DriveControlConstants.DRIVER_CONTROLLER_PORT);

    private final Alert lowBatteryAlert =
            new Alert(
                    "Battery voltage is very low, turn off the robot or replace the battery to avoid damage.",
                    AlertType.kWarning);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        configureDefaultCommands();
        configureButtonBindingsDriver();
        setUpAuton();
    }

    public void disableSubsystems() {
        drive.disableDriveToPose();
    }

    public void configureDefaultCommands() {
        drive.setDefaultCommand(
                drive.driveCommand(
                        () ->
                                -(MathUtil.applyDeadband(
                                        driverController.getLeftY(),
                                        DriveControlConstants.DRIVE_DEADBAND)),
                        () ->
                                -(MathUtil.applyDeadband(
                                        driverController.getLeftX(),
                                        DriveControlConstants.DRIVE_DEADBAND)),
                        () ->
                                -(MathUtil.applyDeadband(
                                        driverController.getRightX(),
                                        DriveControlConstants.DRIVE_DEADBAND)),
                        true));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.defaultBehavior());
        indexerSubsystem.setDefaultCommand(indexerSubsystem.defaultBehavior());
    }

    private void configureButtonBindingsDriver() {
        driverController.b().onTrue(gyro.setYaw(Degrees.of(0)));
        driverController.rightTrigger().whileTrue(commandFactory.runIntakeandIntakeArm());
        driverController.rightBumper().whileTrue(indexerSubsystem.runIndexer());
    }

    private void setUpAuton() {
        autoChooser.setDefaultOption("do nothing", defaultCommand);
        SmartDashboard.putData("Autos/Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setAlerts() {
        lowBatteryAlert.set(
                (RobotController.getBatteryVoltage() > 0.0
                        && RobotController.getBatteryVoltage()
                                <= DriveControlConstants.LOW_BATTERY_VOLTAGE));
        driverDisconnected.set(
                !DriverStation.isJoystickConnected(driverController.getHID().getPort()));
        noAutonSelectedAlert.set(
                DriverStation.isAutonomous()
                        && !DriverStation.isEnabled()
                        && getAutonomousCommand() == defaultCommand);
    }
}
