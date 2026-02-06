// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.vision.VisionPoseEstimator;

public class RobotContainer {
    private SubsystemFactory subsystemFactory = new SubsystemFactory();
    private Gyro gyro = subsystemFactory.buildGyro();
    private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
    private IntakeSubsystem intakeSubsystem = subsystemFactory.buildIntake();
    // this is public because we need to run the visionPoseEstimator periodic from
    // Robot
    public VisionPoseEstimator visionPoseEstimator =
            new VisionPoseEstimator(drive, subsystemFactory.getRobotType());
    public CommandFactory commandFactory = new CommandFactory(drive, gyro);

    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    private static final CommandXboxController driverController =
            new CommandXboxController(DriveControlConstants.DRIVER_CONTROLLER_PORT);

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
                        true, () -> (driverController.a().getAsBoolean()))); // i BELIEVE that this returns whether or not the a button is pressed, but this should be tested!
      // note! do not bind to the a button; it is used in drive command for auto-orient!
        intakeSubsystem.setDefaultCommand(intakeSubsystem.defaultBehavior());
    }

    private void configureButtonBindingsDriver() {
        driverController.b().onTrue(gyro.setYaw(Degrees.of(0)));
        driverController.rightTrigger().whileTrue(intakeSubsystem.runIntake());
    }

    private void setUpAuton() {
        autoChooser.setDefaultOption("do nothing", Commands.none());
        SmartDashboard.putData("Autos/Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
