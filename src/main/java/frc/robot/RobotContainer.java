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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.RebuiltVisionUtil;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.util.GameState;
import frc.robot.vision.VisionPoseEstimator;

public class RobotContainer {
    private SubsystemFactory subsystemFactory = new SubsystemFactory();
    public Gyro gyro = subsystemFactory.buildGyro();

    private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
    private IntakeSubsystem intakeSubsystem = subsystemFactory.buildIntake();
    private ShooterSubsystem shooterSubsystem = subsystemFactory.buildShooter();
    private SpindexerSubsystem spindexerSubsystem = subsystemFactory.buildSpindexer();
    private ShooterIndexerSubsystem shooterIndexerSubsystem =
            subsystemFactory.buildShooterIndexer();

    // this is public because we need to run the visionPoseEstimator periodic from
    // Robot
    public VisionPoseEstimator visionPoseEstimator =
            new VisionPoseEstimator(drive, subsystemFactory.getRobotType());
    public CommandFactory commandFactory =
            new CommandFactory(
                    drive,
                    gyro,
                    shooterSubsystem,
                    shooterIndexerSubsystem,
                    spindexerSubsystem,
                    intakeSubsystem);
    public AutonCommandFactory autonCommandFactory =
            new AutonCommandFactory(drive, intakeSubsystem);

    private static SendableChooser<Command> autoChooser = new SendableChooser<>();
    private Command defaultCommand = Commands.none();

    private static final CommandXboxController driverController =
            new CommandXboxController(DriveControlConstants.DRIVER_CONTROLLER_PORT);
    private static final CommandXboxController tuningController =
            new CommandXboxController(DriveControlConstants.TUNING_CONTROLLER_PORT);

    private final Alert driverDisconnected =
            new Alert("Driver controller disconnected!", AlertType.kWarning);
    private final Alert noAutonSelectedAlert =
            new Alert("Auton is not selected!", AlertType.kWarning);
    private final Alert lowBatteryAlert =
            new Alert(
                    "Battery voltage is very low, turn off the robot or replace the battery to avoid damage.",
                    AlertType.kWarning);

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindingsDriver();
        configureButtonBindingsTuningController();
        setUpAuton();

        SmartDashboard.putData("robot/driverController", driverController.getHID());
    }

    public void disableSubsystems() {
        drive.disableDriveToPose();
        intakeSubsystem.armProfiledPidEnabled = false;
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
                        true,
                        () -> (driverController.a().getAsBoolean())));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.defaultBehavior());
        shooterSubsystem.setDefaultCommand(shooterSubsystem.defaultBehavior());
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.defaultBehavior());
        shooterIndexerSubsystem.setDefaultCommand(shooterIndexerSubsystem.defaultBehavior());
    }

    private void configureButtonBindingsDriver() {
        Trigger canShootIntoHub = new Trigger(() -> GameState.isHubActive(0));

        // note! do not bind to the a button; it is used in drive command for auto-orient!
        driverController.b().onTrue(commandFactory.resetHeading(Degrees.of(0)));
        driverController.rightTrigger().whileTrue(commandFactory.runIntakeandIntakeArm());
        driverController.leftBumper().whileTrue(commandFactory.runSpindexShooterIndexAndShooter());
    }

    private void configureButtonBindingsTuningController() {
        tuningController.rightTrigger().onTrue(intakeSubsystem.deployArm());
        tuningController.leftTrigger().whileTrue(shooterSubsystem.shoot(RebuiltVisionUtil.getDistanceToHub(() -> drive.getPose())));
        tuningController.rightBumper().whileTrue(spindexerSubsystem.runSpindexer());
        tuningController.leftBumper().onTrue(intakeSubsystem.stowArm());
        tuningController.x().whileTrue(shooterIndexerSubsystem.runShooterIndexer());
        tuningController.y().whileTrue(intakeSubsystem.runIntakeArmInVelocity());
        tuningController.povLeft().whileTrue(intakeSubsystem.runIntakeArmOutVelocity());
        tuningController.b().whileTrue(intakeSubsystem.runRoller());
        tuningController.a().whileTrue(shooterSubsystem.tuningSetpoint());
        tuningController
                .povUp()
                // .and(tuningController.a())
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        shooterSubsystem.logShooterSpeedsToCSV(
                                                RebuiltVisionUtil.getDistanceToHub(
                                                        () -> drive.getPose()))));
        tuningController.povDown().onTrue(commandFactory.resetHeading(Degrees.of(180)));
    }

    private void setUpAuton() {
        autoChooser = new SendableChooser<>();
        // autoChooser.addOption(
        //         "bumpToNeutralZoneShooting", autonCommandFactory.bumpToNeutralZoneShooting());
        autoChooser.addOption("hubToDepot", autonCommandFactory.hubToDepot());
        // autoChooser.addOption("bumpToNeutralZone", autonCommandFactory.bumpToNeutralZone());
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
