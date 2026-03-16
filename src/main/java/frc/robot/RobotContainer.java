// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
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

    public DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
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
            new AutonCommandFactory(drive, intakeSubsystem, commandFactory, gyro);

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
    private final Alert gyroNeedsConfiguringAlert =
            new Alert(
                    "Gyro not configured! Auton + vision will not work; click the button below auton selector.",
                    AlertType.kWarning);
    private final Alert autonHasNoGyroAngleAlert =
            new Alert("Selected auton will not configure gyro properly!", AlertType.kWarning);

    private boolean isGyroConfigured = false;
    private boolean autonHasNoGyroAngle = false;

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
                        () -> (driverController.leftBumper().getAsBoolean())));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.defaultBehavior());
        shooterSubsystem.setDefaultCommand(shooterSubsystem.defaultBehavior());
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.defaultBehavior());
        shooterIndexerSubsystem.setDefaultCommand(shooterIndexerSubsystem.defaultBehavior());
    }

    private void configureButtonBindingsDriver() {
        Trigger canShootIntoHub = new Trigger(() -> GameState.isHubActive(0));

        // note! do not bind to the a button; it is used in drive command for auto-orient!
        driverController.b().onTrue(commandFactory.resetHeading(Degrees.of(0)));
        driverController.rightTrigger().whileTrue(intakeSubsystem.deployAndRunIntake());
        driverController.leftTrigger().whileTrue(commandFactory.runSpindexShooterIndexAndShooter());
        driverController.x().whileTrue(drive.setX());
        driverController.y().whileTrue(spindexerSubsystem.runSpindexerBackwards());
        driverController.a().whileTrue(intakeSubsystem.feedFuel());
    }

    private void configureButtonBindingsTuningController() {
        tuningController.rightTrigger().onTrue(intakeSubsystem.deployArm());
        tuningController
                .leftTrigger()
                .whileTrue(
                        shooterSubsystem.shoot(
                                () -> RebuiltVisionUtil.getDistanceToHub(() -> drive.getPose())));
        tuningController.rightBumper().whileTrue(spindexerSubsystem.runSpindexer());
        tuningController.leftBumper().onTrue(intakeSubsystem.stowArmSetpoint());
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
        autoChooser.addOption("trenchToDepot", autonCommandFactory.trenchToDepot());
        autoChooser.addOption(
                "depotSideDepotToNeutralZone",
                autonCommandFactory.depotSideDepotToNeutralZoneShooting());
        autoChooser.addOption(
                "depotSideNeutralZoneIntaking", autonCommandFactory.depotSideNeutralZoneIntaking());
        autoChooser.addOption(
                "outpostSideNeutralZoneIntaking",
                autonCommandFactory.outpostSideNeutralZoneIntaking());
        autoChooser.setDefaultOption("do nothing", defaultCommand);
        autoChooser.setDefaultOption("drive straight", autonCommandFactory.driveStraightTesting());
        SmartDashboard.putData("Autos/Selector", autoChooser);
        SmartDashboard.putData(
                "Autos/configure gyro (CHOOSE AUTON THEN CLICK ME!)", resetGyroByAuton());
    }

    public Command resetGyroByAuton() {
        return Commands.runOnce(
                        () -> {
                            // red outpost side needs -90, blue outpost needs +90
                            // red depot needs 90, blue depot needs -90
                            // for consistency with current pose names, the gyro degrees in the
                            // command name is for BLUE
                            Command selectedAuton = getAutonomousCommand();
                            String autonName = selectedAuton.getName();
                            String[] wordsInAutonName = autonName.split("BLUE GYRO ANGLE: ");
                            if (wordsInAutonName.length < 1) {
                                autonHasNoGyroAngle = true;
                            } else {
                                String autonAngleString = wordsInAutonName[1];
                                try {
                                    Angle desiredGyroAngle =
                                            Degrees.of(Integer.parseInt(autonAngleString));
                                    if (FieldConstants.alliance.isPresent()
                                            && FieldConstants.alliance.get() == Alliance.Red) {
                                        desiredGyroAngle = desiredGyroAngle.unaryMinus();
                                    }
                                    gyro.setYaw(desiredGyroAngle);
                                    drive.resetOdometryAfterGyro();
                                    autonHasNoGyroAngle = false;
                                    isGyroConfigured = true;
                                    SmartDashboard.putBoolean(
                                            "robot/has reset gyro", isGyroConfigured);
                                } catch (NumberFormatException e) {
                                    System.err.println(e.getStackTrace());
                                    autonHasNoGyroAngle = true;
                                }
                            }
                        })
                .ignoringDisable(true);
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
        gyroNeedsConfiguringAlert.set(!isGyroConfigured);
        autonHasNoGyroAngleAlert.set(autonHasNoGyroAngle);
    }
}
