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
import frc.robot.util.FieldCalculationHelpers;
import frc.robot.util.GameState;
import frc.robot.vision.VisionPoseEstimator;
import java.util.Optional;

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
            new AutonCommandFactory(
                    drive,
                    intakeSubsystem,
                    commandFactory,
                    gyro,
                    shooterSubsystem,
                    shooterIndexerSubsystem);

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
    private final Alert csvIssueAlert =
            new Alert(
                    "The CSV file used for shooter speeds has an error. MUST USE MANUAL SHOOT. (if this disappears you can use autoshoot)",
                    AlertType.kWarning);

    private boolean isGyroConfigured = false;
    private boolean autonHasNoGyroAngle = false;
    private String autonGyroConfiguredFor = "";

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
                        () ->
                                (driverController.leftBumper().getAsBoolean()
                                        || driverController.leftTrigger().getAsBoolean())));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.defaultBehavior());
        shooterSubsystem.setDefaultCommand(shooterSubsystem.defaultBehavior());
        spindexerSubsystem.setDefaultCommand(spindexerSubsystem.defaultBehavior());
        shooterIndexerSubsystem.setDefaultCommand(shooterIndexerSubsystem.defaultBehavior());
    }

    private void configureButtonBindingsDriver() {
        Trigger canShootIntoHub = new Trigger(() -> GameState.isHubActive(0));

        // note! do not bind to the left bumper button; it is used in drive command for auto-orient!
        driverController.a().whileTrue(intakeSubsystem.deployAndRunIntakeBackwards());
        driverController.b().onTrue(commandFactory.resetHeading(Degrees.of(0)));
        driverController.rightTrigger().whileTrue(intakeSubsystem.deployAndRunIntake());
        driverController
                .leftTrigger()
                .whileTrue(commandFactory.runSpindexShooterIndexAndShooter(false)); // do not pass
        driverController
                .rightBumper()
                .whileTrue(commandFactory.runSpindexShooterIndexAndShooter(true)); // do pass
        driverController.x().whileTrue(drive.setX());
        driverController.y().whileTrue(spindexerSubsystem.runSpindexerBackwards());
    }

    private void configureButtonBindingsTuningController() {
        tuningController.rightTrigger().whileTrue(intakeSubsystem.deployAndRunIntake());
        tuningController
                .leftTrigger()
                .whileTrue(
                        shooterSubsystem.shoot(
                                () ->
                                        RebuiltVisionUtil.getDistanceToAlignmentTarget(
                                                () -> drive.getPose()),
                                false,
                                () ->
                                        FieldCalculationHelpers.getAlignmentTargetType(
                                                () -> drive.getPose())));
        tuningController.rightBumper().whileTrue(spindexerSubsystem.runSpindexer());
        tuningController.leftBumper().whileTrue(intakeSubsystem.stowArmSetpoint());
        tuningController.x().whileTrue(shooterIndexerSubsystem.runShooterIndexer());
        tuningController.y().whileTrue(intakeSubsystem.runIntakeArmInVelocity());
        tuningController.povLeft().whileTrue(intakeSubsystem.runIntakeArmOutVelocity());
        tuningController.b().whileTrue(intakeSubsystem.runRoller());
        tuningController.a().whileTrue(shooterSubsystem.tuningSetpoint());
        tuningController.povRight().whileTrue(intakeSubsystem.feedFuel());
        tuningController
                .povUp()
                // .and(tuningController.a())
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        shooterSubsystem.logShooterSpeedsToCSV(
                                                RebuiltVisionUtil.getDistanceToAlignmentTarget(
                                                        () -> drive.getPose()))));
        tuningController.povDown().onTrue(commandFactory.resetHeading(Degrees.of(180)));
    }

    private void setUpAuton() {
        autoChooser = new SendableChooser<>();
        autoChooser.addOption(
                "depot side to neutral zone then back and shoot",
                autonCommandFactory.depotSideNeutralZoneAndBackWithShooting());
        autoChooser.addOption(
                "outpost side to neutral zone then back and shoot",
                autonCommandFactory.outpostSideNeutralZoneAndBackWithShooting());
        autoChooser.addOption(
                "center back up and shoot preload", autonCommandFactory.centerMoveAndShoot());
        autoChooser.addOption(
                "move from center to depot and shoot", autonCommandFactory.hubMoveToDepot());
        autoChooser.addOption(
                "SCOOP depot side neutral zone back and shoot",
                autonCommandFactory.depotSideNeutralZoneScoop());
        autoChooser.addOption(
                "SCOOP outpost side neutral zone back and shoot",
                autonCommandFactory.outpostSideNeutralZoneScoop());
        autoChooser.setDefaultOption("do nothing", defaultCommand);
        SmartDashboard.putData("Autos/Selector", autoChooser);
        SmartDashboard.putData(
                "Autos/configure gyro (CHOOSE AUTON THEN CLICK ME!)", resetGyroByAuton());
        SmartDashboard.putData("alliance/reset blue", resetAllianceBlue());
        SmartDashboard.putData("alliance/reset red", resetAllianceRed());
    }

    public Command resetGyroByAuton() {
        return Commands.runOnce(
                        () -> {
                            FieldConstants.alliance = DriverStation.getAlliance();
                            // red outpost side needs -90, blue outpost needs +90
                            // red depot needs 90, blue depot needs -90
                            // for consistency with current pose names, the gyro degrees in the
                            // command name is for BLUE
                            Command selectedAuton = getAutonomousCommand();
                            String autonName = selectedAuton.getName();
                            autonGyroConfiguredFor = autonName;
                            Angle gyroAngle;
                            boolean isCenterAuton = false;
                            switch (autonName) {
                                case "outpost side to neutral zone and then back and shoot":
                                    gyroAngle = Degrees.of(90);
                                    autonHasNoGyroAngle = false;
                                    break;
                                case "depot side to neutral zone then back and shoot":
                                    gyroAngle = Degrees.of(-90);
                                    autonHasNoGyroAngle = false;
                                    break;
                                case "move from center and shoot preload":
                                    gyroAngle = Degrees.of(0);
                                    autonHasNoGyroAngle = false;
                                    isCenterAuton = true;
                                    break;
                                case "move from center to depot and shoot":
                                    gyroAngle = Degrees.of(0);
                                    autonHasNoGyroAngle = false;
                                    isCenterAuton = true;
                                    break;
                                default:
                                    gyroAngle = Degrees.of(0);
                                    autonHasNoGyroAngle = true;
                                    break;
                            }
                            if (FieldConstants.alliance.isPresent()
                                    && FieldConstants.alliance.get() == Alliance.Red) {
                                if (isCenterAuton) {
                                    gyroAngle = Degrees.of(180);
                                } else {
                                    gyroAngle = gyroAngle.unaryMinus();
                                }
                            }
                            gyro.setYaw(gyroAngle);
                            drive.resetOdometryAfterGyro();
                            isGyroConfigured = true;
                        })
                .ignoringDisable(true);
    }

    public Command resetAllianceBlue() {
        return Commands.runOnce(
                        () -> {
                            FieldConstants.alliance = Optional.of(Alliance.Blue);
                            isGyroConfigured = false;
                        })
                .ignoringDisable(true);
    }

    public Command resetAllianceRed() {
        return Commands.runOnce(
                        () -> {
                            FieldConstants.alliance = Optional.of(Alliance.Red);
                            isGyroConfigured = false;
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
        gyroNeedsConfiguringAlert.set(
                !isGyroConfigured
                        || (DriverStation.isAutonomous()
                                && (!getAutonomousCommand()
                                        .getName()
                                        .equals(autonGyroConfiguredFor))));
        autonHasNoGyroAngleAlert.set(autonHasNoGyroAngle);
        csvIssueAlert.set(shooterSubsystem.isErrorInCSVFile());
    }
}
