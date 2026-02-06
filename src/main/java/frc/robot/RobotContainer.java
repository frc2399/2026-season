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
import frc.robot.constants.FieldConstants;
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
                        true));
        intakeSubsystem.setDefaultCommand(intakeSubsystem.defaultBehavior());
    }

    private void configureButtonBindingsDriver() {
        driverController.b().onTrue(gyro.setYaw(Degrees.of(0)));
        driverController.rightTrigger().whileTrue(intakeSubsystem.runIntake());
    }

  private void setUpAuton() {
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("do nothing", Commands.none());

   Command hubDepot = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose())),
     commandFactory.buildPath(FieldConstants.PoseConstants.DEPOT),
     // move into intake position while driving
     drive.driveToPoseOnExecute(),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.HUB_MIDDLE),
     // move into shooting position while driving
     drive.driveToPoseOnExecute());
     // shooting command


   Command hubDepotTowerL1 = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose())),
     commandFactory.buildPath(FieldConstants.PoseConstants.DEPOT),
     // move into intake position while driving
     drive.driveToPoseOnExecute(),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.HUB_MIDDLE),
     // move into shooting position while driving
     drive.driveToPoseOnExecute(),
     // shooting command
     commandFactory.buildPath(FieldConstants.PoseConstants.TOWER_L1),
     // move into climbing position while driving
     drive.driveToPoseOnExecute());
     // climbing command


   Command bumpDepotTowerL1 = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
     commandFactory.buildPath(FieldConstants.PoseConstants.DEPOT),
     // move into intake position while driving
     drive.driveToPoseOnExecute(),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.BUMP_STARTING_LINE),
     // move into shooting position while driving
     drive.driveToPoseOnExecute(),
     // shooting command
     commandFactory.buildPath(FieldConstants.PoseConstants.TOWER_L1),
     // move into climbing position while driving
     drive.driveToPoseOnExecute());
     // climbing command


   Command depotHubTowerL1 = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.DEPOT.pose())),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.HUB_MIDDLE),
     // move into shooting position while driving
     drive.driveToPoseOnExecute(),
     // shooting command
     commandFactory.buildPath(FieldConstants.PoseConstants.TOWER_L1),
     // move into climbing position while driving
     drive.driveToPoseOnExecute());
     // climbing command


   Command bumpNeutralZone = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
     commandFactory.buildPath(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER),
     // move into intaking position while driving
     drive.driveToPoseOnExecute(),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.BUMP_STARTING_LINE),
     // get into shooting position while driving
     drive.driveToPoseOnExecute());
     // shooting command


   Command bumpNeutralZoneTowerL1 = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
     commandFactory.buildPath(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER),
     // move into intaking position while driving
     drive.driveToPoseOnExecute(),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.BUMP_STARTING_LINE),
     // get into shooting position while driving
     drive.driveToPoseOnExecute(),
     // shooting command
     commandFactory.buildPath(FieldConstants.PoseConstants.TOWER_L1),
     // get into climbing position while driving
     drive.driveToPoseOnExecute());
     // climbing command


   Command bumpTowerL1 = Commands.sequence(
     Commands.runOnce(() -> drive.resetOdometry(FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
     commandFactory.buildPath(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER),
     // move into intaking position while driving
     drive.driveToPoseOnExecute(),
     intakeSubsystem.runIntake(),
     commandFactory.buildPath(FieldConstants.PoseConstants.TOWER_L1),
     // get into climbing position while driving
     drive.driveToPoseOnExecute());
     // climbing command

    autoChooser.addOption("hubDepot", hubDepot);
    autoChooser.addOption("hubDepotTowerL1", hubDepotTowerL1);
    autoChooser.addOption("bumpDepotTowerL1", bumpDepotTowerL1);
    autoChooser.addOption("depotHubTowerL1", depotHubTowerL1);
    autoChooser.addOption("bumpNeutralZone", bumpNeutralZone);
    autoChooser.addOption("bumpNeutralZoneTowerL1", bumpNeutralZoneTowerL1);
    autoChooser.addOption("bumpTowerL1", bumpTowerL1);

    SmartDashboard.putData("Autos/Selector", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
