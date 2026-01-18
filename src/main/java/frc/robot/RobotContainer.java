// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.vision.VisionPoseEstimator;
import frc.robot.vision.LimelightHelpers.PoseEstimate;


public class RobotContainer {
  private SubsystemFactory subsystemFactory = new SubsystemFactory();
  private Gyro gyro = subsystemFactory.buildGyro();
  private DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);
  // this is public because we need to run the visionPoseEstimator periodic from
  // Robot
  public VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator(drive, subsystemFactory.getRobotType());
  public CommandFactory commandFactory = new CommandFactory(drive, gyro);

  private static SendableChooser<Command> autoChooser;
  private ComplexWidget autonChooserWidget;

  private static final CommandXboxController driverController = new CommandXboxController(
      DriveControlConstants.DRIVER_CONTROLLER_PORT);
  private static final CommandXboxController operatorController = new CommandXboxController(
      DriveControlConstants.OPERATOR_CONTROLLER_PORT);

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
    drive.setDefaultCommand(drive.driveCommand(
        () -> -(MathUtil.applyDeadband(
            driverController.getLeftY(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            driverController.getLeftX(),
            DriveControlConstants.DRIVE_DEADBAND)),
        () -> -(MathUtil.applyDeadband(
            driverController.getRightX(),
            DriveControlConstants.DRIVE_DEADBAND)),
        true));
  }

  private void configureButtonBindingsDriver() {
    
  }

  private void setUpAuton() {
    SmartDashboard.putData("Autos/Selector", autoChooser);

    SmartDashboard.putData("reset odometry for facing red wall", resetOdometryRed());
    SmartDashboard.putData("reset odometry for facing blue wall", resetOdometryBlue());

    autoChooser = new SendableChooser<>();

    Command hubDepot = Commands.sequence(
      commandFactory.buildPath(Constants.PoseConstants.DEPOT),
      // move into intake position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // ground intaking command from depot
      commandFactory.buildPath(Constants.PoseConstants.HUB_MIDDLE),
      // move into shooting position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // shooting command

    Command hubDepotTowerL1 = Commands.sequence(
      commandFactors.buildPath(Constants.PoseConstants.DEPOT),
      // move into intake position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // ground intaking command from depot
      commandFactory.buildPath(Constants.PoseConstants.HUB_MIDDLE),
      // move into shooting position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // shooting command
      commandFactory.buildPath(Constants.PoseConstants.TOWER_L1),
      // move into climbing position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // climbing command

    Command bumpDepotTowerL1 = Commands.sequence(
      commandFactory.buildPath(Constants.PoseConstants.DEPOT),
      // move into intake position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // ground intaking command from depot
      commandFactory.buildPath(Constants.PoseConstants.BUMP_STARTING_LINE),
      // move into shooting position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // shooting command
      commandFactory.buildPath(Constants.PoseConstants.TOWER_L1),
      // move into climbing position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // climbing command

    Command depotHubTowerL1 = Commands.sequence(
      // get into intake position while driving
      // ground intaking command from depot
      commandFactory.buildPath(Constants.PoseConstants.HUB_MIDDLE),
      // move into shooting position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // shooting command
      commandFactory.buildPath(Constants.PoseConstants.TOWER_L1),
      // move into climbing position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // climbing command

    Command bumpNeutralZone = Commands.sequence(
      commandFactory.buildPath(Constants.PoseConstants.NEUTRAL_ZONE_BORDER),
      // move into intaking position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // intaking command from neutral zone
      commandFactory.buildPath(Constants.PoseConstants.BUMP_STARTING_LINE),
      // get into shooting position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // shooting command

    Command bumpNeutralZoneTowerL1 = Commands.sequence(
      commandFactory.buildPath(Constants.PoseConstants.NEUTRAL_ZONE_BORDER),
      // move into intaking position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // intaking command from neutral zone
      commandFactory.buildPath(Constants.PoseConstants.BUMP_STARTING_LINE),
      // get into shooting position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // shooting command
      commandFactory.buildPath(Constants.PoseConstants.TOWER_L1),
      // get into climbing position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // climbing command

    Command bumpTowerL1 = Commands.sequence(
      commandFactory.buildPath(Constants.PoseConstants.NEUTRAL_ZONE_BORDER),
      // move into intaking position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()),
      // intaking command from neutral zone
      commandFactory.buildPath(Constants.PoseConstants.TOWER_L1),
      // get into climbing position while driving
      drive.driveToPoseOnExecute(() -> commandFactory.getRobotPosition()));
      // climbing command
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command resetOdometryRed() {
    return (gyro.setYaw(Degrees.of(0))).ignoringDisable(true).andThen(

        Commands.runOnce(() ->
        {

          SmartDashboard.putBoolean("reseting odometry red", true);
          var poseEstimate = visionPoseEstimator.getPoseEstimate();
          poseEstimate.ifPresent((PoseEstimate pose) -> {
            var poseCopy = pose.pose;
            drive.resetOdometry(new Pose2d(poseCopy.getTranslation(), new Rotation2d(gyro.getYaw())));
          });

        }).ignoringDisable(true));
  }

  public Command resetOdometryBlue() {

    return (gyro.setYaw(Degrees.of(180)).ignoringDisable(true)).andThen(
        Commands.runOnce(() ->

        {

          SmartDashboard.putBoolean("reseting odometry blue", true);
          var poseEstimate = visionPoseEstimator.getPoseEstimate();
          poseEstimate.ifPresent((PoseEstimate pose) -> {
            var poseCopy = pose.pose;
            drive.resetOdometry(new Pose2d(poseCopy.getTranslation(), new Rotation2d(gyro.getYaw())));
          });

        }).ignoringDisable(true));
  }


}