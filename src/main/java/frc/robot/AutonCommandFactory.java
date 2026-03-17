package frc.robot;

import static frc.robot.constants.FieldConstants.PoseConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import java.util.List;
import java.util.Set;

public class AutonCommandFactory {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final CommandFactory commandFactory;
    private final Gyro gyro;
    private Pose2d finalPose;

    public final PathConstraints constraints =
            new PathConstraints(2, 5, Units.degreesToRadians(720), Units.degreesToRadians(720));

    public final PathConstraints intakeConstraints =
            new PathConstraints(
                    0.62, 5, Units.degreesToRadians(720), Units.degreesToRadians((720)));

    public AutonCommandFactory(
            DriveSubsystem drive,
            IntakeSubsystem intake,
            CommandFactory commandFactory,
            Gyro gyro) {
        this.drive = drive;
        this.intake = intake;
        this.commandFactory = commandFactory;
        this.gyro = gyro;
    }

    public Command buildPathDeferred(
            Pose pose, PathConstraints constraints, double goalEndVelocity) {

        return new DeferredCommand(
                () -> {
                    finalPose = pose.pose();
                    if (FieldConstants.alliance.isPresent()
                            && FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
                        finalPose = FlippingUtil.flipFieldPose(finalPose);
                    }
                    return AutoBuilder.pathfindToPose(finalPose, constraints, goalEndVelocity)
                            .withName(pose.name());
                },
                Set.of(drive));
    }

    public Command trenchToDepot() {
        return Commands.sequence(
                intake.stowArmSetpoint(),
                Commands.runOnce(() -> resetOdometryFlipped(BLUE_DEPOT_STARTING_LINE.pose())),
                buildPathDeferred(IN_THE_DEPOT, constraints, 0),
                drive.driveToPoseOnExecute(),
                intake.deployAndRunIntake().withTimeout(3),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(DEPOT_SHOOTING_SPOT, constraints, 0),
                drive.driveToPoseOnExecute(),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(6));
    }

    public Command depotSideDepotToNeutralZoneShooting() {
        return Commands.sequence(
                        trenchToDepot(),
                        followWaypoints(
                                DEPOT_SHOOTING_SPOT.pose(),
                                constraints,
                                1.5,
                                Rotation2d.fromDegrees(180),
                                DEPOT_SHOOTING_SPOT.pose(),
                                BLUE_DEPOT_STARTING_LINE.pose(),
                                BLUE_DEPOT_WALL_FUEL_CENTER.pose()))
                .withName("depot to neutral zone");
    }

    public Command depotSideNeutralZoneIntaking() {
        return Commands.sequence(
                        intake.stowArmSetpoint(),
                        commandFactory
                                .runSpindexShooterIndexAndShooterNoFeedFuel()
                                .withTimeout(1.5),
                        commandFactory.defaultSpindexerShooterIndexerAndShooter().withTimeout(0.01),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        Commands.runOnce(
                                () -> resetOdometryFlipped(BLUE_DEPOT_STARTING_LINE.pose())),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 0),
                        Commands.parallel(
                                buildPathDeferred(BLUE_DEPOT_BORDER_FUEL_CENTER, constraints, 0),
                                intake.deployAndRunIntake().withTimeout(0.01)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(DEPOT_NEUTRAL_ZONE_CENTER, intakeConstraints, 0)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(BLUE_DEPOT_BORDER_FUEL_CENTER, constraints, 0)),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 0),
                        buildPathDeferred(BLUE_DEPOT_STARTING_LINE, constraints, 0),
                        commandFactory.runSpindexShooterIndexAndShooter())
                .withName("depotToNeutralZone");
    }

    public Command outpostSideNeutralZoneIntaking() {
        return Commands.sequence(
                        intake.stowArmSetpoint(),
                        commandFactory
                                .runSpindexShooterIndexAndShooterNoFeedFuel()
                                .withTimeout(1.5),
                        commandFactory.defaultSpindexerShooterIndexerAndShooter().withTimeout(0.01),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        Commands.runOnce(
                                () -> resetOdometryFlipped(BLUE_OUTPOST_STARTING_LINE.pose())),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 0),
                        Commands.parallel(
                                buildPathDeferred(BLUE_OUTPOST_BORDER_FUEL_CENTER, constraints, 0),
                                intake.deployAndRunIntake().withTimeout(0.01)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        OUTPOST_NEUTRAL_ZONE_CENTER, intakeConstraints, 0)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(BLUE_OUTPOST_BORDER_FUEL_CENTER, constraints, 0)),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 0),
                        buildPathDeferred(OUTPOST_SHOOTING_SPOT, constraints, 0),
                        commandFactory.runSpindexShooterIndexAndShooter())
                .withName("outpostToNeutralZone");
    }

    public Command driveStraightTesting() {
        return Commands.sequence(
                intake.stowArmSetpoint(),
                Commands.runOnce(
                        () ->
                                resetOdometryFlipped(
                                        FieldConstants.PoseConstants.BLUE_OUTPOST_STARTING_LINE
                                                .pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.DRIVE_STRAIGHT_TESTING, intakeConstraints, 0),
                drive.driveToPoseOnExecute());
    }

    public Command followWaypoints(
            Pose2d startingPosition,
            PathConstraints constraints,
            double endVelocity,
            Rotation2d endRotation,
            Pose2d... poses) {
        return Commands.runOnce(
                () -> {
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

                    PathPlannerPath path =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(endVelocity, endRotation));

                    path.preventFlipping = false;
                    CommandScheduler.getInstance().schedule(AutoBuilder.followPath(path));
                });
    }

    public void resetOdometryFlipped(Pose2d pose) {
        boolean red = FieldConstants.alliance.get() == Alliance.Red;
        if (red) {
            pose = FlippingUtil.flipFieldPose(pose);
        }
        drive.resetOdometry(pose);
        gyro.setYaw(pose.getRotation().getMeasure());
        drive.resetOdometryAfterGyro();
    }
}
