package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import java.util.List;
import java.util.Set;

public class AutonCommandFactory {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final CommandFactory commandFactory;
    private Pose2d finalPose;

    public final PathConstraints constraints =
            new PathConstraints(2, 5, Units.degreesToRadians(720), Units.degreesToRadians(720));

    public final PathConstraints intakeConstraints =
            new PathConstraints(
                    0.75, 5, Units.degreesToRadians(720), Units.degreesToRadians((720)));

    public AutonCommandFactory(
            DriveSubsystem drive, IntakeSubsystem intake, CommandFactory commandFactory) {
        this.drive = drive;
        this.intake = intake;
        this.commandFactory = commandFactory;
    }

    public Command buildPathDeferred(
            Pose pose, PathConstraints constraints, double goalEndVelocity) {
        finalPose = pose.pose();
        if (FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
            finalPose = FlippingUtil.flipFieldPose(finalPose);
        }

        return new DeferredCommand(
                () ->
                        AutoBuilder.pathfindToPose(finalPose, constraints, goalEndVelocity)
                                .withName(pose.name()),
                Set.of(drive));
    }

    public Command trenchToDepot() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BLUE_DEPOT_STARTING_LINE
                                                .pose())),
                buildPathDeferred(FieldConstants.PoseConstants.IN_THE_DEPOT, constraints, 0),
                // drive.driveToPoseOnExecute(),
                commandFactory.runIntakeandIntakeArm().withTimeout(3),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT_SHOOTING_SPOT, constraints, 0),
                // drive.driveToPoseOnExecute(),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(6));
    }

    public Command depotSideDepotToNeutralZoneShooting() {
        return Commands.sequence(
                trenchToDepot(),
                depotSideShootingSpotToNeutralZoneWaypoints());
    }

    public Command depotSideNeutralZoneIntaking() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BLUE_DEPOT_STARTING_LINE
                                                .pose())),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT_SHOOTING_SPOT, constraints, 0),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(2),
                depotSideShootingSpotToNeutralZoneWaypoints(),
                // drive.driveToPoseOnExecute(),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        intake.defaultBehavior().withTimeout(0.01),
                        depotSideNeutralZoneIntakingWaypoints()),
                depotSideNeutralZoneToShootingSpotWaypoints(),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(5));
    }

    public Command outpostSideNeutralZoneIntaking() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BLUE_OUTPOST_STARTING_LINE
                                                .pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OUTPOST_SHOOTING_SPOT, constraints, 0),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(2),
                outpostSideShootingSpotToNeutralZoneWaypoints(),
                // drive.driveToPoseOnExecute(),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        intake.defaultBehavior().withTimeout(0.01),
                        outpostSideNeutralZoneIntakingWaypoints()),
                outpostSideNeutralZoneToShootingSpotWaypoints(),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(5));
    }

    public Command depotSideShootingSpotToNeutralZoneWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(FieldConstants.PoseConstants.DEPOT_SHOOTING_SPOT.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.DEPOT_SHOOTING_SPOT.pose(),
                                    FieldConstants.PoseConstants.BLUE_DEPOT_STARTING_LINE.pose(),
                                    FieldConstants.PoseConstants.BLUE_DEPOT_WALL_FUEL_CENTER.pose());

                    PathPlannerPath depotSideShootingSpotToNeutralZone =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    depotSideShootingSpotToNeutralZone.preventFlipping = true;

                    AutoBuilder.followPath(depotSideShootingSpotToNeutralZone).schedule();
                });
    }

    public Command outpostSideShootingSpotToNeutralZoneWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(FieldConstants.PoseConstants.OUTPOST_SHOOTING_SPOT.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.OUTPOST_SHOOTING_SPOT.pose(),
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_STARTING_LINE.pose(),
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_WALL_FUEL_CENTER.pose());

                    PathPlannerPath outpostSideShootingSpotToNeutralZone =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    outpostSideShootingSpotToNeutralZone.preventFlipping = true;

                    AutoBuilder.followPath(outpostSideShootingSpotToNeutralZone).schedule();
                });
    }

    public Command depotSideNeutralZoneToShootingSpotWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(
                            FieldConstants.PoseConstants.BLUE_DEPOT_BORDER_FUEL_EDGE.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.BLUE_DEPOT_BORDER_FUEL_EDGE.pose(),
                                    FieldConstants.PoseConstants.BLUE_DEPOT_WALL_FUEL_EDGE.pose(),
                                    FieldConstants.PoseConstants.BLUE_DEPOT_STARTING_LINE.pose(),
                                    FieldConstants.PoseConstants.DEPOT_SHOOTING_SPOT.pose());

                    PathPlannerPath depotSideNeutralZoneToShootingSpot =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    depotSideNeutralZoneToShootingSpot.preventFlipping = true;

                    AutoBuilder.followPath(depotSideNeutralZoneToShootingSpot).schedule();
                });
    }

    public Command outpostSideNeutralZoneToShootingSpotWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(
                            FieldConstants.PoseConstants.BLUE_OUTPOST_BORDER_FUEL_EDGE.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_BORDER_FUEL_EDGE
                                            .pose(),
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_WALL_FUEL_EDGE.pose(),
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_STARTING_LINE.pose(),
                                    FieldConstants.PoseConstants.OUTPOST_SHOOTING_SPOT.pose());

                    PathPlannerPath outpostSideNeutralZoneToShootingSpot =
                            new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    outpostSideNeutralZoneToShootingSpot.preventFlipping = true;

                    AutoBuilder.followPath(outpostSideNeutralZoneToShootingSpot).schedule();
                });
    }

    public Command depotSideNeutralZoneIntakingWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(
                            FieldConstants.PoseConstants.BLUE_DEPOT_WALL_FUEL_CENTER.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.BLUE_DEPOT_WALL_FUEL_CENTER.pose(),
                                    FieldConstants.PoseConstants.BLUE_DEPOT_BORDER_FUEL_CENTER
                                            .pose(),
                                    FieldConstants.PoseConstants.DEPOT_NEUTRAL_ZONE_CENTER.pose(),
                                    FieldConstants.PoseConstants.DEPOT_IN_NEUTRAL_ZONE.pose(),
                                    FieldConstants.PoseConstants.BLUE_DEPOT_BORDER_FUEL_EDGE
                                            .pose());

                    PathPlannerPath depotSideNeutralZoneIntaking =
                            new PathPlannerPath(
                                    waypoints,
                                    intakeConstraints,
                                    null,
                                    new GoalEndState(1.5, Rotation2d.fromDegrees(180)));

                    depotSideNeutralZoneIntaking.preventFlipping = true;

                    AutoBuilder.followPath(depotSideNeutralZoneIntaking).schedule();
                });
    }

    public Command outpostSideNeutralZoneIntakingWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(
                            FieldConstants.PoseConstants.BLUE_OUTPOST_WALL_FUEL_CENTER.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_WALL_FUEL_CENTER
                                            .pose(),
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_BORDER_FUEL_CENTER
                                            .pose(),
                                    FieldConstants.PoseConstants.OUTPOST_NEUTRAL_ZONE_CENTER.pose(),
                                    FieldConstants.PoseConstants.OUTPOST_IN_NEUTRAL_ZONE.pose(),
                                    FieldConstants.PoseConstants.BLUE_OUTPOST_BORDER_FUEL_EDGE
                                            .pose());

                    PathPlannerPath outpostSideNeutralZoneIntaking =
                            new PathPlannerPath(
                                    waypoints,
                                    intakeConstraints,
                                    null,
                                    new GoalEndState(1.5, Rotation2d.fromDegrees(180)));

                    outpostSideNeutralZoneIntaking.preventFlipping = true;

                    AutoBuilder.followPath(outpostSideNeutralZoneIntaking).schedule();
                });
    }
}
