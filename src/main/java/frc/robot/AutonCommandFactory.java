package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

    public final PathConstraints constraints =
            new PathConstraints(2, 5, Units.degreesToRadians(720), Units.degreesToRadians(720));

    public final PathConstraints intakeConstraints =
            new PathConstraints(
                    0.75, 5, Units.degreesToRadians(720), Units.degreesToRadians((720)));

    public final PathConstraints bumpAndDepotConstraints =
            new PathConstraints(1.5, 5, Units.degreesToRadians(360), Units.degreesToRadians(540));

    public AutonCommandFactory(DriveSubsystem drive, IntakeSubsystem intake) {
        this.drive = drive;
        this.intake = intake;
    }

    public Command buildPathDeferred(
            Pose pose, PathConstraints constraints, double goalEndVelocity) {
        return new DeferredCommand(
                () ->
                        AutoBuilder.pathfindToPose(pose.pose(), constraints, goalEndVelocity)
                                .withName(pose.name()),
                Set.of(drive));
    }

    public Command hubToDepot() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose())),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT_TESTING, constraints, 0),
                // move into intake position while driving
                drive.driveToPoseOnExecute(),
                // intakeSubsystem.runIntake().withTimeout(3),
                // intakeSubsystem.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.HUB_MIDDLE, constraints, 0),
                // move into shooting position while driving
                drive.driveToPoseOnExecute());
    }

    public Command depotSideBumpToNeutralZone() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.DEPOT_SIDE_BUMP_STARTING_LINE
                                                .pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.DEPOT_SIDE_OVER_THE_BUMP,
                        bumpAndDepotConstraints,
                        0.75),
                // drive.driveToPoseOnExecute(),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        buildPathDeferred(
                                FieldConstants.PoseConstants.DEPOT_SIDE_IN_NEUTRAL_ZONE,
                                intakeConstraints,
                                0)),
                intake.defaultBehavior().withTimeout(0.01),
                // drive.driveToPoseOnExecute()),
                buildPathDeferred(
                        FieldConstants.PoseConstants.DEPOT_SIDE_NEUTRAL_ZONE_BORDER,
                        constraints,
                        1.5),
                // get into shooting position while driving
                // drive.driveToPoseOnExecute(),
                depotSideNeutralZoneToHubWaypoints());
        // drive.driveToPoseOnExecute());
    }

    public Command outpostSideBumpToNeutralZone() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.OUTPOST_SIDE_BUMP_STARTING_LINE
                                                .pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OUTPOST_SIDE_OVER_THE_BUMP,
                        bumpAndDepotConstraints,
                        0.75),
                // drive.driveToPoseOnExecute(),
                // buildPathDeferred(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER, constraints),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        buildPathDeferred(
                                FieldConstants.PoseConstants.OUTPOST_SIDE_IN_NEUTRAL_ZONE,
                                intakeConstraints,
                                0)),
                intake.defaultBehavior().withTimeout(0.01),
                // drive.driveToPoseOnExecute()),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OUTPOST_SIDE_NEUTRAL_ZONE_BORDER,
                        constraints,
                        1.5),
                // get into shooting position while driving
                // drive.driveToPoseOnExecute(),
                outpostSideNeutralZoneToHubWaypoints());
        // drive.driveToPoseOnExecute());
    }

    public Command depotSideBumpToNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.DEPOT_SIDE_BUMP_STARTING_LINE
                                                .pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.DEPOT_SIDE_OVER_THE_BUMP,
                        bumpAndDepotConstraints,
                        1.5),
                // move into intaking position while driving
                drive.driveToPoseOnExecute(),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        buildPathDeferred(
                                FieldConstants.PoseConstants.DEPOT_SIDE_IN_NEUTRAL_ZONE,
                                intakeConstraints,
                                0)),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(
                        FieldConstants.PoseConstants.DEPOT_SIDE_NEUTRAL_ZONE_BORDER,
                        constraints,
                        1.5),
                // get into shooting position while driving
                drive.driveToPoseOnExecute(),
                depotSideNeutralZoneToHubWaypoints());
        // shooting command
    }

    public Command outpostSideBumpToNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.OUTPOST_SIDE_BUMP_STARTING_LINE
                                                .pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OUTPOST_SIDE_OVER_THE_BUMP,
                        bumpAndDepotConstraints,
                        1.5),
                // move into intaking position while driving
                drive.driveToPoseOnExecute(),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        buildPathDeferred(
                                FieldConstants.PoseConstants.OUTPOST_SIDE_IN_NEUTRAL_ZONE,
                                intakeConstraints,
                                0)),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OUTPOST_SIDE_NEUTRAL_ZONE_BORDER,
                        constraints,
                        1.5),
                // get into shooting position while driving
                drive.driveToPoseOnExecute(),
                outpostSideNeutralZoneToHubWaypoints());
        // shooting command
    }

    public Command depotSideDepotToNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.DEPOT_SIDE_BUMP_STARTING_LINE
                                                .pose())),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT, constraints, 0),
                drive.driveToPoseOnExecute(),
                intake.runIntake().withTimeout(3),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.HUB_MIDDLE, constraints, 0),
                drive.driveToPoseOnExecute(),
                // shooting command
                depotSideHubToNeutralZoneWaypoints());
    }

    public Command outpostSideDepotToNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.DEPOT_SIDE_BUMP_STARTING_LINE
                                                .pose())),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT, constraints, 0),
                drive.driveToPoseOnExecute(),
                intake.runIntake().withTimeout(3),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.HUB_MIDDLE, constraints, 0),
                drive.driveToPoseOnExecute(),
                // shooting command
                outpostSideHubToNeutralZoneWaypoints());
    }

    public Command depotSideHubToNeutralZoneWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.HUB_MIDDLE.pose(),
                                    FieldConstants.PoseConstants.DEPOT_SIDE_BUMP_STARTING_LINE
                                            .pose(),
                                    FieldConstants.PoseConstants.DEPOT_SIDE_OVER_THE_BUMP.pose(),
                                    FieldConstants.PoseConstants.DEPOT_SIDE_NEUTRAL_ZONE_BORDER
                                            .pose());

                    PathPlannerPath depotSideHubToNeutralZone =
                            new PathPlannerPath(
                                    waypoints,
                                    bumpAndDepotConstraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    depotSideHubToNeutralZone.preventFlipping = true;

                    AutoBuilder.followPath(depotSideHubToNeutralZone).schedule();
                });
    }

    public Command outpostSideHubToNeutralZoneWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.HUB_MIDDLE.pose(),
                                    FieldConstants.PoseConstants.OUTPOST_SIDE_BUMP_STARTING_LINE
                                            .pose(),
                                    FieldConstants.PoseConstants.OUTPOST_SIDE_OVER_THE_BUMP.pose(),
                                    FieldConstants.PoseConstants.OUTPOST_SIDE_NEUTRAL_ZONE_BORDER
                                            .pose());

                    PathPlannerPath outpostSideHubToNeutralZone =
                            new PathPlannerPath(
                                    waypoints,
                                    bumpAndDepotConstraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    outpostSideHubToNeutralZone.preventFlipping = true;

                    AutoBuilder.followPath(outpostSideHubToNeutralZone).schedule();
                });
    }

    public Command depotSideNeutralZoneToHubWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(
                            FieldConstants.PoseConstants.DEPOT_SIDE_NEUTRAL_ZONE_BORDER.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.DEPOT_SIDE_NEUTRAL_ZONE_BORDER
                                            .pose(),
                                    FieldConstants.PoseConstants.DEPOT_SIDE_OVER_THE_BUMP.pose(),
                                    FieldConstants.PoseConstants.DEPOT_SIDE_BUMP_STARTING_LINE
                                            .pose(),
                                    FieldConstants.PoseConstants.HUB_MIDDLE.pose());

                    PathPlannerPath depotSideNeutralZoneToHub =
                            new PathPlannerPath(
                                    waypoints,
                                    bumpAndDepotConstraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    depotSideNeutralZoneToHub.preventFlipping = true;

                    AutoBuilder.followPath(depotSideNeutralZoneToHub).schedule();
                });
    }

    public Command outpostSideNeutralZoneToHubWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(
                            FieldConstants.PoseConstants.OUTPOST_SIDE_NEUTRAL_ZONE_BORDER.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.OUTPOST_SIDE_NEUTRAL_ZONE_BORDER
                                            .pose(),
                                    FieldConstants.PoseConstants.OUTPOST_SIDE_OVER_THE_BUMP.pose(),
                                    FieldConstants.PoseConstants.OUTPOST_SIDE_BUMP_STARTING_LINE
                                            .pose(),
                                    FieldConstants.PoseConstants.HUB_MIDDLE.pose());

                    PathPlannerPath outpostSideNeutralZoneToHub =
                            new PathPlannerPath(
                                    waypoints,
                                    bumpAndDepotConstraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    outpostSideNeutralZoneToHub.preventFlipping = true;

                    AutoBuilder.followPath(outpostSideNeutralZoneToHub).schedule();
                });
    }
}
