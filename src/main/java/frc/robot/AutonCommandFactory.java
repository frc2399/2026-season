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

    public Command bumpToNeutralZone() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OVER_THE_BUMP, bumpAndDepotConstraints, 0.75),
                // drive.driveToPoseOnExecute(),
                // buildPathDeferred(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER, constraints),
                Commands.parallel(
                        intake.runIntake().withTimeout(6),
                        buildPathDeferred(
                                FieldConstants.PoseConstants.IN_NEUTRAL_ZONE,
                                intakeConstraints,
                                0)),
                intake.defaultBehavior().withTimeout(0.01),
                // drive.driveToPoseOnExecute()),
                buildPathDeferred(FieldConstants.PoseConstants.OVER_THE_BUMP, constraints, 1.5),
                // get into shooting position while driving
                // drive.driveToPoseOnExecute(),
                buildPathDeferred(
                        FieldConstants.PoseConstants.BUMP_STARTING_LINE,
                        bumpAndDepotConstraints,
                        0));
        // drive.driveToPoseOnExecute());
    }

    public Command bumpToNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
                buildPathDeferred(
                        FieldConstants.PoseConstants.OVER_THE_BUMP, bumpAndDepotConstraints, 1.5),
                // move into intaking position while driving
                drive.driveToPoseOnExecute(),
                buildPathDeferred(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER, constraints, 0),
                intake.runIntake().withTimeout(1),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.OVER_THE_BUMP, constraints, 1.5),
                // get into shooting position while driving
                drive.driveToPoseOnExecute(),
                buildPathDeferred(
                        FieldConstants.PoseConstants.HUB_MIDDLE, bumpAndDepotConstraints, 0),
                drive.driveToPoseOnExecute());
    }

    public Command depotToNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT, constraints, 0),
                drive.driveToPoseOnExecute(),
                intake.runIntake().withTimeout(3),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.HUB_MIDDLE, constraints, 0),
                drive.driveToPoseOnExecute(),
                // shooting command
                hubToNeutralZoneWaypoints());
    }

    public Command hubToNeutralZoneWaypoints() {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose());

                    List<Waypoint> waypoints =
                            PathPlannerPath.waypointsFromPoses(
                                    FieldConstants.PoseConstants.HUB_MIDDLE.pose(),
                                    FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose(),
                                    FieldConstants.PoseConstants.OVER_THE_BUMP.pose(),
                                    FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER.pose());

                    PathPlannerPath hubToNeutralZone =
                            new PathPlannerPath(
                                    waypoints,
                                    bumpAndDepotConstraints,
                                    null,
                                    new GoalEndState(0, Rotation2d.fromDegrees(180)));

                    hubToNeutralZone.preventFlipping = true;

                    AutoBuilder.followPath(hubToNeutralZone).schedule();
                });
    }
}
