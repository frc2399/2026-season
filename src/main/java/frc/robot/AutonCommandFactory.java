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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
                Commands.runOnce(() -> drive.resetOdometry(BLUE_DEPOT_STARTING_LINE.pose())),
                buildPathDeferred(IN_THE_DEPOT, constraints, 0),
                // drive.driveToPoseOnExecute(),
                commandFactory.runIntakeandIntakeArm().withTimeout(3),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(DEPOT_SHOOTING_SPOT, constraints, 0),
                // drive.driveToPoseOnExecute(),
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
                        BLUE_DEPOT_WALL_FUEL_CENTER.pose()));
    }

    public Command depotSideNeutralZoneIntaking() {
        return Commands.sequence(
                Commands.runOnce(() -> drive.resetOdometry(BLUE_DEPOT_STARTING_LINE.pose())),
                buildPathDeferred(DEPOT_SHOOTING_SPOT, constraints, 0),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(2),
                followWaypoints(
                        DEPOT_SHOOTING_SPOT.pose(),
                        constraints,
                        1.5,
                        Rotation2d.fromDegrees(180),
                        DEPOT_SHOOTING_SPOT.pose(),
                        BLUE_DEPOT_STARTING_LINE.pose(),
                        BLUE_DEPOT_WALL_FUEL_CENTER.pose()),
                // drive.driveToPoseOnExecute(),
                Commands.parallel(
                        commandFactory.runIntakeandIntakeArm().withTimeout(6),
                        intake.defaultBehavior().withTimeout(0.01),
                        followWaypoints(
                                BLUE_DEPOT_WALL_FUEL_CENTER.pose(),
                                intakeConstraints,
                                1.5,
                                Rotation2d.fromDegrees(180),
                                BLUE_DEPOT_WALL_FUEL_CENTER.pose(),
                                BLUE_DEPOT_BORDER_FUEL_CENTER.pose(),
                                DEPOT_NEUTRAL_ZONE_CENTER.pose(),
                                DEPOT_IN_NEUTRAL_ZONE.pose(),
                                BLUE_DEPOT_BORDER_FUEL_EDGE.pose())),
                followWaypoints(
                        BLUE_DEPOT_BORDER_FUEL_EDGE.pose(),
                        constraints,
                        0,
                        Rotation2d.fromDegrees(180),
                        BLUE_DEPOT_BORDER_FUEL_EDGE.pose(),
                        BLUE_DEPOT_WALL_FUEL_EDGE.pose(),
                        BLUE_DEPOT_STARTING_LINE.pose(),
                        DEPOT_SHOOTING_SPOT.pose()),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(5));
    }

    public Command outpostSideNeutralZoneIntaking() {
        return Commands.sequence(
                Commands.runOnce(() -> drive.resetOdometry(BLUE_OUTPOST_STARTING_LINE.pose())),
                buildPathDeferred(OUTPOST_SHOOTING_SPOT, constraints, 0),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(2),
                followWaypoints(
                        OUTPOST_SHOOTING_SPOT.pose(),
                        constraints,
                        1.5,
                        Rotation2d.fromDegrees(180),
                        OUTPOST_SHOOTING_SPOT.pose(),
                        BLUE_OUTPOST_STARTING_LINE.pose(),
                        BLUE_OUTPOST_WALL_FUEL_CENTER.pose()),
                // drive.driveToPoseOnExecute(),
                Commands.parallel(
                        commandFactory.runIntakeandIntakeArm().withTimeout(6),
                        intake.defaultBehavior().withTimeout(0.01),
                        followWaypoints(
                                BLUE_OUTPOST_WALL_FUEL_CENTER.pose(),
                                intakeConstraints,
                                1.5,
                                Rotation2d.fromDegrees(180),
                                BLUE_OUTPOST_WALL_FUEL_CENTER.pose(),
                                BLUE_OUTPOST_BORDER_FUEL_CENTER.pose(),
                                OUTPOST_NEUTRAL_ZONE_CENTER.pose(),
                                OUTPOST_IN_NEUTRAL_ZONE.pose(),
                                BLUE_OUTPOST_BORDER_FUEL_EDGE.pose())),
                followWaypoints(
                        BLUE_OUTPOST_BORDER_FUEL_EDGE.pose(),
                        constraints,
                        0,
                        Rotation2d.fromDegrees(180),
                        BLUE_OUTPOST_BORDER_FUEL_EDGE.pose(),
                        BLUE_OUTPOST_WALL_FUEL_EDGE.pose(),
                        BLUE_OUTPOST_STARTING_LINE.pose(),
                        OUTPOST_SHOOTING_SPOT.pose()),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(5));
    }

    public Command followWaypoints(
            Pose2d startingPosition,
            PathConstraints constraints,
            double endVelocity,
            Rotation2d endRotation,
            Pose2d... poses) {
        return Commands.runOnce(
                () -> {
                    drive.resetOdometry(startingPosition);

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
}
