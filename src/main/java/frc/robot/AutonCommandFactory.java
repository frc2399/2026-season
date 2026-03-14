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
            new PathConstraints(1, 5, Units.degreesToRadians(720), Units.degreesToRadians(720));

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

        return new DeferredCommand(
                () -> {
                    finalPose = pose.pose();
                    System.out.println("building deferred + going to " + pose.name());
                    System.out.println(pose.pose().getX());
                    System.out.println(pose.pose().getY());
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
                intake.stowArm(),
                Commands.runOnce(() -> drive.resetOdometryFlipped(BLUE_DEPOT_STARTING_LINE.pose())),
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
                        BLUE_DEPOT_WALL_FUEL_CENTER.pose()));
    }

    public Command depotSideNeutralZoneIntaking() {
        return Commands.sequence(
                intake.stowArm(),
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
                        intake.deployAndRunIntake().withTimeout(6),
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
                intake.defaultBehavior().withTimeout(0.01),
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
                intake.stowArm(),
                Commands.print("arm stowed"),
                Commands.runOnce(
                        () -> drive.resetOdometryFlipped(BLUE_OUTPOST_STARTING_LINE.pose())),
                buildPathDeferred(BLUE_OUTPOST_BORDER_FUEL_CENTER, constraints, 0),
                // drive.driveToPoseOnExecute(),
                Commands.parallel(
                        Commands.print("going along to pick up fuel"),
                        intake.runRoller().withTimeout(3),
                        buildPathDeferred(OUTPOST_NEUTRAL_ZONE_CENTER, constraints, 0)),
                Commands.parallel(
                        intake.runRoller().withTimeout(3),
                        buildPathDeferred(BLUE_OUTPOST_BORDER_FUEL_CENTER, constraints, 0)
                ),
                // intake.defaultBehavior().withTimeout(0.01),
                Commands.print("defaulted intake"),
                buildPathDeferred(BLUE_OUTPOST_STARTING_LINE, constraints, 0),
                Commands.print("starting line!"),
                buildPathDeferred(OUTPOST_SHOOTING_SPOT, constraints, 0),
                Commands.print("gone back!"),
                commandFactory.runSpindexShooterIndexAndShooter().withTimeout(5),
                Commands.print("final shooting done"));
    }

    public Command driveStraightTesting() {
        return Commands.sequence(
                intake.stowArm(),
                Commands.runOnce(
                        () ->
                                drive.resetOdometryFlipped(
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
}
