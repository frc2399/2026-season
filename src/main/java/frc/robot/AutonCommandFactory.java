package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.constants.FieldConstants.PoseConstants.*;
import static frc.robot.constants.FieldConstants.PoseConstants.SecondPassScoopPoseConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandFactory.TargetZoneType;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.RebuiltVisionUtil;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerSubsystem;
import java.util.List;
import java.util.Set;

public class AutonCommandFactory {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final CommandFactory commandFactory;
    private final Gyro gyro;
    private final ShooterSubsystem shooter;
    private final ShooterIndexerSubsystem shooterIndexer;
    private Pose2d finalPose;

    private Debouncer hasStoppedShootingDebouncer = new Debouncer(0.5);

    private Trigger hasStoppedShootingTrigger;

    public final PathConstraints constraints =
            new PathConstraints(3, 4, Units.degreesToRadians(720), Units.degreesToRadians(720));

    public final PathConstraints neutralConstraints =
            new PathConstraints(3, 4, Units.degreesToRadians(720), Units.degreesToRadians(720));

    public final PathConstraints depotConstraints =
            new PathConstraints(
                    0.75, 4, Units.degreesToRadians(720), Units.degreesToRadians((720)));

    public AutonCommandFactory(
            DriveSubsystem drive,
            IntakeSubsystem intake,
            CommandFactory commandFactory,
            Gyro gyro,
            ShooterSubsystem shooter,
            ShooterIndexerSubsystem shooterIndexer) {
        this.drive = drive;
        this.intake = intake;
        this.commandFactory = commandFactory;
        this.gyro = gyro;
        this.shooter = shooter;
        this.shooterIndexer = shooterIndexer;

        hasStoppedShootingTrigger =
                new Trigger(() -> shooter.isUpToSpeed() && shooterIndexer.isMoving());
    }

    public boolean hasStoppedShooting() {
        return hasStoppedShootingDebouncer.calculate(hasStoppedShootingTrigger.getAsBoolean());
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

    public Pose2d getTargetPoseIfNeedFlippedEarlyShooter(Pose pose) {
        Pose2d finalPose = pose.pose();
        if (FieldConstants.alliance.isPresent()
                && FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
            finalPose = FlippingUtil.flipFieldPose(finalPose);
        }
        return finalPose;
    }

    public Command outpostSideNeutralZoneAndBackWithShooting() {
        return Commands.sequence(
                        // first cycle
                        intake.stowArmSetpoint(),
                        Commands.runOnce(() -> resetOdometryFlipped(OUTPOST_STARTING_POSE.pose())),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                buildPathDeferred(BLUE_OUTPOST_BORDER_FUEL_CENTER, constraints, 2),
                                intake.deployAndRunIntake().withTimeout(0.01)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        OUTPOST_NEUTRAL_ZONE_CENTER, neutralConstraints, 0)),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                buildPathDeferred(OUTPOST_SHOOTING_SPOT, constraints, 0),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        OUTPOST_SHOOTING_SPOT)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()),
                        // second cycle
                        intake.defaultBehavior().withTimeout(0.01),
                        commandFactory.defaultSpindexerShooterIndexerAndShooter().withTimeout(0.01),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                intake.deployArm().withTimeout(0.1),
                                buildPathDeferred(OUTPOST_PASS_2_START, constraints, 2)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(OUTPOST_PASS_2_END, neutralConstraints, 0)))
                .withName("outpost side to neutral zone and then back and shoot");
    }

    public Command outpostSideNeutralZoneScoop() {
        return Commands.sequence(
                        // first cycle
                        intake.stowArmSetpoint(),
                        Commands.runOnce(() -> resetOdometryFlipped(OUTPOST_STARTING_POSE.pose())),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                buildPathDeferred(BLUE_OUTPOST_BORDER_FUEL_CENTER, constraints, 2),
                                intake.deployAndRunIntake().withTimeout(0.01)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        OUTPOST_NEUTRAL_ZONE_CENTER, neutralConstraints, 1)),
                        buildPathDeferred(OUTPOST_MIDDLE_HUB_NEUTRAL_ZONE, neutralConstraints, 1),
                        buildPathDeferred(OUTPOST_CORNER_HUB_NEUTRAL_ZONE, neutralConstraints, 1),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, neutralConstraints, .5),
                        intake.defaultBehavior().withTimeout(0.01),
                        Commands.parallel(
                                buildPathDeferred(OUTPOST_SHOOTING_SPOT, constraints, 0),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        OUTPOST_SHOOTING_SPOT)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()),
                        // second cycle
                        intake.defaultBehavior().withTimeout(0.01),
                        commandFactory.defaultSpindexerShooterIndexerAndShooter().withTimeout(0.01),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        OUTPOST_PAST_TRENCH_ROTATION_SCOOP_2, constraints, 0.5)),
                        buildPathDeferred(OUTPOST_CORNER_HUB_SCOOP_2, neutralConstraints, 1),
                        buildPathDeferred(OUTPOST_MIDDLE_HUB_SCOOP_2, neutralConstraints, 1),
                        buildPathDeferred(
                                OUTPOST_NEUTRAL_ZONE_CENTER_SCOOP_2, neutralConstraints, 1),
                        buildPathDeferred(OUTPOST_OTHER_SIDE_OF_TRENCH, constraints, 1),
                        Commands.parallel(
                                buildPathDeferred(OUTPOST_SHOOTING_SPOT, constraints, 0),
                                intake.defaultBehavior().withTimeout(0.1),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        OUTPOST_SHOOTING_SPOT)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()))
                .withName("outpost side to neutral zone with scoop");
    }

    public Command depotSideNeutralZoneScoop() {
        return Commands.sequence(
                        // first cycle
                        intake.stowArmSetpoint(),
                        Commands.runOnce(() -> resetOdometryFlipped(DEPOT_STARTING_POSE.pose())),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_BORDER_FUEL_CENTER, constraints, 2),
                                intake.deployAndRunIntake().withTimeout(0.01)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        DEPOT_NEUTRAL_ZONE_CENTER, neutralConstraints, 1)),
                        buildPathDeferred(DEPOT_MIDDLE_HUB_NEUTRAL_ZONE, neutralConstraints, 1),
                        buildPathDeferred(DEPOT_CORNER_HUB_NEUTRAL_ZONE, constraints, 1),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, .5),
                        intake.defaultBehavior().withTimeout(0.01),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_SHOOTING_SPOT, constraints, 0),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        DEPOT_SHOOTING_SPOT)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()),
                        // second cycle
                        intake.defaultBehavior().withTimeout(0.01),
                        commandFactory.defaultSpindexerShooterIndexerAndShooter().withTimeout(0.01),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        DEPOT_PAST_TRENCH_ROTATION_SCOOP_2, constraints, 0.5)),
                        buildPathDeferred(DEPOT_CORNER_HUB_SCOOP_2, neutralConstraints, 1),
                        buildPathDeferred(DEPOT_MIDDLE_HUB_SCOOP_2, neutralConstraints, 1),
                        buildPathDeferred(DEPOT_NEUTRAL_ZONE_CENTER_SCOOP_2, neutralConstraints, 1),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 1),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_SHOOTING_SPOT, constraints, 0),
                                intake.defaultBehavior().withTimeout(0.1),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        DEPOT_SHOOTING_SPOT)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()))
                .withName("depot side to neutral zone with scoop");
    }

    public Command depotSideNeutralZoneAndBackWithShooting() {
        return Commands.sequence(
                        // first cycle
                        intake.stowArmSetpoint(),
                        Commands.runOnce(() -> resetOdometryFlipped(DEPOT_STARTING_POSE.pose())),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_BORDER_FUEL_CENTER, constraints, 2),
                                intake.deployAndRunIntake().withTimeout(0.01)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(
                                        DEPOT_NEUTRAL_ZONE_CENTER, neutralConstraints, 0)),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 1),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_SHOOTING_SPOT, constraints, 0),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        DEPOT_SHOOTING_SPOT)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()),
                        // second cycle
                        intake.defaultBehavior().withTimeout(0.01),
                        commandFactory.defaultSpindexerShooterIndexerAndShooter().withTimeout(0.01),
                        Commands.waitUntil(() -> intake.isArmBelowTrench()),
                        buildPathDeferred(DEPOT_OTHER_SIDE_OF_TRENCH, constraints, 2),
                        Commands.parallel(
                                intake.deployArm().withTimeout(0.1),
                                buildPathDeferred(DEPOT_PASS_2_START, constraints, 2)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(DEPOT_PASS_2_END, neutralConstraints, 0)))
                .withName("depot side to neutral zone then back and shoot");
    }

    public Command centerMoveAndShoot() {
        return Commands.sequence(
                        Commands.runOnce(
                                () -> resetOdometryFlipped(FieldConstants.FRONT_OF_BLUE_HUB)),
                        intake.stowArmSetpoint(),
                        Commands.parallel(
                                buildPathDeferred(MIDDLE_SHOOT_POSE, constraints, 0),
                                shooter.shoot(
                                                () ->
                                                        RebuiltVisionUtil
                                                                .getDistanceToAlignmentTarget(
                                                                        () ->
                                                                                getTargetPoseIfNeedFlippedEarlyShooter(
                                                                                        MIDDLE_SHOOT_POSE)),
                                                false,
                                                () -> TargetZoneType.HUB)
                                        .withTimeout(0.1)),
                        commandFactory
                                .runSpindexShooterIndexAndShooter(false)
                                .until(() -> hasStoppedShooting()))
                .withName("move from center and shoot preload");
    }

    //         public Command hubMoveToDepot() {
    //             return Commands.sequence(
    //                             Commands.runOnce(
    //                                     () ->
    //     resetOdometryFlipped(FieldConstants.FRONT_OF_BLUE_HUB)),
    //                             intake.stowArmSetpoint(),
    //                             buildPathDeferred(DEPOT_EDGE, constraints, 0),
    //                             Commands.parallel(
    //                             intake.deployArm().withTimeout(0.1),
    //                             buildPathDeferred(DEPOT_SIDE_INTAKE, constraints, 0)),
    //                             Commands.parallel(
    //                             intake.deployAndRunIntake().withTimeout(0.1),
    //                             buildPathDeferred(DEPOT_SIDE_INTAKE_END, intakeConstraints, 0)),
    //                             //buildPathDeferred(CENTER_MOVE_TO_SHOOTING_SPOT, constraints,
    // 0),
    //                             buildPathDeferred(CENTER_SHOOTING_SPOT, constraints, 0),
    //
    // CommandFactory.runSpindexShooterIndexAndShooterNoFeedFuel().withTimeout(2))
    //                     .withName("move from center to depot and shoot");
    //         }

    public Command hubMoveToDepot() {
        return Commands.sequence(
                        Commands.runOnce(
                                () -> resetOdometryFlipped(FieldConstants.FRONT_OF_BLUE_HUB)),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_EDGE, constraints, 0),
                                intake.stowArmSetpoint()),
                        Commands.parallel(
                                intake.deployArm().withTimeout(0.1),
                                buildPathDeferred(DEPOT_SIDE_INTAKE, constraints, 0)),
                        // Commands.parallel(
                        //         intake.deployHigherAndRunIntake().withTimeout(0.1),
                        //         buildPathDeferred(DEPOT_MIDDLE, depotConstraints, 1)),
                        // Commands.parallel(
                        //         intake.deployAndRunIntake().withTimeout(0.1),
                        //         buildPathDeferred(DEPOT_SIDE_INTAKE_END, depotConstraints, 0)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(DEPOT_SIDE_INTAKE_END, depotConstraints, 0)),
                        drive.driveToPoseOnExecute(
                                () -> new Pose2d(0.7, 7, new Rotation2d(Degrees.of(-52)))),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(DEPOT_CORNER, constraints, 0),
                        buildPathDeferred(DEPOT_WALL_INTAKE, constraints, 0),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(DEPOT_CORNER_WALL_INTAKE, depotConstraints, 0)),
                        // buildPathDeferred(DEPOT_CORNER_STRAIGHT, constraints, 0),
                        // Commands.parallel(
                        //         intake.deployAndRunIntake().withTimeout(0.1),
                        //         buildPathDeferred(
                        //                 DEPOT_CORNER_INTAKE_STRAIGHT, depotConstraints, 0)),
                        // buildPathDeferred(CENTER_MOVE_TO_SHOOTING_SPOT, constraints, 0),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(CENTER_SHOOTING_SPOT, constraints, 0),
                        commandFactory.runSpindexShooterIndexAndShooter(false).withTimeout(7))
                .withName("move from center to depot and shoot");
    }

    public Command hubMoveToDepotNoWallScrape() {
        return Commands.sequence(
                        Commands.runOnce(
                                () -> resetOdometryFlipped(FieldConstants.FRONT_OF_BLUE_HUB)),
                        Commands.parallel(
                                buildPathDeferred(DEPOT_EDGE, constraints, 0),
                                intake.stowArmSetpoint()),
                        Commands.parallel(
                                intake.deployArm().withTimeout(0.1),
                                buildPathDeferred(DEPOT_SIDE_INTAKE, constraints, 0)),
                        // Commands.parallel(
                        //         intake.deployHigherAndRunIntake().withTimeout(0.1),
                        //         buildPathDeferred(DEPOT_MIDDLE, depotConstraints, 1)),
                        // Commands.parallel(
                        //         intake.deployAndRunIntake().withTimeout(0.1),
                        //         buildPathDeferred(DEPOT_SIDE_INTAKE_END, depotConstraints, 0)),
                        Commands.parallel(
                                intake.deployAndRunIntake().withTimeout(0.1),
                                buildPathDeferred(DEPOT_SIDE_INTAKE_END, depotConstraints, 0)),
                        drive.driveToPoseOnExecute(
                                () -> new Pose2d(0.7, 7, new Rotation2d(Degrees.of(-52)))),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(DEPOT_CORNER, constraints, 0),
                        intake.defaultBehavior().withTimeout(0.01),
                        buildPathDeferred(CENTER_SHOOTING_SPOT, constraints, 0),
                        commandFactory.runSpindexShooterIndexAndShooter(false).withTimeout(7))
                .withName("move from center to depot and shoot no wall scrape");
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
