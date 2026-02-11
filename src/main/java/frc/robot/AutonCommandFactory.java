package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import java.util.Set;

public class AutonCommandFactory {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;

    public final PathConstraints constraints =
            new PathConstraints(0.8, 5, Units.degreesToRadians(720), Units.degreesToRadians((720)));

    public final PathConstraints bumpConstraints =
            new PathConstraints(0.4, 5, Units.degreesToRadians(360), Units.degreesToRadians(540));

    public AutonCommandFactory(DriveSubsystem drive, IntakeSubsystem intake) {
        this.drive = drive;
        this.intake = intake;
    }

    public Command buildPathDeferred(Pose pose, PathConstraints constraints) {
        return new DeferredCommand(
                () -> AutoBuilder.pathfindToPose(pose.pose(), constraints, 0).withName(pose.name()),
                Set.of(drive));
    }

    public Command buildPath(Pose pose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(pose.pose(), constraints, 0).withName(pose.name());
    }

    public Command hubToDepot() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> drive.resetOdometry(FieldConstants.PoseConstants.HUB_MIDDLE.pose())),
                buildPathDeferred(FieldConstants.PoseConstants.DEPOT_TESTING, constraints),
                // move into intake position while driving
                drive.driveToPoseOnExecute(),
                // intakeSubsystem.runIntake().withTimeout(3),
                // intakeSubsystem.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.HUB_MIDDLE, constraints),
                // move into shooting position while driving
                drive.driveToPoseOnExecute());
    }

    public Command bumpToNeutralZone() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
                buildPathDeferred(FieldConstants.PoseConstants.OVER_THE_BUMP, bumpConstraints),
                // move into intaking position while driving
                drive.driveToPoseOnExecute(),
                buildPathDeferred(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER, constraints),
                intake.runIntake().withTimeout(1),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.OVER_THE_BUMP, constraints),
                // get into shooting position while driving
                drive.driveToPoseOnExecute(),
                buildPathDeferred(FieldConstants.PoseConstants.BUMP_STARTING_LINE, bumpConstraints),
                drive.driveToPoseOnExecute());
    }

    public Command bumpNeutralZoneShooting() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BUMP_STARTING_LINE.pose())),
                buildPathDeferred(FieldConstants.PoseConstants.OVER_THE_BUMP, bumpConstraints),
                // move into intaking position while driving
                drive.driveToPoseOnExecute(),
                buildPathDeferred(FieldConstants.PoseConstants.NEUTRAL_ZONE_BORDER, constraints),
                intake.runIntake().withTimeout(1),
                intake.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.OVER_THE_BUMP, constraints),
                // get into shooting position while driving
                drive.driveToPoseOnExecute(),
                buildPathDeferred(FieldConstants.PoseConstants.HUB_MIDDLE, bumpConstraints),
                drive.driveToPoseOnExecute());
    }
}
