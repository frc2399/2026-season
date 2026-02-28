package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
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
    private Pose2d finalPose;

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

    public Command hubToDepot() {
        return Commands.sequence(
                Commands.runOnce(
                        () ->
                                drive.resetOdometry(
                                        FieldConstants.PoseConstants.BLUE_HUB_MIDDLE.pose())),
                buildPathDeferred(FieldConstants.PoseConstants.IN_THE_DEPOT, constraints,0),
                // move into intake position while driving
                drive.driveToPoseOnExecute(),
                // intakeSubsystem.runIntake().withTimeout(3),
                // intakeSubsystem.defaultBehavior().withTimeout(0.01),
                buildPathDeferred(FieldConstants.PoseConstants.BLUE_HUB_MIDDLE, constraints,0),
                // move into shooting position while driving
                drive.driveToPoseOnExecute());
    }
}
