package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;

    public final PathConstraints constraints =
            new PathConstraints(0.4, 5, Units.degreesToRadians(720), Units.degreesToRadians((720)));

    public final PathConstraints bumpConstraints =
            new PathConstraints(0.2, 5, Units.degreesToRadians(360), Units.degreesToRadians(540));

    public CommandFactory(DriveSubsystem drive, Gyro gyro) {
        this.drive = drive;
        this.gyro = gyro;
    }

    public Command buildPath(Pose pose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(pose.pose(), constraints, 0).withName(pose.name());
    }
}
