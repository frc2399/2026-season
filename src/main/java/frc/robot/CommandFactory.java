package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.AutoBuilder;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;

    private final PathConstraints constraints = new PathConstraints(0.5, 5, 
      Units.degreesToRadians(360), Units.degreesToRadians((540)));

    public CommandFactory(DriveSubsystem drive, Gyro gyro) {
        this.drive = drive;
        this.gyro = gyro;
    }

    public Command buildPath(Pose pose) {
        return AutoBuilder.pathfindToPose(pose.pose(), constraints, 0).withName(pose.name());
    }
}
