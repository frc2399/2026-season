package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Pose;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerSubsystem;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;
    private final ShooterSubsystem shooter;
    private final ShooterIndexerSubsystem shooterIndexer;

    private final PathConstraints constraints =
            new PathConstraints(0.5, 5, Units.degreesToRadians(360), Units.degreesToRadians((540)));

    public CommandFactory(
            DriveSubsystem drive,
            Gyro gyro,
            ShooterSubsystem shooter,
            ShooterIndexerSubsystem shooterIndexer) {
        this.drive = drive;
        this.gyro = gyro;
        this.shooter = shooter;
        this.shooterIndexer = shooterIndexer;
    }

    public Command runShooterAndShooterIndexer() {
        return Commands.parallel(shooter.shoot(), shooterIndexer.runShooterIndexer());
    }

    public Command buildPath(Pose pose) {
        return AutoBuilder.pathfindToPose(pose.pose(), constraints, 0).withName(pose.name());
    }
}
