package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;
    private final ShooterSubsystem shooter;
    private final ShooterIndexerSubsystem shooterIndexer;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intakeSubsystem;

    public enum TargetFuel {
        HUB,
        DEPOT_SIDE,
        OUTPOST_SIDE
    }

    public CommandFactory(
            DriveSubsystem drive,
            Gyro gyro,
            ShooterSubsystem shooter,
            ShooterIndexerSubsystem shooterIndexer,
            SpindexerSubsystem spindexer,
            IntakeSubsystem intakeSubsystem) {
        this.drive = drive;
        this.gyro = gyro;
        this.shooter = shooter;
        this.shooterIndexer = shooterIndexer;
        this.spindexer = spindexer;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command runIntakeandIntakeArm() {
        return Commands.parallel(intakeSubsystem.deployAndRunIntake());
    }

    public Command runSpindexShooterIndexAndShooter() {
        return Commands.sequence(
                shooter.shoot(() -> RebuiltVisionUtil.getDistanceToHub()).withTimeout(1.0),
                Commands.parallel(spindexer.runSpindexer(), shooterIndexer.runShooterIndexer()));
    }

    public Command resetHeading(Angle yaw) {
        return Commands.parallel(
                gyro.setYawCommand(yaw), Commands.runOnce(() -> drive.resetOdometryAfterGyro()));
    }
}
