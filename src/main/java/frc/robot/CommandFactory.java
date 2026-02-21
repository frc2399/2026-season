package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerSubsystem;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;
    private final ShooterSubsystem shooter;
    private final ShooterIndexerSubsystem shooterIndexer;
    private final IntakeSubsystem intakeSubsystem;

    public CommandFactory(
            DriveSubsystem drive,
            Gyro gyro,
            ShooterSubsystem shooter,
            ShooterIndexerSubsystem shooterIndexer,
            IntakeSubsystem intakeSubsystem) {
        this.drive = drive;
        this.gyro = gyro;
        this.shooter = shooter;
        this.shooterIndexer = shooterIndexer;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command runIntakeandIntakeArm() {
        return Commands.parallel(intakeSubsystem.deployAndRunIntake());
    }

    public Command runShooterAndShooterIndexer() {
        return Commands.parallel(shooter.shoot(), shooterIndexer.runShooterIndexer());
    }
}
