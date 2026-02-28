package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.RebuiltVisionUtil;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;
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
    private final IntakeArmSubsystem intakeArmSubsystem;

    public CommandFactory(
            DriveSubsystem drive,
            Gyro gyro,
            ShooterSubsystem shooter,
            ShooterIndexerSubsystem shooterIndexer,
            SpindexerSubsystem spindexer,
            IntakeSubsystem intakeSubsystem,
            IntakeArmSubsystem intakeArmSubsystem) {
        this.drive = drive;
        this.gyro = gyro;
        this.shooter = shooter;
        this.shooterIndexer = shooterIndexer;
        this.spindexer = spindexer;
        this.intakeSubsystem = intakeSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
    }

    public Command runIntakeandIntakeArm() {
        return Commands.parallel(intakeSubsystem.runIntake(), intakeArmSubsystem.intakeArmDeploy());
    }

    public Command runSpindexShooterIndexAndShooter() {
        return Commands.sequence(
                shooter.shoot(RebuiltVisionUtil.getDistanceToHub(() -> drive.getRobotPose()))
                        .withTimeout(1.0),
                Commands.parallel(spindexer.runSpindexer(), shooterIndexer.runShooterIndexer()));
    }
}
