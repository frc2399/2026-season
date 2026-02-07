package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeArmSubsystem intakeArmSubsystem;

    public CommandFactory(
            DriveSubsystem drive,
            Gyro gyro,
            IntakeSubsystem intakeSubsystem,
            IntakeArmSubsystem intakeArmSubsystem) {
        this.drive = drive;
        this.gyro = gyro;
        this.intakeSubsystem = intakeSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
    }

    public Command runIntakeandIntakeArm() {
        return Commands.parallel(intakeSubsystem.runIntake(), intakeArmSubsystem.deploy());
    }
}
