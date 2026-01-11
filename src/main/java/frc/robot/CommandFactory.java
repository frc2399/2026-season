package frc.robot;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gyro.Gyro;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;

    public CommandFactory(DriveSubsystem drive, Gyro gyro) {
        this.drive = drive;
        this.gyro = gyro;
    }
}
