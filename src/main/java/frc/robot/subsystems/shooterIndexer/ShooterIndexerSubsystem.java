package frc.robot.subsystems.shooterIndexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterIndexerSubsystem extends SubsystemBase {
    private ShooterIndexerIO io;

    public ShooterIndexerSubsystem(ShooterIndexerIO io) {
        this.io = io;
    }

    public Command runShooterIndexer() {
        return this.run(() -> io.runShooterIndexer()).withName("runShooterIndexer");
    }

    public Command backwardsShooterIndexer() {
        return this.run(() -> io.backwardsRunShooterIndexer())
                .withName("backwardsRunShooterIndexer");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterIndexerDefaultBehavior");
    }
}
