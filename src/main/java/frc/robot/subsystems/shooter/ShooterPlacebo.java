package frc.robot.subsystems.shooter;

public class ShooterPlacebo implements ShooterIO {
    public void runShooter() {}

    public void defaultBehavior() {}

    public boolean isUpToSpeed() {
        return true;
    }

    @Override
    public void updateStates(ShooterIOState state) {}
}
