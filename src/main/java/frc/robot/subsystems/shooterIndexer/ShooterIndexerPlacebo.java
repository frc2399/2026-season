package frc.robot.subsystems.shooterIndexer;

public class ShooterIndexerPlacebo implements ShooterIndexerIO {
    public void runShooterIndexer() {}

    public void backwardsRunShooterIndexer() {}

    public void defaultBehavior() {}

    @Override
    public void updateStates(ShooterIndexerIOState state) {}

    @Override
    public boolean isUpToSpeed() {
        return true;
    }
}
