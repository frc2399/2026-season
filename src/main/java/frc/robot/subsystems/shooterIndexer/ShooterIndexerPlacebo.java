package frc.robot.subsystems.shooterIndexer;

public class ShooterIndexerPlacebo implements ShooterIndexerIO {
    public void runShooterIndexer() {}

    public void backwardsRunShooterIndexer() {}

    public void defaultBehavior() {}

    @Override
    public boolean isUpToSpeed() {
        return true;
    }

    @Override
    public void updateStates(ShooterIndexerIOState state) {}
}
