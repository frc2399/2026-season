package frc.robot.subsystems.shooterIndexer;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIndexerPlacebo implements ShooterIndexerIO {

    public void runShooterIndexerAtSpeed(AngularVelocity speed) {}

    public void runShooterIndexer() {}

    public void backwardsRunShooterIndexer() {}

    public void defaultBehavior() {}

    @Override
    public void updateStates(ShooterIndexerIOState state) {}

    @Override
    public boolean isMoving() {
        return true;
    }
}
