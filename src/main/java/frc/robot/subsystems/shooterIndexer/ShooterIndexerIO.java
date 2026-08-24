package frc.robot.subsystems.shooterIndexer;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIndexerIO {

    public void runShooterIndexerAtSpeed(AngularVelocity speed);

    public void runShooterIndexer();

    public void backwardsRunShooterIndexer();

    public void defaultBehavior();

    public void updateStates(ShooterIndexerIOState state);

    public boolean isMoving();

    public static class ShooterIndexerIOState {
        public double shooterIndexerDesiredSpeedRad_P_S = 0;
        public double shooterIndexerActualSpeedRad_P_S = 0;
        public double shooterIndexerAppliedVoltage = 0;
        public double shooterIndexerCurrent = 0;
    }
}
