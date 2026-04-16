package frc.robot.subsystems.shooterIndexer;

public interface ShooterIndexerIO {
    public void runShooterIndexer();

    public void backwardsRunShooterIndexer();

    public void defaultBehavior();

    public void updateStates(ShooterIndexerIOState state);

    public boolean isUpToSpeed();

    public static class ShooterIndexerIOState {
        public double shooterIndexerDesiredSpeedRad_P_S = 0;
        public double shooterIndexerActualSpeedRad_P_S = 0;
        public double shooterIndexerAppliedVoltage = 0;
        public double shooterIndexerCurrent = 0;
        public boolean isUpToSpeed = false;
    }
}
