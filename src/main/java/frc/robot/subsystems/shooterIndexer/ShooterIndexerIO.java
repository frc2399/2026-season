package frc.robot.subsystems.shooterIndexer;

public interface ShooterIndexerIO {
    public void runShooterIndexer();

    public void backwardsRunShooterIndexer();

    public void defaultBehavior();

    public void updateStates(ShooterIndexerIOState state);

    public void logShooterSpeedsToCSV();

    public static class ShooterIndexerIOState {
        public double shooterIndexerDesiredSpeed = 0;
        public double shooterIndexerActualSpeed = 0;
        public double shooterIndexerAppliedVoltage = 0;
        public double shooterIndexerCurrent = 0;
    }
}
