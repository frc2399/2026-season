package frc.robot.subsystems.indexer;

public interface IndexerIO {

    public void runIndexer();

    public void defaultBehavior();

    public void updateStates(IndexerIOState state);

    public static class IndexerIOState {
        public double Indexerdesiredspeed = 0;
        public double Indexeractualspeed = 0;
        public double driveVoltage = 0;
        public double driveCurrent = 0;
    }
}
