public class ClimberSubsystem extends SubsystemBase {
    private IndexerIO io;

    public ClimberSubsystem(IndexerIO io) {
        this.io = io;
    }

    public Command climber () {
        return this.run(() -> io.runIndexer().withName("runIndexer"));
    }
}
