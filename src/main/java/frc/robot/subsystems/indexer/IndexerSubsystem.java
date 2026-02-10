package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private IndexerIO io;

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    public Command runIndexer() {
        return this.run(() -> io.runIndexer()).withName("runIndexer");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("indexerDefaultBehavior");
    }
}
