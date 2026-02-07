package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOState;

public class IndexerSubsystem extends SubsystemBase {

    private IndexerIO io;

    private IndexerIOState indexerState = new IndexerIOState();

    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    public Command indexer() {
        return this.run(() -> io.runIndexer()).withName("runIndexer");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("indexerDefaultBehavior");
    }

    @Override
    public void periodic() {
        io.updateStates(indexerState);
        SmartDashboard.putNumber("Indexer/desiredspeed", indexerState.Indexerdesiredspeed);
        SmartDashboard.putNumber("Indexer/actualspeed", indexerState.Indexeractualspeed);
        SmartDashboard.putNumber("Indexer/drivevoltage", indexerState.driveVoltage);
        SmartDashboard.putNumber("Indexer/drivecurrent", indexerState.driveCurrent);
    }
}
