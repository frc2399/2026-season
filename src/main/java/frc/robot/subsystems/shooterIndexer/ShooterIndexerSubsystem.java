package frc.robot.subsystems.shooterIndexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooterIndexer.ShooterIndexerIO.ShooterIndexerIOState;

public class ShooterIndexerSubsystem extends SubsystemBase {
    private ShooterIndexerIO io;

    private ShooterIndexerIOState shooterIndexerState = new ShooterIndexerIOState();

    public ShooterIndexerSubsystem(ShooterIndexerIO io) {
        this.io = io;
    }

    public Command runShooterIndexer() {
        return this.run(() -> io.runShooterIndexer()).withName("runShooterIndexer");
    }

    public Command backwardsShooterIndexer() {
        return this.run(() -> io.backwardsRunShooterIndexer())
                .withName("backwardsRunShooterIndexer");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterIndexerDefaultBehavior");
    }

    public Command logShooterSpeedsToCSV() {
        return this.runOnce(() -> io.logShooterSpeedsToCSV());
    }

    @Override
    public void periodic() {
        io.updateStates(shooterIndexerState);
        SmartDashboard.putNumber(
                "shooterIndexer/desiredSpeed", shooterIndexerState.shooterIndexerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooterIndexer/actualSpeed", shooterIndexerState.shooterIndexerActualSpeed);
        SmartDashboard.putNumber(
                "shooterIndexer/driveVoltage", shooterIndexerState.shooterIndexerAppliedVoltage);
        SmartDashboard.putNumber(
                "shooterIndexer/driveCurrent", shooterIndexerState.shooterIndexerCurrent);
    }
}
