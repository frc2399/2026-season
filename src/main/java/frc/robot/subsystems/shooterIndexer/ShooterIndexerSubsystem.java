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

    @Override
    public void periodic() {
        io.updateStates(shooterIndexerState);
        SmartDashboard.putNumber(
                "shooterIndexer/desired speed (rad per s)",
                shooterIndexerState.shooterIndexerDesiredSpeedRad_P_S);
        SmartDashboard.putNumber(
                "shooterIndexer/actual speed (rad per s)",
                shooterIndexerState.shooterIndexerActualSpeedRad_P_S);
        SmartDashboard.putNumber(
                "shooterIndexer/voltage (volt)", shooterIndexerState.shooterIndexerAppliedVoltage);
        SmartDashboard.putNumber(
                "shooterIndexer/current (amps)", shooterIndexerState.shooterIndexerCurrent);
    }
}
