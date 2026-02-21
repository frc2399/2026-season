package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOState;

public class SpindexerSubsystem extends SubsystemBase {

    private SpindexerIO io;

    private SpindexerIOState spindexerState = new SpindexerIOState();

    public SpindexerSubsystem(SpindexerIO io) {
        this.io = io;
    }

    public Command runSpindexer() {
        return this.run(() -> io.runSpindexer()).withName("runSpindexer");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("spindexerDefaultBehavior");
    }

    @Override
    public void periodic() {
        io.updateStates(spindexerState);
        SmartDashboard.putNumber(
                "Spindexer/desiredSpeed", spindexerState.spindexerDesiredSpeedRad_P_S);
        SmartDashboard.putNumber(
                "Spindexer/actualSpeed", spindexerState.spindexerActualSpeedRad_P_S);
        SmartDashboard.putNumber("Spindexer/driveVoltage", spindexerState.spindexerAppliedVoltage);
        SmartDashboard.putNumber("Spindexer/driveCurrent", spindexerState.spindexerCurrent);
    }
}
