package frc.robot.subsystems.spindexer;

import edu.wpi.first.units.measure.AngularVelocity;

public interface SpindexerIO {

    void runSpindexerAtSpeed(AngularVelocity speed);

    public void runSpindexer();

    public void defaultBehavior();

    public void updateStates(SpindexerIOState state);

    public void runSpindexerBackwards();

    public static class SpindexerIOState {
        public double spindexerDesiredSpeedRad_P_S = 0;
        public double spindexerActualSpeedRad_P_S = 0;
        public double spindexerAppliedVoltage = 0;
        public double spindexerCurrent = 0;
    }
}
