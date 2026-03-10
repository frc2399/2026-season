package frc.robot.subsystems.spindexer;

public interface SpindexerIO {

    public void runSpindexer();

    public void defaultBehavior();

    public void updateStates(SpindexerIOState state);

    public static class SpindexerIOState {
        public double spindexerDesiredSpeedRad_P_S = 0;
        public double spindexerActualSpeedRad_P_S = 0;
        public double spindexerAppliedVoltage = 0;
        public double spindexerCurrent = 0;
    }
}
