package frc.robot.subsystems.spindexer;

public interface SpindexerIO {

    public void runSpindexer();

    public void defaultBehavior();

    public void updateStates(SpindexerIOState state);

    public void runSpindexerBackwards();

    public static class SpindexerIOState {
        public double spindexerDesiredSpeed = 0;
        public double spindexerActualSpeed = 0;
        public double spindexerAppliedVoltage = 0;
        public double spindexerCurrent = 0;
    }
}
