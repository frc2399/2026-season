package frc.robot.subsystems.spindexer;

public interface SpindexerIO {

    public void runSpindexer();

    public void defaultBehavior();

    public void updateStates(SpindexerIOState state);

    public static class SpindexerIOState {
        public double SpindexerDesiredSpeed = 0;
        public double SpindexerActualSpeed = 0;
        public double SpindexerAppliedVoltage = 0;
        public double SpindexerCurrent = 0;
    }
}
