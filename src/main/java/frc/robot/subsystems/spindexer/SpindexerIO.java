package frc.robot.subsystems.spindexer;

public interface SpindexerIO {

    public void runSpindexer();

    public void defaultBehavior();

    public void updateStates(SpindexerIOState state);

    public static class SpindexerIOState {
        public double Spindexerdesiredspeed = 0;
        public double Spindexeractualspeed = 0;
        public double driveVoltage = 0;
        public double driveCurrent = 0;
    }
}
