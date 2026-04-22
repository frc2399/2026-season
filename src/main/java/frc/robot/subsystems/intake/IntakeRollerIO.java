package frc.robot.subsystems.intake;

public interface IntakeRollerIO {
    public void runIntake();

    public void runIntakeBackwards();

    public void setZero();

    public void updateState(IntakeRollerIOState state);

    public static class IntakeRollerIOState {
        public double desiredSpeedRadiansPerSecond = 0;
        public double leaderActualSpeedRadiansPerSecond = 0;
        public double followerActualSpeedRadiansPerSecond = 0;
        public double leaderCurrent = 0;
        public double followerCurrent = 0;
        public double leaderAppliedVoltage = 0;
        public double followerAppliedVoltage = 0;
    }
}
