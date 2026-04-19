package frc.robot.subsystems.intake;

public interface IntakeRollerIO {
    public void runIntake();

    public void runIntakeBackwards();

    public void setZero();

    public void updateState(IntakeRollerIOState state);

    public static class IntakeRollerIOState {
        public double leaderDesiredSpeedRadiansPerSecond = 0;
        public double leaderActualSpeedRadiansPerSecond = 0;
        public double followerActualSpeedRadiansPerSecond = 0;
        public double followerDesiredSpeedRadiansPerSecond = 0;
        public double current = 0;
        public double appliedVoltage = 0;
    }
}
