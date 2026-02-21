package frc.robot.subsystems.intake;

public interface IntakeRollerIO {
    public void runIntake();

    public void setZero();

    public void updateState(IntakeRollerIOState state);

    public static class IntakeRollerIOState {
        public double desiredSpeedRadiansPerSecond = 0;
        public double actualSpeedRadiansPerSecond = 0;
        public double current = 0;
        public double appliedVoltage = 0;
    }
}
