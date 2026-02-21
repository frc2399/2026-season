package frc.robot.subsystems.intake;

public interface IntakeIO {
    public void runIntake();

    public void setZero();

    public void updateState(IntakeRollerIOState state);

    public static class IntakeRollerIOState {
        public double desiredSpeed = 0;
        public double actualSpeed = 0;
        public double current = 0;
        public double appliedVoltage = 0;
    }
}
