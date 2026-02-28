package frc.robot.subsystems.intake;

public interface IntakeIO {
    public static class IntakeIOStates {
        public double desiredSpeed = 0.0;
        public double actualSpeed = 0.0;
        public double current = 0.0;
        public double appliedVoltage = 0.0;
    }

    public void updateStates(IntakeIOStates states);

    public void runIntake();

    public void setZero();

    public void periodicUpdate();
}
