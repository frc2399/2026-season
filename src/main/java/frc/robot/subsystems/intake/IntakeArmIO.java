package frc.robot.subsystems.intake;

public interface IntakeArmIO {
    public void setSetpoint(IntakeArmSetpoint setpoint);

    public enum IntakeArmSetpoint {
        DEPLOYED,
        STOWED
    }

    public void updateState(IntakeArmIOState state);

    public static class IntakeArmIOState {
        public double desiredAngleDegrees = 0;
        public double actualAngleDegrees = 0;
        public double velocityDegreesPerSecond = 0; 
        public double appliedVoltage = 0;
        public double current = 0;
    }
}
