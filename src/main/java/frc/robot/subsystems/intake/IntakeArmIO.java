package frc.robot.subsystems.intake;

public interface IntakeArmIO {
    public void setSetpoint(IntakeArmSetpoint setpoint);

    public enum IntakeArmSetpoint {
        DEPLOYED,
        STOWED
    }
}
