package frc.robot.subsystems.intakeArm;

public interface IntakeArmIO {
    public void setSetpoint(IntakeArmSetpoint setpoint);

    public enum IntakeArmSetpoint {
        DEPLOYED,
        STOWED
    }
}
