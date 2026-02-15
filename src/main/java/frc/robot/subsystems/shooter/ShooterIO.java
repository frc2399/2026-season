package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void runShooter();

    public void defaultBehavior();

    public void updateStates(ShooterIOState state);

    public static class ShooterIOState {
        public double topRollerDesiredSpeed = 0.0;
        public double topRollerActualSpeed = 0.0;
        public double topRollerCurrent = 0.0;
        public double topRollerAppliedVoltage = 0.0;
        public double bottomRollerDesiredSpeed = 0.0;
        public double bottomRollerActualSpeed = 0.0;
        public double bottomRollerCurrent = 0.0;
        public double bottomRollerAppliedVoltage = 0.0;
    }
}
