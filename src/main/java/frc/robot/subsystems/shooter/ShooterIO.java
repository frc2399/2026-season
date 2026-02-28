package frc.robot.subsystems.shooter;

public interface ShooterIO {
    public void runShooter();

    public void defaultBehavior();

    public void updateStates(ShooterIOState state);

    public void periodicUpdate();

    public static class ShooterIOState {
        public double topRollerDesiredSpeedRad_P_S = 0.0;
        public double topRollerActualSpeedRad_P_S = 0.0;
        public double topRollerCurrent = 0.0;
        public double topRollerAppliedVoltage = 0.0;
        public double bottomRollerDesiredSpeedRad_P_S = 0.0;
        public double bottomRollerActualSpeedRad_P_S = 0.0;
        public double bottomRollerCurrent = 0.0;
        public double bottomRollerAppliedVoltage = 0.0;
    }
}
