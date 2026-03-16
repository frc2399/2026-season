package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    public void runShooterWithSpeeds(
            AngularVelocity topSpeed, AngularVelocity bottomSpeed, boolean shouldInterpolate);

    public void defaultBehavior();

    public void updateStates(ShooterIOState state);

    public ShooterSpeeds getCurrentTopAndBottomSpeeds();

    public void runTunableNumberSetpoints();

    public boolean isUpToSpeed();

    public void passFuel();

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

    public record ShooterSpeeds(
            double topSpeedRadiansPerSecond, double bottomSpeedRadiansPerSecond) {}
}
