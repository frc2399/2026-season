package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    public void runShooterWithSpeeds(AngularVelocity topSpeed, AngularVelocity bottomSpeed);

    public void defaultBehavior();

    public void updateStates(ShooterIOState state);

    public ShooterSpeeds getCurrentTopAndBottomSpeeds();

    public void runTunableNumberSetpoints();

    public boolean isUpToSpeed();

    public void passFuelOrManualShoot();

    public default void runTopRollerVolts(Voltage volts) {}

    public default void runBottomRollerVolts(Voltage volts) {}

    public static class ShooterIOState {
        public double topRollerDesiredSpeed = 0.0;
        public double topRollerActualSpeed = 0.0;
        public double topRollerCurrent = 0.0;
        public double topRollerAppliedVoltage = 0.0;
        public double topRollerPositionRadians = 0.0;
        public double bottomRollerDesiredSpeed = 0.0;
        public double bottomRollerActualSpeed = 0.0;
        public double bottomRollerCurrent = 0.0;
        public double bottomRollerAppliedVoltage = 0.0;
        public boolean isUpToSpeed = false;
        public double bottomRollerPositionRadians = 0.0;
    }

    public record ShooterSpeeds(
            double topSpeedRadiansPerSecond, double bottomSpeedRadiansPerSecond) {}
}
