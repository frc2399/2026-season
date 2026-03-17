package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterPlacebo implements ShooterIO {
    public void runShooterWithSpeeds(
            AngularVelocity topSpeed, AngularVelocity bottomSpeed, boolean shouldInterpolate) {}

    public void defaultBehavior() {}

    @Override
    public void updateStates(ShooterIOState state) {}

    @Override
    public boolean isUpToSpeed() {
        return true;
    }

    @Override
    public ShooterSpeeds getCurrentTopAndBottomSpeeds() {
        return new ShooterSpeeds(50 * Math.random(), 50 * Math.random());
    }

    @Override
    public void runTunableNumberSetpoints() {}

    @Override
    public void passFuelOrManualShoot() {}
}
