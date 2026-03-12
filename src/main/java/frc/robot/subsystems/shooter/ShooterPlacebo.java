package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterPlacebo implements ShooterIO {
    public void runShooterWithSpeeds(
            AngularVelocity topSpeed, AngularVelocity bottomSpeed, boolean shouldInterpolate) {
        System.out.println("am " + shouldInterpolate + " interpolating");
        System.out.println("top" + topSpeed.in(RadiansPerSecond));
        System.out.println("bottom" + bottomSpeed.in(RadiansPerSecond));
    }

    public void defaultBehavior() {}

    @Override
    public void updateStates(ShooterIOState state) {}

    @Override
    public ShooterSpeeds getCurrentTopAndBottomSpeeds() {
        return new ShooterSpeeds(50 * Math.random(), 50 * Math.random());
    }

    @Override
    public void runTunableNumberSetpoints() {}
}
