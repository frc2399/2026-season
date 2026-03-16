package frc.robot.subsystems.shooter;

public class ShooterPlacebo implements ShooterIO {
    public void runShooter() {}

    public void defaultBehavior() {}

    @Override
    public void updateStates(ShooterIOState state) {}

    @Override
    public ShooterSpeeds getCurrentTopAndBottomSpeeds() {
        return new ShooterSpeeds(50 * Math.random(), 50 * Math.random());
    }

    @Override
    public void runTunableNumberSetpoints() {}

    @Override
    public void passFuel() {}
}
