package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import frc.robot.CommandFactory.TargetFuel;

public class ShooterPlacebo implements ShooterIO {
    public void runShooter(TargetFuel targetFuel) {}

    public void defaultBehavior() {}

    @Override
    public void updateStates(ShooterIOState state) {}

    @Override
    public void periodicUpdate() {}
}
