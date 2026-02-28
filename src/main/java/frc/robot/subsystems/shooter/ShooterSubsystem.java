package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.TargetFuel;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOState;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOState shooterStates = new ShooterIOState();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command shoot(TargetFuel targetFuel) {
        return this.run(() -> io.runShooter(targetFuel)).withName("runShooter");

    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterDefaultBehavior");
    }

    // fake default command so it runs the tunable number speed setpoints instead of actual default
    // command
    public Command tuningSetpoint() {
        return this.run(() -> {}).withName("tuningDefaultCommand");
    }

    @Override
    public void periodic() {
        io.updateStates(shooterStates);
        SmartDashboard.putNumber(
                "shooter/topRollerDesiredSpeed", shooterStates.topRollerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooter/topRollerActualSpeed", shooterStates.topRollerActualSpeed);
        SmartDashboard.putNumber("shooter/topRollerCurrent", shooterStates.topRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/topRollerAppliedVoltage", shooterStates.topRollerAppliedVoltage);
        SmartDashboard.putNumber(
                "shooter/bottomRollerDesiredSpeed", shooterStates.bottomRollerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooter/bottomRollerActualSpeed", shooterStates.bottomRollerActualSpeed);
        SmartDashboard.putNumber("shooter/bottomRollerCurrent", shooterStates.bottomRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/bottomRollerAppliedVoltage", shooterStates.bottomRollerAppliedVoltage);
        io.periodicUpdate();
    }
}
