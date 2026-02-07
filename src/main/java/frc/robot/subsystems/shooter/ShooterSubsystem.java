package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOState;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOState shooterStates = new ShooterIOState();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command shoot() {
        return this.run(() -> io.runShooter()).withName("runShooter");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterDefaultBehavior");
    }

    @Override
    public void periodic() {
        io.updateStates(shooterStates);
        SmartDashboard.putNumber(
                "shoot/topRollerDesiredSpeed", shooterStates.topRollerDesiredSpeed);
        SmartDashboard.putNumber("topRollerActualSpeed", shooterStates.topRollerActualSpeed);
        SmartDashboard.putNumber("topRollerCurrent", shooterStates.topRollerCurrent);
        SmartDashboard.putNumber("topRollerAppliedVoltage", shooterStates.topRollerAppliedVoltage);
        SmartDashboard.putNumber(
                "bottomRollerDesiredSpeed", shooterStates.bottomRollerDesiredSpeed);
        SmartDashboard.putNumber("bottomRollerActualSpeed", shooterStates.bottomRollerActualSpeed);
        SmartDashboard.putNumber("bottomRollerCurrent", shooterStates.bottomRollerCurrent);
        SmartDashboard.putNumber(
                "bottomRollerAppliedVoltage", shooterStates.bottomRollerAppliedVoltage);
    }
}
