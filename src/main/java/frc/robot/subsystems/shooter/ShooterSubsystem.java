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
                "shooter/topRollerDesiredSpeed", shooterStates.topRollerDesiredSpeedRad_P_S);
        SmartDashboard.putNumber(
                "shooter/topRollerActualSpeed", shooterStates.topRollerActualSpeedRad_P_S);
        SmartDashboard.putNumber("shooter/topRollerCurrent", shooterStates.topRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/topRollerAppliedVoltage", shooterStates.topRollerAppliedVoltage);
        SmartDashboard.putNumber(
                "shooter/bottomRollerDesiredSpeed", shooterStates.bottomRollerDesiredSpeedRad_P_S);
        SmartDashboard.putNumber(
                "shooter/bottomRollerActualSpeed", shooterStates.bottomRollerActualSpeedRad_P_S);
        SmartDashboard.putNumber("shooter/bottomRollerCurrent", shooterStates.bottomRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/bottomRollerAppliedVoltage", shooterStates.bottomRollerAppliedVoltage);
    }
}
