package frc.robot.subsystems.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakeArm.IntakeArmIO.IntakeArmSetpoint;

public class IntakeArmSubsystem extends SubsystemBase {
    private IntakeArmIO io;

    public IntakeArmSubsystem(IntakeArmIO io) {
        this.io = io;
    }

    public Command deploy() {
        return this.run(() -> io.setSetpoint(IntakeArmSetpoint.DEPLOYED)).withName("deploy");
    }

    public Command stow() {
        return this.run(() -> io.setSetpoint(IntakeArmSetpoint.STOWED)).withName("stow");
    }
}
