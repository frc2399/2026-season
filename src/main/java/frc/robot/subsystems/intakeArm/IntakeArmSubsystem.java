package frc.robot.subsystems.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakeArm.IntakeArmIO.IntakeArmSetpoint;

public class IntakeArmSubsystem extends SubsystemBase {
    private IntakeArmIO io;

    public IntakeArmSubsystem(IntakeArmIO io) {
        this.io = io;
    }

    public Command intakeArmDeploy() {
        return this.runOnce(() -> io.setSetpoint(IntakeArmSetpoint.DEPLOYED))
                .withName("intakeArmDeploy");
    }

    public Command intakeArmStow() {
        return this.runOnce(() -> io.setSetpoint(IntakeArmSetpoint.STOWED)).withName("intakeArmStow");
    }
}
