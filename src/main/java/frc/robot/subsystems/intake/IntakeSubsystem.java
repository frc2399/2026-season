package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeArmIO.IntakeArmSetpoint;
import frc.robot.subsystems.intake.IntakeRollerIO.IntakeRollerIOState;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeRollerIO rollerIO;
    private IntakeArmIO armIO;
    private IntakeRollerIOState rollerState = new IntakeRollerIOState();

    public IntakeSubsystem(IntakeRollerIO rollerIO, IntakeArmIO armIO) {
        this.rollerIO = rollerIO;
        this.armIO = armIO;
    }

    public Command deployAndRunIntake() {
        return this.run(
                        () -> {
                            armIO.setSetpoint(IntakeArmSetpoint.DEPLOYED);
                            rollerIO.runIntake();
                        })
                .withName("intake: deploy arm and run roller");
    }

    public Command defaultBehavior() {
        return this.run(
                        () -> {
                            rollerIO.setZero();
                            armIO.setSetpoint(IntakeArmSetpoint.STOWED);
                        })
                .withName("intake defaultBehavior (arm stoed + roller at 0)");
    }

    @Override
    public void periodic() {
        rollerIO.updateState(rollerState);

        SmartDashboard.putNumber("intake/roller/desired speed", rollerState.desiredSpeed);
        SmartDashboard.putNumber("intake/roller/actual speed", rollerState.actualSpeed);
        SmartDashboard.putNumber("intake/roller/current", rollerState.current);
        SmartDashboard.putNumber("intake/roller/applied output", rollerState.appliedVoltage);
    }
}
