package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeRollerIOState;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;
    private IntakeRollerIOState rollerState = new IntakeRollerIOState();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public Command runIntake() {
        return this.run(() -> io.runIntake()).withName("runIntake");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.setZero()).withName("defaultBehavior");
    }

    @Override
    public void periodic() {
        io.updateState(rollerState);

        SmartDashboard.putNumber("intake/roller/desired speed", rollerState.desiredSpeed);
        SmartDashboard.putNumber("intake/roller/actual speed", rollerState.actualSpeed);
        SmartDashboard.putNumber("intake/roller/current", rollerState.current);
        SmartDashboard.putNumber("intake/roller/applied output", rollerState.appliedVoltage);
    }
}
