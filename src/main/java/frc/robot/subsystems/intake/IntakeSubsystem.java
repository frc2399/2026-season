package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOStates;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOStates states = new IntakeIOStates();

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
        io.periodicUpdate();
    }
}
