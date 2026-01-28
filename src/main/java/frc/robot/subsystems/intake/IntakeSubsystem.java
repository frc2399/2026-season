package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;


    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }
    
    public Command runIntake() {
        return this.run(() -> io.runIntake()).withName("runIntake");
    }
}
