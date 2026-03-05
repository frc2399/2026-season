package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO io;

    public Command extend() {
        return this.run(() -> io.extend()).withName("climberExtend");
    }

    public Command retract() {
        return this.run(() -> io.retract()).withName("climberRetract");
    }

    public Command runForwards() {
        return this.run(() -> io.runForwards()).withName("climberRunForwards");
    }

    public Command runBackwards() {
        return this.run(() -> io.runBackwards()).withName("climberRunBackwards");
    }

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }
}
