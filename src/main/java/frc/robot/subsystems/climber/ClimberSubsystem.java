package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO io;

    public Command extend() {
        return this.run(() -> io.extend()).withName("extend");
    }

    public Command retract() {
        return this.run(() -> io.retract()).withName("retract");
    }

    public Command runForwards() {
        return this.run(() -> io.runForwards()).withName("runForwards");
    }

    public Command runBackwards() {
        return this.run(() -> io.runBackwards()).withName("runBackwards");
    }

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }
}
