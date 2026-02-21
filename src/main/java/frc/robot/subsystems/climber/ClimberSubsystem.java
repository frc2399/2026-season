package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public Command runClimber() {
        return this.run(() -> io.runClimber()).withName("runClimber");
    }

    public Command runClimberDefaultBehavior() {
        return this.run(() -> io.runClimberDefaultBehavior()).withName("runClimberDefaultBeavhior");
    }
}
