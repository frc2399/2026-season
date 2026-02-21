package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO io;

    public void extend() {}

    public void retract() {}

    public void runForwards() {}

    public void runBackwards() {}

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }
}
