public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public Command climber () {
        return this.run(() -> io.runClimber().withName("runClimber"));
    }

    public Command climber () {
        return this.run(() -> io.climberDefaultBehavior().withName("climberDefaultBeavhior"));
    }
}
