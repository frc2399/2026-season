package frc.robot.subsystems.drive;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveSubsystemConfigurations {
    

    /*
     * A record is a shorthand way of making a class that's only really meant to store data.
     * The Java compiler will auto-generate a constructor and instance variables that match the parameters.
     * As a result, the code is shorter, cleaner, and easier to read!
     */
    public record DriveConfig (
        double kS,
        double kV,
        double driveP,
        double driveD,
        double turnP,
        double turnD,
        double headingP,
        double headingD,
        Distance trackWidth,
        Distance trackLength,
        LinearAcceleration maxAccel,
        LinearVelocity maxlLinearSpeed,
        AngularAcceleration maxAngularAccel   
    ) {}
}
