package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;

import com.opencsv.CSVWriter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOState;
import frc.robot.subsystems.shooter.ShooterIO.ShooterSpeeds;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOState shooterStates = new ShooterIOState();

    // for csv logging
    ZoneId LOGGING_TIMEZONE =
            ZoneId.of("America/New_York"); // so we can display in our local time zone
    private static final DateTimeFormatter LOGGING_TIME_FORMATTER =
            DateTimeFormatter.ISO_OFFSET_DATE_TIME;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command shoot() {
        return this.run(() -> io.runShooter()).withName("runShooter");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterDefaultBehavior");
    }

    // fake default command so it runs the tunable number speed setpoints instead of actual default
    // command
    public Command tuningSetpoint() {
        return this.run(() -> io.runTunableNumberSetpoints()).withName("tuningDefaultCommand");
    }

    //     public void logShooterSpeedsToCSVCommand() {
    //         this.runOnce(() -> logShooterSpeedsToCSV());
    //     }

    public void logShooterSpeedsToCSV(Distance distanceToHub) {
        // code modified from
        // https://www.geeksforgeeks.org/java/writing-a-csv-file-in-java-using-opencsv/
        try {
            File csvFile = new File("/home/lvuser/shooter-speeds.csv");
            FileWriter outputWriter = new FileWriter(csvFile, true);
            CSVWriter csvWriter = new CSVWriter(outputWriter);

            ShooterSpeeds desiredShooterSpeeds = io.getCurrentTopAndBottomSpeeds();
            double desiredSpeedTopRadiansPerSecond =
                    desiredShooterSpeeds.topSpeedRadiansPerSecond();
            double desiredSpeedBottomRadiansPerSecond =
                    desiredShooterSpeeds.bottomSpeedRadiansPerSecond();

            String timestamp = ZonedDateTime.now(LOGGING_TIMEZONE).format(LOGGING_TIME_FORMATTER);

            String[] data =
                    new String[] {
                        Double.toString(distanceToHub.in(Inches)),
                        Double.toString(desiredSpeedTopRadiansPerSecond),
                        Double.toString(desiredSpeedBottomRadiansPerSecond),
                        timestamp
                    };
            csvWriter.writeNext(data);

            csvWriter.close();
        } catch (IOException e) {
            System.out.println(
                    "********COULD NOT WRITE TO THE SHOOTER SPEEDS CSV - SEE STACK TRACE********");
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        io.updateStates(shooterStates);
        SmartDashboard.putNumber(
                "shooter/topRollerDesiredSpeed (rad per s)", shooterStates.topRollerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooter/topRollerActualSpeed (rad per s)", shooterStates.topRollerActualSpeed);
        SmartDashboard.putNumber("shooter/topRollerCurrent (amps)", shooterStates.topRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/topRollerAppliedVoltage (volt)", shooterStates.topRollerAppliedVoltage);
        SmartDashboard.putNumber(
                "shooter/bottomRollerDesiredSpeed (rad per s)",
                shooterStates.bottomRollerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooter/bottomRollerActualSpeed (rad per s)",
                shooterStates.bottomRollerActualSpeed);
        SmartDashboard.putNumber(
                "shooter/bottomRollerCurrent (amps)", shooterStates.bottomRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/bottomRollerAppliedVoltage (volt)",
                shooterStates.bottomRollerAppliedVoltage);
    }
}
