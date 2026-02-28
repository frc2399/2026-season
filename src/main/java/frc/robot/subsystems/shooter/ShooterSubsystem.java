package frc.robot.subsystems.shooter;

import com.opencsv.CSVWriter;
import edu.wpi.first.wpilibj.Filesystem;
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

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOState shooterStates = new ShooterIOState();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public Command shoot() {
        return this.run(() -> io.runShooter()).withName("runShooter");
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterDefaultBehavior");
    }

    public Command logShooterSpeedsToCSVCommand() {
        return this.runOnce(() -> logShooterSpeedsToCSV());
    }

    public void logShooterSpeedsToCSV() {
        System.out.println("we are calling the method :(");
        // code modified from
        // https://www.geeksforgeeks.org/java/writing-a-csv-file-in-java-using-opencsv/
        try {
            File csvFile = new File(Filesystem.getDeployDirectory(), "shooter-speeds.csv");
            FileWriter outputWriter = new FileWriter(csvFile, true);
            CSVWriter csvWriter = new CSVWriter(outputWriter);

            ShooterSpeeds desiredShooterSpeeds = io.getCurrentTopAndBottomSpeeds();
            double desiredSpeedTop = desiredShooterSpeeds.topSpeed();
            double desiredSpeedBottom = desiredShooterSpeeds.bottomSpeed();

            ZoneId zone = ZoneId.of("America/New_York"); // so we can display in our local time zone
            ZonedDateTime zonedDateTime = ZonedDateTime.now(zone);

            String[] data =
                    new String[] {
                        "distance (currently unable to log)",
                        "" + desiredSpeedTop,
                        "" + desiredSpeedBottom,
                        zonedDateTime.toString()
                    };
            csvWriter.writeNext(data);

            csvWriter.close();
            System.out.println("yippee we logged!");
            System.out.println(
                    ""
                            + desiredSpeedTop
                            + " "
                            + desiredSpeedBottom
                            + " "
                            + zonedDateTime.toString());
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
                "shooter/topRollerDesiredSpeed", shooterStates.topRollerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooter/topRollerActualSpeed", shooterStates.topRollerActualSpeed);
        SmartDashboard.putNumber("shooter/topRollerCurrent", shooterStates.topRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/topRollerAppliedVoltage", shooterStates.topRollerAppliedVoltage);
        SmartDashboard.putNumber(
                "shooter/bottomRollerDesiredSpeed", shooterStates.bottomRollerDesiredSpeed);
        SmartDashboard.putNumber(
                "shooter/bottomRollerActualSpeed", shooterStates.bottomRollerActualSpeed);
        SmartDashboard.putNumber("shooter/bottomRollerCurrent", shooterStates.bottomRollerCurrent);
        SmartDashboard.putNumber(
                "shooter/bottomRollerAppliedVoltage", shooterStates.bottomRollerAppliedVoltage);
        io.periodicUpdate();
    }
}
