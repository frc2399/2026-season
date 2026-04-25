package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;
import com.opencsv.CSVWriter;
import com.opencsv.exceptions.CsvValidationException;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandFactory.TargetZoneType;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOState;
import frc.robot.subsystems.shooter.ShooterIO.ShooterSpeeds;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOState shooterStates = new ShooterIOState();
    private String csvFilepath;
    private boolean shouldInterpolate = false;

    // for csv logging
    ZoneId LOGGING_TIMEZONE =
            ZoneId.of("America/New_York"); // so we can display in our local time zone
    private static final DateTimeFormatter LOGGING_TIME_FORMATTER =
            DateTimeFormatter.ISO_OFFSET_DATE_TIME;

    // for linear interpolating between distances for shooter
    // if we want to allow things besides Double, we have to write custom implementations as far as
    // i can tell, so they're doubles, with the key as inches and the value as RPM
    InterpolatingTreeMap<Double, Double> topShooterSpeedTreeMapMeterRadS =
            new InterpolatingTreeMap<Double, Double>(
                    InverseInterpolator.forDouble(), Interpolator.forDouble());
    InterpolatingTreeMap<Double, Double> bottomShooterSpeedTreeMapMeterRadS =
            new InterpolatingTreeMap<Double, Double>(
                    InverseInterpolator.forDouble(), Interpolator.forDouble());

    // for both: m = (y2-y1) / (x2-x1)
    // TOP SHOOTER INTERPOLATION
    private static final Distance TOP_DISTANCE_POINT_ONE = Inches.of(127);
    private static final Distance TOP_DISTANCE_POINT_TWO = Inches.of(250);
    private static final AngularVelocity TOP_SPEED_POINT_ONE = RadiansPerSecond.of(260);
    private static final AngularVelocity TOP_SPEED_POINT_TWO = RadiansPerSecond.of(360);
    private static final double TOP_SHOOTER_SLOPE_RADIANS_PER_SECOND_PER_METER =
            TOP_SPEED_POINT_TWO.minus(TOP_SPEED_POINT_ONE).in(RadiansPerSecond)
                    / TOP_DISTANCE_POINT_TWO.minus(TOP_DISTANCE_POINT_ONE).in(Meters);

    // BOTTOM SHOOTER INTERPOLATION
    private static final Distance BOTTOM_DISTANCE_POINT_ONE = Inches.of(127);
    private static final Distance BOTTOM_DISTANCE_POINT_TWO = Inches.of(250);
    private static final AngularVelocity BOTTOM_SPEED_POINT_ONE = RadiansPerSecond.of(0);
    private static final AngularVelocity BOTTOM_SPEED_POINT_TWO = RadiansPerSecond.of(0);
    private static final double BOTTOM_SHOOTER_SLOPE_RADIANS_PER_SECOND_PER_METER =
            BOTTOM_SPEED_POINT_TWO.minus(BOTTOM_SPEED_POINT_ONE).in(RadiansPerSecond)
                    / BOTTOM_DISTANCE_POINT_TWO.minus(BOTTOM_DISTANCE_POINT_ONE).in(Meters);

    public ShooterSubsystem(ShooterIO io, String csvFilepath) {
        this.io = io;
        this.csvFilepath = csvFilepath;
        if (!csvFilepath.equals("")) {
            readCSV();
        }
    }

    public Command shoot(
            Supplier<Distance> distFromTarget,
            boolean shouldManualShoot,
            Supplier<TargetZoneType> targetZoneType) {
        if (shouldManualShoot) {
            return this.run(
                            () -> {
                                AngularVelocity topSpeed = RadiansPerSecond.of(222.529479629277);
                                RadiansPerSecond.of(
                                        topShooterSpeedTreeMapMeterRadS.get(
                                                distFromTarget.get().in(Meters)));
                                AngularVelocity bottomSpeed =
                                        RadiansPerSecond.of(233.00145514124299);
                                RadiansPerSecond.of(
                                        bottomShooterSpeedTreeMapMeterRadS.get(
                                                distFromTarget.get().in(Meters)));
                                // making sure it doesnt interpolate for manual shoot
                                io.runShooterWithSpeeds(topSpeed, bottomSpeed);
                            })
                    .withName("runShooter");
        } else {
            return this.run(
                            () -> {
                                AngularVelocity topSpeed;
                                AngularVelocity bottomSpeed;
                                if (targetZoneType.get() == TargetZoneType.HUB) {
                                    topSpeed =
                                            RadiansPerSecond.of(
                                                    topShooterSpeedTreeMapMeterRadS.get(
                                                            distFromTarget.get().in(Meters)));
                                    bottomSpeed =
                                            RadiansPerSecond.of(
                                                    bottomShooterSpeedTreeMapMeterRadS.get(
                                                            distFromTarget.get().in(Meters)));
                                } else {
                                    // point slope form: y-y1 = m(x-x1) => y = m(x-x1) + y1
                                    topSpeed =
                                            RadiansPerSecond.of(
                                                    TOP_SHOOTER_SLOPE_RADIANS_PER_SECOND_PER_METER
                                                                    * (distFromTarget
                                                                            .get()
                                                                            .minus(
                                                                                    TOP_DISTANCE_POINT_ONE)
                                                                            .in(Meters))
                                                            + TOP_SPEED_POINT_ONE.in(
                                                                    RadiansPerSecond));
                                    bottomSpeed =
                                            RadiansPerSecond.of(
                                                    BOTTOM_SHOOTER_SLOPE_RADIANS_PER_SECOND_PER_METER
                                                                    * (distFromTarget
                                                                            .get()
                                                                            .minus(
                                                                                    BOTTOM_DISTANCE_POINT_ONE)
                                                                            .in(Meters))
                                                            + BOTTOM_SPEED_POINT_ONE.in(
                                                                    RadiansPerSecond));

                                    if (topSpeed.in(RadiansPerSecond)
                                            > RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(
                                                    RadiansPerSecond)) {
                                        topSpeed = RobotConstants.MotorConstants.VORTEX_FREE_SPEED;
                                    }

                                    if (bottomSpeed.in(RadiansPerSecond)
                                            > RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(
                                                    RadiansPerSecond)) {
                                        bottomSpeed =
                                                RobotConstants.MotorConstants.VORTEX_FREE_SPEED;
                                    }
                                }
                                io.runShooterWithSpeeds(topSpeed, bottomSpeed);
                            })
                    .withName("runShooter");
        }
    }

    public Command defaultBehavior() {
        return this.run(() -> io.defaultBehavior()).withName("shooterDefaultBehavior");
    }

    public boolean isUpToSpeed() {
        return io.isUpToSpeed();
    }

    // fake default command so it runs the tunable number speed setpoints instead of actual default
    // command
    public Command tuningSetpoint() {
        return this.run(() -> io.runTunableNumberSetpoints()).withName("tuningDefaultCommand");
    }

    public void readCSV() {
        try {
            File csvFile = new File(csvFilepath);
            FileReader fileReader = new FileReader(csvFile);
            // skips the header
            CSVReaderBuilder builder = new CSVReaderBuilder(fileReader).withSkipLines(1);
            CSVReader reader = builder.build();
            String[] line;
            while ((line = reader.readNext()) != null) {
                // order: distance (inches), top speed (rad/s), bottom speed (rad/s), timestamp
                Distance distanceToHub = Inches.of(Double.valueOf(line[0]));
                AngularVelocity topSpeed = RadiansPerSecond.of(Double.valueOf(line[1]));
                AngularVelocity bottomSpeed = RadiansPerSecond.of(Double.valueOf(line[2]));

                topShooterSpeedTreeMapMeterRadS.put(
                        distanceToHub.in(Meters), topSpeed.in(RadiansPerSecond));
                bottomShooterSpeedTreeMapMeterRadS.put(
                        distanceToHub.in(Meters), bottomSpeed.in(RadiansPerSecond));
            }
            shouldInterpolate = true;
        } catch (IOException e) {
            System.out.println(
                    "********COULD NOT READ FROM THE SHOOTER SPEEDS CSV - SEE STACK TRACE********");
            e.printStackTrace();
        } catch (CsvValidationException c) {
            System.out.println(
                    "********INVALID ENTRY IN SHOOTER SPEEDS CSV - SEE STACK TRACE********");
            c.printStackTrace();
        }
    }

    public void logShooterSpeedsToCSV(Distance distanceToHub) {
        // code modified from
        // https://www.geeksforgeeks.org/java/writing-a-csv-file-in-java-using-opencsv/
        try {
            File csvFile = new File(csvFilepath);
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

    public boolean isErrorInCSVFile() {
        return !shouldInterpolate;
    }

    @Override
    public void periodic() {
        io.updateStates(shooterStates);
        SmartDashboard.putBoolean("shooter/should interpolate", shouldInterpolate);
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
        SmartDashboard.putBoolean("shooter/isUpToSpeed", shooterStates.isUpToSpeed);
    }
}
