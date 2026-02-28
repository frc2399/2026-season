package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.FieldConstants;

public class FieldCalculationHelpers {
    public static Boolean shouldTargetHub(Pose2d robotLocation) {
        double poseX = robotLocation.getX();
        if (FieldConstants.alliance.isPresent()) {
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
                if (FieldConstants.AllianceZoneBoundaries.BLUE_DRIVER_STATION_WALL_X.in(Meters)
                                <= poseX
                        && poseX
                                <= FieldConstants.AllianceZoneBoundaries.BLUE_ZONE_BOUNDARY_X.in(
                                        Meters)) {
                    return true;
                } else {
                    return false;
                }
            }
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
                if (FieldConstants.AllianceZoneBoundaries.RED_ZONE_BOUNDARY_X.in(Meters) <= poseX
                        && poseX
                                <= FieldConstants.AllianceZoneBoundaries.RED_DRIVER_STATION_WALL_X
                                        .in(Meters)) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        return false;
    }
}
