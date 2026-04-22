package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CommandFactory.TargetZoneType;
import frc.robot.constants.FieldConstants;

public class FieldCalculationHelpers {
    public static Boolean shouldTargetHub(Pose2d robotLocation) {
        double poseX = robotLocation.getX();
        if (FieldConstants.alliance.isPresent()) {
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
                if (FieldConstants.FieldBoundaries.BLUE_DRIVER_STATION_WALL_X.in(Meters) <= poseX
                        && poseX
                                <= FieldConstants.FieldBoundaries.BLUE_ZONE_BOUNDARY_X.in(Meters)) {
                    return true;
                } else {
                    return false;
                }
            }
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
                if (FieldConstants.FieldBoundaries.RED_ZONE_BOUNDARY_X.in(Meters) <= poseX
                        && poseX
                                <= FieldConstants.FieldBoundaries.RED_DRIVER_STATION_WALL_X.in(
                                        Meters)) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        return false;
    }

    public static TargetZoneType getAlignmentTargetType(Pose2d robotLocation) {
        double poseY = robotLocation.getY();
        if (shouldTargetHub(robotLocation)) {
            return TargetZoneType.HUB;
        }
        if (FieldConstants.alliance.isPresent()
                && FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
            if (FieldConstants.FieldBoundaries.HORIZONTAL_CENTER_LINE.in(Meters) <= poseY) {
                return TargetZoneType.OUTPOST_SIDE;
            } else {
                return TargetZoneType.DEPOT_SIDE;
            }
        } else {
            if (FieldConstants.FieldBoundaries.HORIZONTAL_CENTER_LINE.in(Meters) >= poseY) {
                return TargetZoneType.OUTPOST_SIDE;
            } else {
                return TargetZoneType.DEPOT_SIDE;
            }
        }
    }
}
