package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CommandFactory.TargetZoneType;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;

public class FieldCalculationHelpers {
    public static Boolean shouldTargetHub(Supplier<Pose2d> robotLocation) {
        double poseX = robotLocation.get().getX();
        if (FieldConstants.alliance.isPresent()) {
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
                if (FieldConstants.FieldBoundaries.BLUE_DRIVER_STATION_WALL_X.in(Meters) <= poseX) {
                    return true;
                } else {
                    return false;
                }
            }
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
                if (FieldConstants.FieldBoundaries.RED_ZONE_BOUNDARY_X.in(Meters) <= poseX) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        return false;
    }

    public static boolean amInDangerZone(Supplier<Pose2d> robotLocation) {
        if (shouldTargetHub(robotLocation)) return false;
        double y = robotLocation.get().getY();
        double yScreenLine = FieldConstants.FieldBoundaries.SCREEN_LINE.in(Meters);
        if (Math.abs(y - yScreenLine) < FieldConstants.FieldBoundaries.SCREEN_RANGE.in(Meters))
            return true;
        return false;
    }

    public static TargetZoneType getAlignmentTargetType(Supplier<Pose2d> robotLocation) {
        double poseY = robotLocation.get().getY();
        if (shouldTargetHub(robotLocation)) {
            return TargetZoneType.HUB;
        }
        if (FieldConstants.alliance.isPresent()
                && FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
            if (FieldConstants.FieldBoundaries.SCREEN_LINE.in(Meters) <= poseY) {

                return TargetZoneType.DEPOT_SIDE;
            } else {

                return TargetZoneType.OUTPOST_SIDE;
            }
        } else {
            if (FieldConstants.FieldBoundaries.SCREEN_LINE.in(Meters) >= poseY) {

                return TargetZoneType.DEPOT_SIDE;
            } else {

                return TargetZoneType.OUTPOST_SIDE;
            }
        }
    }
}
