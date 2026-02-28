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

    // if return true then shoot right and if false shoot left
    public static Boolean shouldRobotPassLeftOrRight(Pose2d robotLocation) {
        double poseY = robotLocation.getY();
        if (FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
            if (FieldConstants.FieldBoundaries.CENTER_LINE.in(Meters) <= poseY) {
                return true;
            } else {
                return false;
            }
        } else {
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
                if (FieldConstants.FieldBoundaries.CENTER_LINE.in(Meters) >= poseY) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        return null;
    }
}
