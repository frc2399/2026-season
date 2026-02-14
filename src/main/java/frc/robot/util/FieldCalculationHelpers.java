package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

import static edu.wpi.first.units.Units.Meters;

public class  FieldCalculationHelpers {
      public Boolean shouldTargetHub(Pose2d robotLocation) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        double poseX = robotLocation.getX();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
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
            if (alliance.get() == DriverStation.Alliance.Red) {
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
