package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RebuiltVisionUtil {

    public static Supplier<Pose2d> getGoalPose(
            Supplier<Pose2d> robotPose, BooleanSupplier isBlueAlliance) {
        Pose2d returnPose = new Pose2d();
        return () -> returnPose;
    }

    public static Pose2d getHubPose() {
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue) {
            return FieldConstants.BLUE_CENTER_OF_HUB_POSE;
        } else {
            return FieldConstants.RED_CENTER_OF_HUB_POSE;
        }
    }

    public static Distance getDistanceToHub(Supplier<Pose2d> robotPose) {
        Pose2d hubPose = getHubPose();
        if (robotPose.get() == null) {
            return Inches.of(0);
        }
        double distanceBetweenRobotAndHub = hubPose.getTranslation().getDistance(robotPose.get().getTranslation());
        return Meters.of(distanceBetweenRobotAndHub);
    }
}
