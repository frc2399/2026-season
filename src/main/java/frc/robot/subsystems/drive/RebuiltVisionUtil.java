package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RebuiltVisionUtil {

    public static Supplier<Pose2d> getGoalPose(
            Supplier<Pose2d> robotPose, BooleanSupplier isBlueAlliance) {
        Pose2d returnPose = new Pose2d();
        return () -> returnPose;
    }

    public static Distance getDistanceToHub(Supplier<Pose2d> robotPose) {
        Pose2d hubPose = getHubPose();
        if (robotPose.get() == null) {
            return Inches.of(0);
        }
        Pose2d shooterPose =
                robotPose
                        .get()
                        .transformBy(RobotConstants.TransformConstants.ROBOT_TO_SHOOTER_TRANSFORM);
        double distanceBetweenRobotAndHub =
                hubPose.getTranslation().getDistance(shooterPose.getTranslation());
        return Meters.of(distanceBetweenRobotAndHub);
    }

    public static Pose2d getHubPose() {
        if (FieldConstants.alliance.isPresent() && FieldConstants.alliance.get() == Alliance.Blue) {
            return FieldConstants.BLUE_CENTER_OF_HUB_POSE;
        } else {
            return FieldConstants.RED_CENTER_OF_HUB_POSE;
        }
    }
}
