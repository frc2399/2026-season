package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class RebuiltVisionUtil {
    
    public static Supplier<Pose2d> getGoalPose(Supplier<Pose2d> robotPose, BooleanSupplier isBlueAlliance) {
        Pose2d returnPose = new Pose2d();
        return () -> returnPose;
    }

    public static Pose2d getHubPose() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return FieldConstants.BLUE_HUB_POSE;
        } else {
            return FieldConstants.RED_HUB_POSE;
        }
    }
}
