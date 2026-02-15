package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
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
            return FieldConstants.HubConstants.BLUE_CENTER_OF_HUB_POSE;
        } else {
            return FieldConstants.HubConstants.RED_CENTER_OF_HUB_POSE;
        }
    }

     public static boolean isShootingAngleAlignedToHub(Pose2d robotPose2d) {
        Angle robotAngleToHub = ;
        


        if Math.abs ()
        return true;
        else
        return false;
    }
}
