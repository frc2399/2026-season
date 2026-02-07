package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RebuiltVisionUtil {

    public static Supplier<Pose2d> getGoalPose(
            Supplier<Pose2d> robotPose, BooleanSupplier isBlueAlliance) {
        Pose2d returnPose = new Pose2d();
        return () -> returnPose;
    }
}
