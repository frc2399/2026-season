package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.HubConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.TransformConstants;
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
            return FieldConstants.HubConstants.BLUE_CENTER_OF_HUB_POSE;
        } else {
            return FieldConstants.HubConstants.RED_CENTER_OF_HUB_POSE;
        }
    }

    public static double getDesiredAngleToHub(Supplier<Pose2d> robotPose) {

        Pose2d orientTargetPose = RebuiltVisionUtil.getHubPose();
        Transform2d robotToShooterTransform = TransformConstants.ROBOT_TO_SHOOTER_TRANSFORM;

        Translation2d targetToRobotTranslation =
                orientTargetPose.getTranslation().minus(robotPose.get().transformBy(robotToShooterTransform).getTranslation());
        Angle desiredAngle =
                Radians.of(
                        Math.atan2(
                                targetToRobotTranslation.getY(), targetToRobotTranslation.getX()));
        double desiredAngleInDegrees = desiredAngle.in(Degrees);
        return desiredAngleInDegrees;
    }

    public static double getActualAngleToHub(Supplier<Pose2d> robotPose) {
        double actualAngleInDegrees = robotPose.get().getRotation().getDegrees();
        return actualAngleInDegrees;
    }

    public static double getDiffOfAngleInDegrees(
            double actualAngleInDegrees, double desiredAngleInDegrees) {
        double diffOfAngleInDegrees = actualAngleInDegrees - desiredAngleInDegrees;
        return diffOfAngleInDegrees;
    }

    public double getRangeOfAngleToHubInDegrees(
            double robotDistanceToHub, Pose2d orientTargetPose, Pose2d poseToOrientToTarget) {
        double hubRadiusMinusFuelRadiusInMeters =
                HubConstants.HUB_RADIUS.minus(HubConstants.FUEL_RADIUS).in(Meters);

        Translation2d targetToRobotTranslation =
                orientTargetPose.getTranslation().minus(poseToOrientToTarget.getTranslation());
        double hubToRobotDistance = targetToRobotTranslation.getNorm();
        double rangeOfAngleToHubInDegrees =
                Math.atan2(hubRadiusMinusFuelRadiusInMeters, hubToRobotDistance);
        return rangeOfAngleToHubInDegrees;
    }

    public static boolean isShootingAngleAlignedToHub(Supplier<Pose2d> robotPose) {

        double robotAngleToHubInDegrees = getDesiredAngleToHub(robotPose);
        double diffOfAngleInDegrees =
                getDiffOfAngleInDegrees(
                        getActualAngleToHub(robotPose), getDesiredAngleToHub(robotPose));

        if (Math.abs(diffOfAngleInDegrees) <= (Math.abs(robotAngleToHubInDegrees))) {
            return true;
        } else {
            return false;
        }
    }
}
