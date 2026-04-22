package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public final class FieldConstants {

    public static record Pose(String name, Pose2d pose) {}

    public static Optional<Alliance> alliance = Optional.empty();

    // values taken from field drawings
    public static final Distance HALF_WIDTH_OF_HUB = Inches.of(47.0 / 2.0);
    public static final Distance HALF_LENGTH_OF_ROBOT_WITH_BUMPER = Inches.of(15);
    public static final Distance ROBOT_HUB_MARGIN = Inches.of(15);
    public static final Distance HALF_WIDTH_OF_ROBOT_WITH_BUMPER = Inches.of(15);
    public static final Distance STARTING_LINE_TO_CENTER_DISTANCE = Inches.of(47 + 120);
    public static final Distance FUEL_PIT_HALF_WIDTH = Inches.of(35.95);
    public static final Distance LENGTH_OF_BUMP = Inches.of(73.00 + 12.00);
    public static final Pose2d FIELD_CENTER =
            new Pose2d(Inches.of(325.61), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d RED_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d FRONT_OF_RED_HUB =
            RED_CENTER_OF_HUB_POSE.plus(
                    new Transform2d(HALF_WIDTH_OF_HUB, Inches.of(0), Rotation2d.kZero));
    public static final Pose2d FRONT_OF_BLUE_HUB =
            BLUE_CENTER_OF_HUB_POSE.plus(
                    new Transform2d(
                            HALF_WIDTH_OF_HUB.unaryMinus(), Inches.of(0), Rotation2d.kZero));
    public static final Pose2d BACK_OF_DEPOT =
            new Pose2d(Inches.of(0), Inches.of(158.84 + 75.93), Rotation2d.k180deg);

    public static class PoseConstants {
        public static final Pose DEPOT_STARTING_POSE =
                new Pose(
                        "blueDepotStartingLine",
                        new Pose2d(
                                new Translation2d(Meters.of(3.53), Meters.of(7.51)),
                                new Rotation2d(Degrees.of(-90))));
        // this postion is when robot is on the blue alliance starting line farthest
        // from scoring
        // table.
        public static final Pose OUTPOST_STARTING_POSE =
                new Pose(
                        "blueOutpostStartingLine",
                        new Pose2d(
                                new Translation2d(Meters.of(16.54 - 12.57), Meters.of(8.07 - 7.71)),
                                Rotation2d.kCCW_90deg));
        public static final Pose DEPOT_BORDER_FUEL_CENTER =
                new Pose(
                        "blueDepotBorderFuelCenter",
                        new Pose2d(
                                new Translation2d(Meters.of(7.74), Meters.of(7)),
                                Rotation2d.k180deg));
        public static final Pose DEPOT_PASS_2_START =
                new Pose(
                        "depot pass to start",
                        new Pose2d(
                                new Translation2d(Meters.of(6.8), Meters.of(6.78)),
                                Rotation2d.k180deg));
        public static final Pose DEPOT_PASS_2_END =
                new Pose(
                        "depot pass to end",
                        new Pose2d(
                                new Translation2d(Meters.of(6.8), Meters.of(1.22)),
                                Rotation2d.k180deg));
        public static final Pose OUTPOST_PASS_2_START =
                new Pose(
                        "outpost pass to start",
                        new Pose2d(
                                new Translation2d(Meters.of(6.8), Meters.of(1.22)),
                                Rotation2d.kZero));
        public static final Pose OUTPOST_PASS_2_END =
                new Pose(
                        "outpost pass to end",
                        new Pose2d(
                                new Translation2d(Meters.of(6.8), Meters.of(6.78)),
                                Rotation2d.kZero));
        public static final Pose BLUE_OUTPOST_BORDER_FUEL_CENTER =
                new Pose(
                        "blueOutpostBorderFuelCenter",
                        new Pose2d(
                                new Translation2d(Meters.of(7.7), Meters.of(1)), Rotation2d.kZero));
        public static final Pose DEPOT_NEUTRAL_ZONE_CENTER =
                new Pose(
                        "depotNeutralZoneCenter",
                        new Pose2d(
                                new Translation2d(Meters.of(7.74), Meters.of(5.08)),
                                Rotation2d.k180deg));
        public static final Pose OUTPOST_NEUTRAL_ZONE_CENTER =
                new Pose(
                        "outpostNeutralZoneCenter",
                        new Pose2d(
                                new Translation2d(Meters.of(7.74), Meters.of(2.96)),
                                Rotation2d.kZero));
        public static final Pose DEPOT_SHOOTING_SPOT =
                new Pose(
                        "depotShootingSpot",
                        new Pose2d(
                                new Translation2d(Meters.of(3.53), Meters.of(7.35)),
                                new Rotation2d(Degrees.of(-67))));
        public static final Pose OUTPOST_SHOOTING_SPOT =
                new Pose(
                        "outpostShootingSpot",
                        new Pose2d(
                                new Translation2d(Meters.of(3.24), Meters.of(0.68)),
                                new Rotation2d(Degrees.of(70))));
        public static final Pose MIDDLE_SHOOT_POSE =
                new Pose("center shooting spot", new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));
        public static final Pose TUNING_POSE_TRIANGLE_NEAR_ALLIANCE_WALL =
                new Pose(
                        "tuning pose triangle near alliance wall",
                        new Pose2d(1.5, 6, Rotation2d.kZero));
        public static final Pose TUNING_POSE_TRIANGLE_FAR_FROM_ALLIANCE_WALL =
                new Pose(
                        "tuning pose triangle near alliance wall",
                        new Pose2d(3, 6, Rotation2d.kZero));
        public static final Pose OUTPOST_OTHER_SIDE_OF_TRENCH =
                new Pose(
                        "outpost other side of trench",
                        new Pose2d(
                                new Translation2d(Meters.of(6), Meters.of(0.75)),
                                new Rotation2d(Degrees.of(90))));
        public static final Pose DEPOT_OTHER_SIDE_OF_TRENCH =
                new Pose(
                        "depot other side of trench",
                        new Pose2d(
                                new Translation2d(Meters.of(6), Meters.of(7.39)),
                                new Rotation2d(Degrees.of(-90))));
    }

    public static class FieldBoundaries {
        public static final Distance RED_ZONE_BOUNDARY_X = Meters.of(12.56);
        public static final Distance RED_DRIVER_STATION_WALL_X = Meters.of(16.94);
        public static final Distance BLUE_ZONE_BOUNDARY_X = Meters.of(3.98);
        public static final Distance BLUE_DRIVER_STATION_WALL_X = Meters.of(0);
        public static final Distance HORIZONTAL_CENTER_LINE = Meters.of(4.034663);
    }

    public static class AlignmentTargetPoseConstants {
        public static final Distance HUB_RADIUS = Inches.of(23.5);
        public static final Distance FUEL_RADIUS = Inches.of(2.955);

        // values taken from field drawings
        public static final Pose2d RED_CENTER_OF_HUB_POSE =
                new Pose2d(Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
        public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
                new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
        public static final Pose2d BLUE_OUTPOST_ALIGN_POSE =
                new Pose2d(new Translation2d(Meters.of(3.24), Meters.of(0.68)), Rotation2d.kZero);
        public static final Pose2d RED_OUTPOST_ALIGN_POSE =
                new Pose2d(Meters.of(13), Meters.of(7.35), Rotation2d.kZero);
        public static final Pose2d BLUE_DEPOT_ALIGN_POSE =
                new Pose2d(new Translation2d(Meters.of(3.24), Meters.of(7.35)), Rotation2d.kZero);
        public static final Pose2d RED_DEPOT_ALIGN_POSE =
                new Pose2d(Meters.of(13), Meters.of(0.68), Rotation2d.kZero);
    }
}
