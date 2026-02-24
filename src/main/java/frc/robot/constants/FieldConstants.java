package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public final class FieldConstants {

    public static record Pose(String name, Pose2d pose) {}

    public static Optional<Alliance> alliance;

    public static class PoseConstants {
        public static final Pose DRIVE_STRAIGHT =
                new Pose("driveStraight", new Pose2d(0.5, 2, Rotation2d.fromDegrees(0)));
        public static final Pose HUB_MIDDLE =
                new Pose("hubMiddle", new Pose2d(3.488, 4.021, Rotation2d.fromDegrees(0)));
        public static final Pose DEPOT =
                new Pose("depot", new Pose2d(1.176, 5.958, Rotation2d.fromDegrees(0)));
        public static final Pose DEPOT_TESTING =
                new Pose("depot", new Pose2d(2.2, 5, Rotation2d.fromDegrees(0)));
        public static final Pose TOWER_L1 =
                new Pose("towerL1", new Pose2d(1.608, 3.378, Rotation2d.fromDegrees(0)));
        public static final Pose NEUTRAL_ZONE_BORDER =
                new Pose("neutralZoneBorder", new Pose2d(6.86, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose IN_NEUTRAL_ZONE =
                new Pose("inNeutralZone", new Pose2d(8.63, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose BUMP_STARTING_LINE =
                new Pose("bumpStartingLine", new Pose2d(3.482, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose OVER_THE_BUMP =
<<<<<<< HEAD
                new Pose("overTheBump", new Pose2d(5.482, 5.356, Rotation2d.fromDegrees(0)));
        // this position is when robot is up againt red hub
        public static final Pose RED_HUB_MIDDLE =
                new Pose(
                        "redHubMiddle",
                        FRONT_OF_RED_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER,
                                        Inches.of(0),
                                        Rotation2d.k180deg)));
        // this position is when robot is up againt blue hub
        public static final Pose BLUE_HUB_MIDDLE =
                new Pose(
                        "blueHubMiddle",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER.negate(),
                                        Inches.of(0),
                                        Rotation2d.k180deg)));
        // this postion is when robot is on the blue alliance starting line closes to scroing table.
        public static final Pose BLUE_LEFT_STARTING_LINE =
                new Pose(
                        "blueLeftStartingLine",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER.negate(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER),
                                        Rotation2d.kZero)));
        // this postion is when robot is on the blue alliance starting line farthest from scroing
        // table.
        public static final Pose BLUE_RIGHT_STARTING_LINE =
                new Pose(
                        "blueRightStartingLine",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER.negate(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .negate(),
                                        Rotation2d.kZero)));
        // this postion is when robot is on the red alliance starting line closes to scroing table.
        public static final Pose RED_LEFT_STARTING_LINE =
                new Pose(
                        "redLeftStartingLine",
                        FRONT_OF_RED_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER,
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .negate(),
                                        Rotation2d.kZero)));
        // this postion is when robot is on the red alliance starting line farthest from scroing
        // table.
        public static final Pose RED_RIGHT_STARTING_LINE =
                new Pose(
                        "redRightStartingLine",
                        FRONT_OF_RED_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER,
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER),
                                        Rotation2d.kZero)));
        // this position is when robot is on red side of fuel edge closest to scoring table
        public static final Pose RED_RIGHT_EDGE_FUEL =
                new Pose(
                        "redRightEdgeFuel",
                        FRONT_OF_RED_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .negate()
                                                .plus(FUEL_PIT_HALF_WIDTH),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER),
                                        Rotation2d.kZero)));
        // this position is when robot is on red side of fuel edge farthest to scoring table
        public static final Pose RED_LEFT_EDGE_FUEL =
                new Pose(
                        "redLeftEdgeFuel",
                        FRONT_OF_RED_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .negate()
                                                .plus(FUEL_PIT_HALF_WIDTH),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .negate(),
                                        Rotation2d.kZero)));
        // this position is when robot is on blue side of fuel edge closest to scoring table
        public static final Pose BLUE_RIGHT_EDGE_FUEL =
                new Pose(
                        "blueRightEdgeFuel",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .plus(FUEL_PIT_HALF_WIDTH)
                                                .negate(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .negate(),
                                        Rotation2d.k180deg)));
        // this position is when robot is on blue side of fuel edge farthest to scoring table
        public static final Pose BLUE_LEFT_EDGE_FUEL =
                new Pose(
                        "blueLeftEdgeFuel",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .plus(FUEL_PIT_HALF_WIDTH)
                                                .negate(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER),
                                        Rotation2d.k180deg)));
=======
                new Pose("overTheBump", new Pose2d(5.482, 5.356, Rotation2d.fromDegrees(180)));
>>>>>>> main
    }

    public static class AllianceZoneBoundaries {
        public static final Distance RED_ZONE_BOUNDARY_X = Meters.of(12.56);
        public static final Distance RED_DRIVER_STATION_WALL_X = Meters.of(16.94);
        public static final Distance BLUE_ZONE_BOUNDARY_X = Meters.of(3.98);
        public static final Distance BLUE_DRIVER_STATION_WALL_X = Meters.of(0);
    }

    // values taken from field drawings
    public static final Distance HALF_WIDTH_OF_HUB = Inches.of(47.0 / 2.0);
    public static final Distance HALF_LENGTH_OF_ROBOT_WITH_BUMPER = Inches.of(15);
    public static final Distance ROBOT_HUB_MARGIN = Inches.of(15);
    public static final Distance HALF_WIDTH_OF_ROBOT_WITH_BUMPER = Inches.of(15);
    public static final Distance STARTING_LINE_TO_CENTER_DISTANCE = Inches.of(47 + 120);
    public static final Distance FUEL_PIT_HALF_WIDTH = Inches.of(35.95);
    public static final Pose2d RED_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d FRONT_OF_RED_HUB =
            RED_CENTER_OF_HUB_POSE.plus(
                    new Transform2d(HALF_WIDTH_OF_HUB, Inches.of(0), Rotation2d.kZero));
    public static final Pose2d FRONT_OF_BLUE_HUB =
            BLUE_CENTER_OF_HUB_POSE.plus(
                    new Transform2d(HALF_WIDTH_OF_HUB.negate(), Inches.of(0), Rotation2d.kZero));
}
