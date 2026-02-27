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
        public static final Pose HUB_MIDDLE =
                new Pose("hubMiddle", new Pose2d(3.488, 4.021, Rotation2d.fromDegrees(180)));
        public static final Pose DEPOT =
                new Pose("depot", new Pose2d(1.176, 5.958, Rotation2d.fromDegrees(0)));
        public static final Pose DEPOT_TESTING =
                new Pose("depot", new Pose2d(2.2, 5, Rotation2d.fromDegrees(0)));
        public static final Pose TOWER_L1 =
                new Pose("towerL1", new Pose2d(1.608, 3.378, Rotation2d.fromDegrees(0)));
        public static final Pose DEPOT_SIDE_NEUTRAL_ZONE_BORDER =
                new Pose(
                        "depotSideNeutralZoneBorder",
                        new Pose2d(6.86, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose DEPOT_SIDE_IN_NEUTRAL_ZONE =
                new Pose(
                        "depotSideInNeutralZone",
                        new Pose2d(8.63, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose DEPOT_SIDE_BUMP_STARTING_LINE =
                new Pose(
                        "depotSideBumpStartingLine",
                        new Pose2d(3.482, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose DEPOT_SIDE_OVER_THE_BUMP =
                new Pose(
                        "depotSideOverTheBump",
                        new Pose2d(5.75, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose DEPOT_SIDE_NEUTRAL_ZONE_CENTER =
                new Pose(
                        "depotSideNeutralZoneCenter",
                        new Pose2d(7.7, 5.356, Rotation2d.fromDegrees(180)));
        public static final Pose DEPOT_END_NEUTRAL_ZONE =
                new Pose("depotEndNeutralZone", new Pose2d(7.7, 3.07, Rotation2d.fromDegrees(270)));
        public static final Pose DEPOT_END_NEUTRAL_ZONE_BORDER =
                new Pose(
                        "depotEndNeutralZoneBorder",
                        new Pose2d(7.4, 3.07, Rotation2d.fromDegrees(0)));
        public static final Pose DEPOT_INTAKE_END =
                new Pose("depotIntakeEnd", new Pose2d(7.4, 5.356, Rotation2d.fromDegrees(90)));
        public static final Pose OUTPOST_SIDE_NEUTRAL_ZONE_BORDER =
                new Pose(
                        "outpostSideNeutralZoneBorder",
                        new Pose2d(6.86, 2.7, Rotation2d.fromDegrees(180)));
        public static final Pose OUTPOST_SIDE_IN_NEUTRAL_ZONE =
                new Pose(
                        "outpostSideInNeutralZone",
                        new Pose2d(8.63, 2.7, Rotation2d.fromDegrees(180)));
        public static final Pose OUTPOST_SIDE_BUMP_STARTING_LINE =
                new Pose(
                        "outposeSideBumpStartingLine",
                        new Pose2d(3.482, 2.7, Rotation2d.fromDegrees(180)));
        public static final Pose OUTPOST_SIDE_OVER_THE_BUMP =
                new Pose(
                        "outposeSideOverTheBump",
                        new Pose2d(5.75, 2.7, Rotation2d.fromDegrees(180)));
        public static final Pose OUTPOST_SIDE_NEUTRAL_ZONE_CENTER =
                new Pose(
                        "outpostSideNeutralZoneCenter",
                        new Pose2d(7.7, 2.7, Rotation2d.fromDegrees(180)));
        public static final Pose OUTPOST_END_NEUTRAL_ZONE =
                new Pose("outpostEndNeutralZone", new Pose2d(7.7, 5.8, Rotation2d.fromDegrees(90)));
        public static final Pose OUTPOST_END_NEUTRAL_ZONE_BORDER =
                new Pose(
                        "outpostEndNeutralZoneBorder",
                        new Pose2d(7.4, 5.1, Rotation2d.fromDegrees(0)));
        public static final Pose OUTPOST_INTAKE_END =
                new Pose("outpostIntakeEnd", new Pose2d(7.4, 2.7, Rotation2d.fromDegrees(90)));

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
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER.unaryMinus(),
                                        Inches.of(0),
                                        Rotation2d.k180deg)));
        // this postion is when robot is on the blue alliance starting line closes to scroing table.
        public static final Pose BLUE_DEPOT_STARTING_LINE =
                new Pose(
                        "blueDepotStartingLine",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER.unaryMinus(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN.divide(2)),
                                        Rotation2d.kZero)));
        // this postion is when robot is on the blue alliance starting line farthest from scroing
        // table.
        public static final Pose BLUE_OUTPOST_STARTING_LINE =
                new Pose(
                        "blueOutpostStartingLine",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER.unaryMinus(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN.divide(2))
                                                .unaryMinus(),
                                        Rotation2d.kZero)));
        public static final Pose BLUE_OUTPOST_WALL_FUEL_CENTER =
                new Pose(
                        "blueOutpostWallFuelCenter",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .minus(FUEL_PIT_HALF_WIDTH)
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN.divide(2))
                                                .unaryMinus(),
                                        Rotation2d.k180deg)));
        // this position is when robot is on blue side of fuel edge farthest to scoring table
        public static final Pose BLUE_DEPOT_WALL_FUEL_CENTER =
                new Pose(
                        "blueDepotWallFuelCenter",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .minus(FUEL_PIT_HALF_WIDTH)
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN.divide(2)),
                                        Rotation2d.k180deg)));
        public static final Pose BLUE_DEPOT_BORDER_FUEL_CENTER =
                new Pose(
                        "blueDepotBorderFuelCenter",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .minus(FUEL_PIT_HALF_WIDTH)
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER),
                                        HALF_WIDTH_OF_HUB.plus(LENGTH_OF_BUMP),
                                        Rotation2d.k180deg)));
        public static final Pose BLUE_OUTPOST_BORDER_FUEL_CENTER =
                new Pose(
                        "blueOutpostBorderFuelCenter",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE
                                                .minus(FUEL_PIT_HALF_WIDTH)
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER),
                                        HALF_WIDTH_OF_HUB.plus(LENGTH_OF_BUMP).unaryMinus(),
                                        Rotation2d.k180deg)));
        public static final Pose BLUE_OUTPOST_BORDER_FUEL_EDGE =
                new Pose(
                        "blueOutpostBorderFuelEdge",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE.minus(FUEL_PIT_HALF_WIDTH),
                                        HALF_WIDTH_OF_HUB.plus(LENGTH_OF_BUMP).unaryMinus(),
                                        Rotation2d.k180deg)));
        public static final Pose BLUE_OUTPOST_WALL_FUEL_EDGE =
                new Pose(
                        "blueOutpostWallFuelEdge",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE.minus(FUEL_PIT_HALF_WIDTH),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN.divide(2))
                                                .unaryMinus(),
                                        Rotation2d.k180deg)));
        public static final Pose BLUE_DEPOT_BORDER_FUEL_EDGE =
                new Pose(
                        "blueDepotBorderFuelEdge",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE.minus(FUEL_PIT_HALF_WIDTH),
                                        HALF_WIDTH_OF_HUB.plus(LENGTH_OF_BUMP),
                                        Rotation2d.k180deg)));
        public static final Pose BLUE_DEPOT_WALL_FUEL_EDGE =
                new Pose(
                        "blueDepotWallFuelEdge",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        STARTING_LINE_TO_CENTER_DISTANCE.minus(FUEL_PIT_HALF_WIDTH),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN.divide(2)),
                                        Rotation2d.k180deg)));
        public static final Pose IN_THE_DEPOT =
                new Pose(
                        "inTheDepot",
                        BACK_OF_DEPOT.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER
                                                .plus(Inches.of(5))
                                                .unaryMinus(),
                                        Inches.of(0),
                                        Rotation2d.k180deg)));
        public static final Pose DEPOT_NEUTRAL_ZONE_CENTER =
                new Pose(
                        "depotNeutralZoneCenter",
                        FIELD_CENTER.plus(
                                new Transform2d(
                                        FUEL_PIT_HALF_WIDTH
                                                .unaryMinus()
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER),
                                        Rotation2d.kCCW_90deg)));
        public static final Pose DEPOT_IN_NEUTRAL_ZONE =
                new Pose(
                        "depotInNeutralZone",
                        FIELD_CENTER.plus(
                                new Transform2d(
                                        ROBOT_HUB_MARGIN
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER)
                                                .unaryMinus(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER),
                                        Rotation2d.kZero)));
        public static final Pose OUTPOST_IN_NEUTRAL_ZONE =
                new Pose(
                        "outpostInNeutralZone",
                        FIELD_CENTER.plus(
                                new Transform2d(
                                        ROBOT_HUB_MARGIN
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER)
                                                .unaryMinus(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .unaryMinus(),
                                        Rotation2d.kZero)));
        public static final Pose OUTPOST_NEUTRAL_ZONE_CENTER =
                new Pose(
                        "outpostNeutralZoneCenter",
                        FIELD_CENTER.plus(
                                new Transform2d(
                                        FUEL_PIT_HALF_WIDTH
                                                .unaryMinus()
                                                .plus(HALF_LENGTH_OF_ROBOT_WITH_BUMPER),
                                        HALF_WIDTH_OF_HUB
                                                .plus(ROBOT_HUB_MARGIN)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .unaryMinus(),
                                        Rotation2d.kZero)));
        public static final Pose PRELOADED_SHOOTING_SPOT =
                new Pose(
                        "preloadedShootingSpot",
                        FRONT_OF_BLUE_HUB.plus(
                                new Transform2d(
                                        HALF_LENGTH_OF_ROBOT_WITH_BUMPER
                                                .plus(ROBOT_HUB_MARGIN)
                                                .unaryMinus(),
                                        HALF_WIDTH_OF_HUB
                                                .plus(LENGTH_OF_BUMP)
                                                .plus(HALF_WIDTH_OF_ROBOT_WITH_BUMPER)
                                                .plus(ROBOT_HUB_MARGIN),
                                        Rotation2d.kZero)));
    }

    public static class AllianceZoneBoundaries {
        public static final Distance RED_ZONE_BOUNDARY_X = Meters.of(12.56);
        public static final Distance RED_DRIVER_STATION_WALL_X = Meters.of(16.94);
        public static final Distance BLUE_ZONE_BOUNDARY_X = Meters.of(3.98);
        public static final Distance BLUE_DRIVER_STATION_WALL_X = Meters.of(0);
    }
}
