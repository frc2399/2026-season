package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public final class FieldConstants {

    public static record Pose(String name, Pose2d pose) {}

    public static Optional<Alliance> alliance;

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
    }

    public static class AllianceZoneBoundaries {
        public static final Distance RED_ZONE_BOUNDARY_X = Meters.of(12.56);
        public static final Distance RED_DRIVER_STATION_WALL_X = Meters.of(16.94);
        public static final Distance BLUE_ZONE_BOUNDARY_X = Meters.of(3.98);
        public static final Distance BLUE_DRIVER_STATION_WALL_X = Meters.of(0);
    }

    // values taken from field drawings
    public static final Pose2d RED_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
}
