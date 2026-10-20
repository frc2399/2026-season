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

    public static record Pose(String name, Pose2d pose) {
        public static Pose of(String name, double x, double y, double degrees) {
            return new Pose(name, new Pose2d(x, y, Rotation2d.fromDegrees(degrees)));
        }
    }

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
                Pose.of("blueDepotStartingLine", 3.53, 7.51, -90);
        // this postion is when robot is on the blue alliance starting line farthest
        // from scoring
        // table.
        public static final Pose OUTPOST_STARTING_POSE =
                Pose.of("blueOutpostStartingLine", (16.54 - 12.57), (8.07 - 7.71), 90);
        public static final Pose DEPOT_BORDER_FUEL_CENTER =
                Pose.of("blueDepotBorderFuelCenter", 7.74, 7, 180);
        public static final Pose DEPOT_PASS_2_START =
                Pose.of("depot pass to start", 6.8, 6.78, 180);
        public static final Pose DEPOT_PASS_2_END = Pose.of("depot pass to end", 6.8, 1.22, 180);
        public static final Pose OUTPOST_PASS_2_START =
                Pose.of("outpost pass to start", 6.8, 1.22, 0);
        public static final Pose OUTPOST_PASS_2_END = Pose.of("outpost pass to end", 6.8, 6.78, 0);
        public static final Pose BLUE_OUTPOST_BORDER_FUEL_CENTER =
                Pose.of("blueOutpostBorderFuelCenter", 7.7, 1, 0);
        public static final Pose DEPOT_NEUTRAL_ZONE_CENTER =
                Pose.of("depotNeutralZoneCenter", 7.74, 5.08, 180);
        public static final Pose OUTPOST_NEUTRAL_ZONE_CENTER =
                Pose.of("outpostNeutralZoneCenter", 7.74, 2.96, 0);
        public static final Pose DEPOT_SHOOTING_SPOT =
                Pose.of("depotShootingSpot", 3.53, 7.35, -72);
        public static final Pose OUTPOST_SHOOTING_SPOT =
                Pose.of("outpostShootingSpot", 3.24, 0.68, 67);
        public static final Pose MIDDLE_SHOOT_POSE = Pose.of("center shooting spot", 2.5, 4, 0);
        public static final Pose OUTPOST_OTHER_SIDE_OF_TRENCH =
                Pose.of("outpost other side of trench", 6, 0.75, 90);
        public static final Pose DEPOT_OTHER_SIDE_OF_TRENCH =
                Pose.of("depot other side of trench", 6, 7.39, -90);
        public static final Pose DEPOT_OTHER_SIDE_OF_TRENCH_FIRST =
                Pose.of("depot other side of trench first", 6, 7.25, -90);
        public static final Pose OUTPOST_OTHER_SIDE_OF_TRENCH_FIRST =
                Pose.of("outpost other side of trench first", 6, 0.68, 90);
        public static final Pose DEPOT_EDGE = Pose.of("depot edge", 1.37, 4.9, 0);
        public static final Pose DEPOT_EDGE_NEW = Pose.of("depot edge new", 1.37, 7.3, 180);
        public static final Pose DEPOT_SIDE_INTAKE = Pose.of("depot intake start", 0.42, 4.9, 0);
        public static final Pose DEPOT_SIDE_INTAKE_END_NEW =
                Pose.of("depot intake end new", 0.42, 4.9, 180);
        public static final Pose DEPOT_SIDE_INTAKE_END = Pose.of("depot intake end", 0.42, 7, 0);
        public static final Pose DEPOT_SIDE_INTAKE_NEW =
                Pose.of("depot intake new", 0.42, 7.6, 180);
        public static final Pose DEPOT_INTAKE_FAR = Pose.of("depot intake end", 0.42, 7.6, 0);
        public static final Pose DEPOT_CORNER = Pose.of("depot intake corner", 2.1, 7.4, 0);
        public static final Pose DEPOT_WALL_INTAKE = Pose.of("depot wall intake", 2, 7.52, 90);
        public static final Pose DEPOT_CORNER_WALL_INTAKE =
                Pose.of("depot wall intake", 0.7, 7.52, 90);
        public static final Pose DEPOT_CORNER_STRAIGHT = Pose.of("depot intake corner", 1, 6.86, 0);
        public static final Pose DEPOT_CORNER_INTAKE_TURNED =
                Pose.of("depot intake turned in corner", 0.53, 7.31, 90);
        public static final Pose DEPOT_CORNER_INTAKE_STRAIGHT =
                Pose.of("depot intake end", 0.755, 7.6, 0);
        public static final Pose DEPOT_MIDDLE = Pose.of("depot middle", 0.6, 6.75, 24.5);
        public static final Pose CENTER_MOVE_TO_SHOOTING_SPOT =
                Pose.of("depot moving to shooting spot", 1.37, 4.9, -26);
        public static final Pose CENTER_SHOOTING_SPOT = Pose.of("depot to shoot", 2.686, 5.48, -36);
        public static final Pose CENTER_SHOOTING_SPOT_DEEP =
                Pose.of("depot to shoot deep", 1.686, 5.48, -31);
        public static final Pose OUTPOST_MIDDLE_HUB_NEUTRAL_ZONE =
                Pose.of("outpost middle hub neutral zone", 6.5, 3.25, 90);
        public static final Pose OUTPOST_CORNER_HUB_NEUTRAL_ZONE =
                Pose.of("outpost corner hub neutral zone", 5.75, 2.5, 180);
        public static final Pose DEPOT_MIDDLE_HUB_NEUTRAL_ZONE =
                Pose.of("depot middle hub neutral zone", 6.5, 4.5, 90);
        public static final Pose DEPOT_CORNER_HUB_NEUTRAL_ZONE =
                Pose.of("depot corner hub neutral zone", 5.75, 5.75, 0);

        public static class SecondPassScoopPoseConstants {
            public static final Pose DEPOT_PAST_TRENCH_ROTATION_SCOOP_2 =
                    Pose.of("depot past trench rotation scoop 2",6,6.79,180);
            public static final Pose DEPOT_CORNER_HUB_SCOOP_2 =
                    Pose.of("depot corner hub scoop 2",5.75,5.75,180);
            public static final Pose DEPOT_MIDDLE_HUB_SCOOP_2 =
                    Pose.of("depot middle hub scoop 2", 6.5, 4.5, -90);
            public static final Pose DEPOT_NEUTRAL_ZONE_CENTER_SCOOP_2 =
                    Pose.of("depot neutral zone center scoop 2", 7.74, 5.08, 0);
            public static final Pose DEPOT_BORDER_FUEL_CENTER_SCOOP_2 =
                    Pose.of( "depot border fuel center scoop 2", 7.74, 5.08,0);
            public static final Pose OUTPOST_PAST_TRENCH_ROTATION_SCOOP_2 =
                    Pose.of( "outpost past trench rotation scoop 2", 6, 1.35, 0);
            public static final Pose OUTPOST_CORNER_HUB_SCOOP_2 =
                    Pose.of( "outpost corner hub scoop 2",5.75, 2.5,0);
            public static final Pose OUTPOST_MIDDLE_HUB_SCOOP_2 =
                    Pose.of( "outpost middle hub scoop 2",6.5,3.25,-90);
            public static final Pose OUTPOST_NEUTRAL_ZONE_CENTER_SCOOP_2 =
                    Pose.of("outpost neutral zone center scoop 2",7.74,2.96,180);
            public static final Pose OUTPOST_BORDER_FUEL_CENTER_SCOOP_2 =
                    Pose.of("outpost border fuel center scoop 2",7.7,1,180);
        }
    }

    public static class FieldBoundaries {
        public static final Distance RED_ZONE_BOUNDARY_X = Inches.of(181.56 + (2 * 143.50));
        public static final Distance RED_DRIVER_STATION_WALL_X = Meters.of(16.94);
        public static final Distance BLUE_ZONE_BOUNDARY_X = Inches.of(181.56);
        public static final Distance BLUE_DRIVER_STATION_WALL_X = Meters.of(0);
        public static final Distance SCREEN_LINE =
                Inches.of(204); // screen 17 ft from scoring table (measured by fta)
        public static final Distance HORIZONTAL_CENTER_LINE = Meters.of(4.034663);
        public static final Distance SCREEN_RANGE = Meters.of(0);
    }

    public static class HubConstants {
        public static class AlignmentTargetPoseConstants {
            public static final Distance HUB_RADIUS = Inches.of(23.5);
            public static final Distance FUEL_RADIUS = Inches.of(2.955);

            // values taken from field drawings
            public static final Pose2d RED_CENTER_OF_HUB_POSE =
                    new Pose2d(
                            Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
            public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
                    new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
            public static final Pose2d BLUE_OUTPOST_ALIGN_POSE =
                    new Pose2d(new Translation2d(Meters.of(3.24), Meters.of(2)), Rotation2d.kZero);
            public static final Pose2d RED_OUTPOST_ALIGN_POSE =
                    new Pose2d(Meters.of(13), Meters.of(6), Rotation2d.kZero);
            public static final Pose2d BLUE_DEPOT_ALIGN_POSE =
                    new Pose2d(new Translation2d(Meters.of(3.24), Meters.of(6)), Rotation2d.kZero);
            public static final Pose2d RED_DEPOT_ALIGN_POSE =
                    new Pose2d(Meters.of(13), Meters.of(2), Rotation2d.kZero);
        }
    }
}
