package frc.robot.constants;

import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    public static class AllianceZoneBoundaries {
    public static final Distance RED_ZONE_INNER_BOUNDARY_X = Inches.of(158.61);
    public static final Distance RED_ZONE_OUTER_BOUNDARY_X = Inches.of(0);
    public static final Distance BLUE_ZONE_INNER_BOUNDARY_X = Inches.of(0);
    }

    // values taken from field drawings
    public static final Pose2d RED_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56 + (2 * 143.50)), Inches.of(158.32), Rotation2d.kZero);
    public static final Pose2d BLUE_CENTER_OF_HUB_POSE =
            new Pose2d(Inches.of(181.56), Inches.of(158.32), Rotation2d.kZero);
}
