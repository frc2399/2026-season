package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    public static record Pose(String name, Pose2d pose) {}
    public static class PoseConstants {
        public static final Pose DRIVE_STRAIGHT = new Pose("driveStraight", new Pose2d(5.1, 2, Rotation2d.fromDegrees(0)));
        public static final Pose HUB_MIDDLE = new Pose("hubMiddle", new Pose2d(12.583, 4.427, Rotation2d.fromDegrees(90)));
        public static final Pose DEPOT = new Pose("depot", new Pose2d(15.95, 2.0574, Rotation2d.fromDegrees(270)));
        public static final Pose TOWER_L1 = new Pose("towerL1", new Pose2d(15.43, 4.229, Rotation2d.fromDegrees(270)));
        public static final Pose NEUTRAL_ZONE_BORDER = new Pose("neutralZoneBorder", new Pose2d(x, y, Rotation2d.fromDegrees(90)));
        public static final Pose BUMP_STARTING_LINE = new Pose("bumpStartingLine", new Pose2d(x, y, Rotation2d.fromDegrees(90)));
  }
    
}