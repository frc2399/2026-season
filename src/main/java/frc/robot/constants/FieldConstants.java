package frc.robot.constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    public static record Pose(String name, Pose2d pose) {}
    public static class PoseConstants {
      public static final Pose HUB_MIDDLE = new Pose("hubMiddle", new Pose2d(3.488, 4.021, Rotation2d.fromDegrees(0)));
      public static final Pose DEPOT = new Pose("depot", new Pose2d(1.176, 5.958, Rotation2d.fromDegrees(180)));
      public static final Pose TOWER_L1 = new Pose("towerL1", new Pose2d(1.608, 3.378, Rotation2d.fromDegrees(0)));
      public static final Pose NEUTRAL_ZONE_BORDER = new Pose("neutralZoneBorder", new Pose2d(6.86, 5.356, Rotation2d.fromDegrees(180)));
      public static final Pose BUMP_STARTING_LINE = new Pose("bumpStartingLine", new Pose2d(3.482, 5.356, Rotation2d.fromDegrees(180)));
  }
}
