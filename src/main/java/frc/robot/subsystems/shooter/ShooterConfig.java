package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.SubsystemFactory;
import frc.robot.constants.RobotConstants;

public class ShooterConfig {
    public static double BOTTOM_KS;
    public static double BOTTOM_KV;
    public static double BOTTOM_P;
    public static double TOP_KS;
    public static double TOP_KV;
    public static double TOP_P;
    public static boolean BOTTOM_ROLLER_INVERTED;

    static {
        switch (SubsystemFactory.getRobotType()) {
            case BETA:
                BOTTOM_KS = 0.123;
                BOTTOM_KV = 0.01697538;
                BOTTOM_P = 0.003;
                TOP_KS = 0.16;
                TOP_KV = 12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
                TOP_P = 0.00008;
                BOTTOM_ROLLER_INVERTED = false;
                break;
            case COMP:
                BOTTOM_KS = .122;
                BOTTOM_KV = 0.0171;
                BOTTOM_P = 0.003;
                TOP_KS = .1;
                TOP_KV = 0.017;
                TOP_P = 0.000005;
                BOTTOM_ROLLER_INVERTED = true;
                break;
            default:
                BOTTOM_KS = 0;
                BOTTOM_KV = 0;
                BOTTOM_P = 0;
                TOP_KS = 0;
                TOP_KV = 0;
                TOP_P = 0;
                BOTTOM_ROLLER_INVERTED = false;
        }
    }
}
