package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class ShooterHardwareBeta implements ShooterIO {
    private SparkFlex shooterBottomSparkFlex;
    private SparkFlex shooterTopSparkFlex;
    private SparkClosedLoopController shooterBottomPIDController;
    private SparkClosedLoopController shooterTopPIDController;

    private static final double topRollerCurveFitSlope =
            1474.38483; // from desmos (natural log fit)
    private static final double topRollerCurveFitIntercept =
            -2995.98209; // also from desmos (natural log fit)
    private static final Distance bottomRollerCurveBoundary = Inches.of(32);
    private static final double bottomRollerCloseToHubCurveSlope =
            -107.14286; // all of these are from desmos, linear piecewise fit)
    private static final double bottomRollerCloseToHubCurveIntercept = 5478.57143;
    private static final double bottomRollerFarFromHubCurveSlope = 7.58432;
    private static final double bottomRollerFarFromHubCurveIntercept = 1860.20103;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final double MIN_OUTPUT_RANGE = -1;
    private final double MAX_OUTPUT_RANGE = 1;
    private final double SHOOTER_TOP_P = 0;
    private final double SHOOTER_TOP_D = 0;
    private final double SHOOTER_TOP_KS = 0.01;
    private final double SHOOTER_TOP_KV = 0.01;
    // 12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
    private final double SHOOTER_BOTTOM_P = 0;
    private final double SHOOTER_BOTTOM_D = 0;
    private final double SHOOTER_BOTTOM_KS = 0.01;
    private final double SHOOTER_BOTTOM_KV = 0.01;
    // 12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);

    private final ClosedLoopConfig closedLoopConfigShooterTop = new ClosedLoopConfig();
    private final ClosedLoopConfig closedLoopConfigShooterBottom = new ClosedLoopConfig();

    private RelativeEncoder shooterBottomEncoder;
    private RelativeEncoder shooterTopEncoder;

    private AngularVelocity desiredBottomVelocity = RadiansPerSecond.of(0);
    private AngularVelocity desiredTopVelocity = RadiansPerSecond.of(0);

    public ShooterHardwareBeta() {
        SparkFlexConfig shooterBottomMotorConfig = new SparkFlexConfig();
        SparkFlexConfig shooterTopMotorConfig = new SparkFlexConfig();

        shooterBottomMotorConfig.idleMode(IdleMode.kCoast);
        shooterTopMotorConfig.idleMode(IdleMode.kCoast);
        shooterBottomMotorConfig.inverted(true);
        shooterTopMotorConfig.inverted(true);
        shooterBottomMotorConfig.smartCurrentLimit(
                (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        shooterTopMotorConfig.smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        shooterBottomMotorConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        shooterTopMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));

        shooterBottomMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));
        shooterTopMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        shooterBottomMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterTopMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterBottomMotorConfig.closedLoop.pid(SHOOTER_BOTTOM_P, 0, SHOOTER_BOTTOM_D);
        shooterTopMotorConfig.closedLoop.pid(SHOOTER_TOP_P, 0, SHOOTER_TOP_D);
        shooterBottomMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
        shooterTopMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        closedLoopConfigShooterTop.feedForward.sva(SHOOTER_TOP_KS, SHOOTER_TOP_KV, 0);
        closedLoopConfigShooterBottom.feedForward.sva(SHOOTER_BOTTOM_KS, SHOOTER_BOTTOM_KV, 0);

        shooterBottomMotorConfig.apply(closedLoopConfigShooterBottom);
        shooterTopMotorConfig.apply(closedLoopConfigShooterTop);

        shooterBottomSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.SHOOTER_BOTTOM_BETA_CAN_ID,
                        MotorType.kBrushless);
        shooterTopSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.SHOOTER_TOP_BETA_CAN_ID,
                        MotorType.kBrushless);

        shooterBottomSparkFlex.configure(
                shooterBottomMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        shooterTopSparkFlex.configure(
                shooterTopMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        shooterBottomPIDController = shooterBottomSparkFlex.getClosedLoopController();
        shooterTopPIDController = shooterTopSparkFlex.getClosedLoopController();

        shooterBottomEncoder = shooterBottomSparkFlex.getEncoder();
        shooterTopEncoder = shooterTopSparkFlex.getEncoder();
    }

    public void runShooter(Distance distanceToHub) {
        desiredBottomVelocity = getBottomRollerSpeed(distanceToHub);
        desiredTopVelocity = getTopRollerSpeed(distanceToHub);

        shooterBottomPIDController.setSetpoint(
                desiredBottomVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        shooterTopPIDController.setSetpoint(
                desiredTopVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    private AngularVelocity getBottomRollerSpeed(Distance distanceToHub) {
        AngularVelocity calculatedDesiredVelocity;
        //  .compareTo returns a number greater than 0 if what it's being called on is GREATER than
        // the parameter
        if (distanceToHub.compareTo(bottomRollerCurveBoundary) < 0) {
            calculatedDesiredVelocity =
                    RPM.of(
                            bottomRollerCloseToHubCurveSlope * distanceToHub.in(Inches)
                                    + bottomRollerCloseToHubCurveIntercept);
        } else {
            calculatedDesiredVelocity =
                    RPM.of(
                            bottomRollerFarFromHubCurveSlope * distanceToHub.in(Inches)
                                    + bottomRollerFarFromHubCurveIntercept);
        }
        // check to make sure it's not negative!
        if (calculatedDesiredVelocity.in(RadiansPerSecond) < 0) {
            return RadiansPerSecond.of(0);
        }
        // check to make sure it's not greater than max speed
        if (calculatedDesiredVelocity.compareTo(RobotConstants.MotorConstants.VORTEX_FREE_SPEED)
                > 0) {
            return RobotConstants.MotorConstants.VORTEX_FREE_SPEED;
        }
        return calculatedDesiredVelocity;
    }

    private AngularVelocity getTopRollerSpeed(Distance distanceToHub) {
        AngularVelocity calculatedDesiredVelocity =
                RPM.of(
                        topRollerCurveFitSlope
                                * Math.log(distanceToHub.in(Inches) + topRollerCurveFitIntercept));
        // check to make sure it's not negative!
        if (calculatedDesiredVelocity.in(RadiansPerSecond) < 0) {
            return RadiansPerSecond.of(0);
        }
        // check to make sure it's not greater than max speed (.compareTo returns a number greater
        // than 0 if what it's being called on is GREATER than the parameter)
        if (calculatedDesiredVelocity.compareTo(RobotConstants.MotorConstants.VORTEX_FREE_SPEED)
                > 0) {
            return RobotConstants.MotorConstants.VORTEX_FREE_SPEED;
        }
        return calculatedDesiredVelocity;
    }

    public void defaultBehavior() {
        desiredBottomVelocity = RadiansPerSecond.of(0);
        desiredTopVelocity = RadiansPerSecond.of(0);

        shooterBottomPIDController.setSetpoint(
                desiredBottomVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        shooterTopPIDController.setSetpoint(
                desiredTopVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void updateStates(ShooterIOState state) {
        state.topRollerDesiredSpeed = desiredTopVelocity.in(RadiansPerSecond);
        state.topRollerActualSpeed = shooterTopEncoder.getVelocity();
        state.topRollerCurrent = shooterTopSparkFlex.getOutputCurrent();
        state.topRollerAppliedVoltage =
                shooterTopSparkFlex.getBusVoltage() * shooterTopSparkFlex.getAppliedOutput();
        state.bottomRollerDesiredSpeed = desiredBottomVelocity.in(RadiansPerSecond);
        state.bottomRollerCurrent = shooterBottomSparkFlex.getOutputCurrent();
        state.bottomRollerActualSpeed = shooterBottomEncoder.getVelocity();
        state.bottomRollerAppliedVoltage =
                shooterBottomSparkFlex.getBusVoltage() * shooterTopSparkFlex.getAppliedOutput();
    }
}
