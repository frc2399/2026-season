package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
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
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;
import frc.robot.util.TunableNumber;

public class ShooterHardwareBetaAndComp implements ShooterIO {
    private SparkFlex shooterBottomSparkFlex;
    private SparkFlex shooterTopSparkFlex;
    private SparkClosedLoopController shooterBottomPIDController;
    private SparkClosedLoopController shooterTopPIDController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final double MIN_OUTPUT_RANGE = -1;
    private final double MAX_OUTPUT_RANGE = 1;
    private final double SHOOTER_BETA_TOP_P = 0.00008;
    private final double SHOOTER_BETA_TOP_D = 0;
    private final double SHOOTER_BETA_TOP_KS = 0.16;
    private final double SHOOTER_BETA_TOP_KV =
            12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
    private final double SHOOTER_BETA_BOTTOM_P = 0.0003;
    private final double SHOOTER_BETA_BOTTOM_D = 0;
    private final double SHOOTER_BETA_BOTTOM_KS = 0.123;
    private final double SHOOTER_BETA_BOTTOM_KV = 0.01697538;
    // 12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);

    private final double SHOOTER_TOP_DESIRED_SPEED = 0;
    private final double SHOOTER_BOTTOM_DESIRED_SPEED = 0;

    private final TunableNumber TUNABLE_SHOOTER_TOP_DESIRED_SPEED_RPM =
            new TunableNumber(
                    "Shooter/shooter_top_desired_speed (rpm)",
                    SHOOTER_TOP_DESIRED_SPEED,
                    true);
    private final TunableNumber TUNABLE_SHOOTER_BOTTOM_DESIRED_SPEED_RPM =
            new TunableNumber(
                    "Shooter/shooter_bottom_desired_speed (rpm)",
                    SHOOTER_BOTTOM_DESIRED_SPEED,
                    true);

    private final ClosedLoopConfig closedLoopConfigShooterTop = new ClosedLoopConfig();
    private final ClosedLoopConfig closedLoopConfigShooterBottom = new ClosedLoopConfig();

    private RelativeEncoder shooterBottomEncoder;
    private RelativeEncoder shooterTopEncoder;

    public AngularVelocity desiredBottomVelocity = RadiansPerSecond.of(0);
    public AngularVelocity desiredTopVelocity = RadiansPerSecond.of(0);

    SparkFlexConfig shooterBottomMotorConfig = new SparkFlexConfig();
    SparkFlexConfig shooterTopMotorConfig = new SparkFlexConfig();

    public ShooterHardwareBetaAndComp(boolean shouldInvertBottomRoller) {

        shooterBottomMotorConfig.idleMode(IdleMode.kCoast);
        shooterTopMotorConfig.idleMode(IdleMode.kCoast);
        shooterBottomMotorConfig.inverted(shouldInvertBottomRoller);
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
        shooterBottomMotorConfig.closedLoop.pid(SHOOTER_BETA_BOTTOM_P, 0, SHOOTER_BETA_BOTTOM_D);
        shooterTopMotorConfig.closedLoop.pid(SHOOTER_BETA_TOP_P, 0, SHOOTER_BETA_TOP_D);
        shooterBottomMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
        shooterTopMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        closedLoopConfigShooterTop.feedForward.sva(SHOOTER_BETA_TOP_KS, SHOOTER_BETA_TOP_KV, 0);
        closedLoopConfigShooterBottom.feedForward.sva(
                SHOOTER_BETA_BOTTOM_KS, SHOOTER_BETA_BOTTOM_KV, 0);

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

        var shooterBottomStatus =
                shooterBottomSparkFlex.configure(
                        shooterBottomMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
        var shooterTopStatus =
                shooterTopSparkFlex.configure(
                        shooterTopMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        shooterBottomPIDController = shooterBottomSparkFlex.getClosedLoopController();
        shooterTopPIDController = shooterTopSparkFlex.getClosedLoopController();

        shooterBottomEncoder = shooterBottomSparkFlex.getEncoder();
        shooterTopEncoder = shooterTopSparkFlex.getEncoder();

        if (shooterBottomStatus != REVLibError.kOk) {
            System.err.println("Failed to configure shooter bottom motor: " + shooterBottomStatus);
        }

        if (shooterTopStatus != REVLibError.kOk) {
            System.err.println("Failed to configure shooter top motor: " + shooterTopStatus);
        }
    }

    public void runShooter() {
        desiredBottomVelocity = RadiansPerSecond.of(230.3834612632515);
        desiredTopVelocity = RadiansPerSecond.of(314.1592653589793);

        shooterBottomPIDController.setSetpoint(
                desiredBottomVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        shooterTopPIDController.setSetpoint(
                desiredTopVelocity.in(RadiansPerSecond), ControlType.kVelocity);
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

    public ShooterSpeeds getCurrentTopAndBottomSpeeds() {
        double topSpeedToLog = desiredTopVelocity.in(RadiansPerSecond);
        double bottomSpeedToLog = desiredBottomVelocity.in(RadiansPerSecond);
        return new ShooterSpeeds(topSpeedToLog, bottomSpeedToLog);
    }

    @Override
    public void runTunableNumberSetpoints() {
        desiredTopVelocity = RPM.of(TUNABLE_SHOOTER_TOP_DESIRED_SPEED_RPM.get());
        desiredBottomVelocity = RPM.of(TUNABLE_SHOOTER_BOTTOM_DESIRED_SPEED_RPM.get());

        shooterBottomPIDController.setSetpoint(
                desiredBottomVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        shooterTopPIDController.setSetpoint(
                desiredTopVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }
}
