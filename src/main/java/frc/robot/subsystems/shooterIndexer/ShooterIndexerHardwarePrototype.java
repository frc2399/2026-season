package frc.robot.subsystems.shooterIndexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class ShooterIndexerHardwarePrototype implements ShooterIndexerIO {

    private SparkMax shooterIndexerSparkMax;
    private final SparkClosedLoopController shooterIndexerPidController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double SHOOTER_INDEXER_P = 0;
    private final double SHOOTER_INDEXER_D = 0;
    private final double SHOOTER_INDEXER_KS = 0.1;
    private final double SHOOTER_INDEXER_KV = 0;

    private final ClosedLoopConfig shooterIndexerClosedLoopConfig = new ClosedLoopConfig();
    private final RelativeEncoder shooterIndexerEncoder;

    private AngularVelocity desiredVelocity = RadiansPerSecond.of(0);

    public ShooterIndexerHardwarePrototype() {

        SparkMaxConfig shooterIndexerSparkMaxConfig = new SparkMaxConfig();

        shooterIndexerSparkMaxConfig.idleMode(IdleMode.kBrake);
        shooterIndexerSparkMaxConfig.inverted(true);
        shooterIndexerSparkMaxConfig.smartCurrentLimit(
                (int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

        shooterIndexerSparkMaxConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        shooterIndexerSparkMaxConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        shooterIndexerSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterIndexerSparkMaxConfig.closedLoop.pid(SHOOTER_INDEXER_P, 0, SHOOTER_INDEXER_D);
        shooterIndexerSparkMaxConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        shooterIndexerClosedLoopConfig.feedForward.sva(SHOOTER_INDEXER_KS, SHOOTER_INDEXER_KV, 0);

        shooterIndexerSparkMaxConfig.apply(shooterIndexerClosedLoopConfig);

        shooterIndexerSparkMax =
                new SparkMax(
                        RobotConstants.MotorIdConstants.SHOOTER_INDEXER_CAN_ID,
                        MotorType.kBrushless);

        shooterIndexerSparkMax.configure(
                shooterIndexerSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        shooterIndexerPidController = shooterIndexerSparkMax.getClosedLoopController();

        shooterIndexerEncoder = shooterIndexerSparkMax.getEncoder();
    }

    public void runShooterIndexer() {
        desiredVelocity =
                RadiansPerSecond.of(0.5 * MotorConstants.NEO_FREE_SPEED.in(RadiansPerSecond));
        shooterIndexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void backwardsRunShooterIndexer() {
        desiredVelocity =
                RadiansPerSecond.of(-0.5 * MotorConstants.NEO_FREE_SPEED.in(RadiansPerSecond));
        shooterIndexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void defaultBehavior() {
        desiredVelocity = RadiansPerSecond.of(0);
        shooterIndexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void updateStates(ShooterIndexerIOState state) {
        state.shooterIndexerDesiredSpeed = desiredVelocity.in(RadiansPerSecond);
        state.shooterIndexerActualSpeed = shooterIndexerEncoder.getVelocity();
        state.shooterIndexerCurrent = shooterIndexerSparkMax.getOutputCurrent();
        state.shooterIndexerAppliedVoltage = shooterIndexerSparkMax.getBusVoltage();
    }
}
