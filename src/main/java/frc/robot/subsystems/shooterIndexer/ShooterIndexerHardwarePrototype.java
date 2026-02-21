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

    private AngularVelocity desiredVelocityRad_P_S;

    public ShooterIndexerHardwarePrototype() {

        SparkMaxConfig shooterIndexerSparkMaxConfigRad_P_S = new SparkMaxConfig();

        shooterIndexerSparkMaxConfigRad_P_S.idleMode(IdleMode.kBrake);
        shooterIndexerSparkMaxConfigRad_P_S.inverted(true);
        shooterIndexerSparkMaxConfigRad_P_S.smartCurrentLimit(
                (int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

        shooterIndexerSparkMaxConfigRad_P_S.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        shooterIndexerSparkMaxConfigRad_P_S.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        shooterIndexerSparkMaxConfigRad_P_S.closedLoop.feedbackSensor(
                FeedbackSensor.kPrimaryEncoder);
        shooterIndexerSparkMaxConfigRad_P_S.closedLoop.pid(SHOOTER_INDEXER_P, 0, SHOOTER_INDEXER_D);
        shooterIndexerSparkMaxConfigRad_P_S.closedLoop.outputRange(
                MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        shooterIndexerClosedLoopConfig.feedForward.sva(SHOOTER_INDEXER_KS, SHOOTER_INDEXER_KV, 0);

        shooterIndexerSparkMaxConfigRad_P_S.apply(shooterIndexerClosedLoopConfig);

        shooterIndexerSparkMax =
                new SparkMax(
                        RobotConstants.MotorIdConstants.SHOOTER_INDEXER_CAN_ID,
                        MotorType.kBrushless);

        shooterIndexerSparkMax.configure(
                shooterIndexerSparkMaxConfigRad_P_S,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        shooterIndexerPidController = shooterIndexerSparkMax.getClosedLoopController();

        shooterIndexerEncoder = shooterIndexerSparkMax.getEncoder();
    }

    public void runShooterIndexer() {
        desiredVelocityRad_P_S =
                RadiansPerSecond.of(0.5 * MotorConstants.NEO_FREE_SPEED.in(RadiansPerSecond));
        shooterIndexerPidController.setSetpoint(
                desiredVelocityRad_P_S.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void backwardsRunShooterIndexer() {
        desiredVelocityRad_P_S =
                RadiansPerSecond.of(-0.5 * MotorConstants.NEO_FREE_SPEED.in(RadiansPerSecond));
        shooterIndexerPidController.setSetpoint(
                desiredVelocityRad_P_S.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void defaultBehavior() {
        desiredVelocityRad_P_S = RadiansPerSecond.of(0);
        shooterIndexerPidController.setSetpoint(
                desiredVelocityRad_P_S.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void updateStates(ShooterIndexerIOState state) {
        state.shooterIndexerDesiredSpeedRad_P_S = desiredVelocityRad_P_S.in(RadiansPerSecond);
        state.shooterIndexerActualSpeedRad_P_S = shooterIndexerEncoder.getVelocity();
        state.shooterIndexerCurrent = shooterIndexerSparkMax.getOutputCurrent();
        state.shooterIndexerAppliedVoltage =
                shooterIndexerSparkMax.getAppliedOutput() * shooterIndexerSparkMax.getBusVoltage();
    }
}
