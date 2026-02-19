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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;
import frc.robot.constants.RobotConstants.MotorIdConstants;

public class ShooterIndexerHardwareBeta implements ShooterIndexerIO {

    private SparkFlex shooterIndexerSparkFlex;
    private final SparkFlexConfig shooterIndexerSparkFlexConfig;
    private final ClosedLoopConfig shooterIndexerClosedLoopConfig;
    private final RelativeEncoder shooterIndexerEncoder;
    private final SparkClosedLoopController shooterIndexerPidController;

    private static final boolean SHOOTER_INDEXER_MOTOR_INVERTED = false;
    private static final IdleMode SHOOTER_INDEXER_IDLE_MODE = IdleMode.kBrake;

    private static final Angle SHOOTER_INDEXER_POSITION_CONVERSION_FACTOR =
            Radians.of(2 * Math.PI / 3); // 3 : 1 gear ratio
    private static final AngularVelocity SHOOTER_INDEXER_VELOCITY_CONVERSION_FACTOR =
            RadiansPerSecond.of(2 * Math.PI / 60 / 3);

    private static final double SHOOTER_INDEXER_P = 0;
    private static final double SHOOTER_INDEXER_I = 0;
    private static final double SHOOTER_INDEXER_D = 0;
    private static final double SHOOTER_INDEXER_MIN_OUTPUT = -1;
    private static final double SHOOTER_INDEXER_MAX_OUTPUT = 1;

    private static final double SHOOTER_INDEXER_KS = 0.01;
    private static final double SHOOTER_INDEXER_KV =
            12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
    //      0.01;
    private static final double SHOOTER_INDEXER_KA = 0;

    private AngularVelocity desiredVelocity = RadiansPerSecond.of(0);

    public ShooterIndexerHardwareBeta() {
        shooterIndexerSparkFlexConfig = new SparkFlexConfig();
        shooterIndexerClosedLoopConfig = new ClosedLoopConfig();

        shooterIndexerSparkFlexConfig
                .inverted(SHOOTER_INDEXER_MOTOR_INVERTED)
                .idleMode(SHOOTER_INDEXER_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        shooterIndexerSparkFlexConfig
                .encoder
                .positionConversionFactor(SHOOTER_INDEXER_POSITION_CONVERSION_FACTOR.in(Radians))
                .velocityConversionFactor(
                        SHOOTER_INDEXER_VELOCITY_CONVERSION_FACTOR.in(RadiansPerSecond));
        shooterIndexerSparkFlexConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SHOOTER_INDEXER_P, SHOOTER_INDEXER_I, SHOOTER_INDEXER_D)
                .outputRange(SHOOTER_INDEXER_MIN_OUTPUT, SHOOTER_INDEXER_MAX_OUTPUT);

        shooterIndexerClosedLoopConfig.feedForward.sva(
                SHOOTER_INDEXER_KS, SHOOTER_INDEXER_KV, SHOOTER_INDEXER_KA);

        shooterIndexerSparkFlexConfig.apply(shooterIndexerClosedLoopConfig);

        shooterIndexerSparkFlex =
                new SparkFlex(MotorIdConstants.SHOOTER_INDEXER_BETA_CAN_ID, MotorType.kBrushless);
        shooterIndexerSparkFlex.configure(
                shooterIndexerSparkFlexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        shooterIndexerEncoder = shooterIndexerSparkFlex.getEncoder();
        shooterIndexerPidController = shooterIndexerSparkFlex.getClosedLoopController();
    }

    @Override
    public void runShooterIndexer() {
        desiredVelocity = MotorConstants.VORTEX_FREE_SPEED.times(.0);
        // desiredVelocity = MotorConstants.VORTEX_FREE_SPEED.times(0.03);
        shooterIndexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    @Override
    public void backwardsRunShooterIndexer() {
        desiredVelocity = MotorConstants.VORTEX_FREE_SPEED.times(-0.0781);
        shooterIndexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    @Override
    public void defaultBehavior() {
        desiredVelocity = MotorConstants.VORTEX_FREE_SPEED.times(0);
        shooterIndexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    @Override
    public void updateStates(ShooterIndexerIOState state) {
        state.shooterIndexerDesiredSpeed = desiredVelocity.in(RadiansPerSecond);
        state.shooterIndexerActualSpeed = shooterIndexerEncoder.getVelocity();
        state.shooterIndexerAppliedVoltage =
                shooterIndexerSparkFlex.getBusVoltage()
                        * shooterIndexerSparkFlex.getAppliedOutput();
        state.shooterIndexerCurrent = shooterIndexerSparkFlex.getOutputCurrent();
    }
}
