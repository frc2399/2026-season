package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class IndexerHardwarePrototype implements IndexerIO {

    public void defaultBehavior() {}
private SparkFlex indexerSparkFlex;
    private final SparkClosedLoopController indexerPidController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double INDEXER_P = 0;
    private final double INDEXER_D = 0;
    private final double INDEXER_KS = 0.1;
    private final double INDEXER_KV = 0;

    private final ClosedLoopConfig indexClosedLoopConfig = new ClosedLoopConfig();
    private final RelativeEncoder indexerEncoder;

    public IndexerHardwarePrototype() {
        SparkMaxConfig indexSparkMaxConfig = new SparkMaxConfig();

        indexSparkMaxConfig.idleMode(IdleMode.kBrake);
        indexSparkMaxConfig.inverted(true);
        indexSparkMaxConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

        indexSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
        indexSparkMaxConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        indexSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        indexSparkMaxConfig.closedLoop.pid(INDEXER_P, 0, INDEXER_D);
        indexSparkMaxConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        indexClosedLoopConfig.feedForward.sva(INDEXER_KS, INDEXER_KV, 0);

        indexSparkMaxConfig.apply(indexClosedLoopConfig);

        indexerSparkFlex =
                new SparkFlex(RobotConstants.MotorIdConstants.INDEXER_CAN_ID, MotorType.kBrushless);
        indexerSparkFlex.configure(
            indexSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexerPidController = indexerSparkFlex.getClosedLoopController();

        indexerEncoder = indexerSparkFlex.getEncoder();
    }

    public void runIndexer() {
        indexerPidController.setSetpoint(
                0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond), ControlType.kVelocity);

        SmartDashboard.putNumber(
                "Indexer/desiredspeed", 0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        SmartDashboard.putNumber("Indexer/actualspeed", indexerEncoder.getVelocity());
    }

    public void setZero() {
        indexerPidController.setSetpoint(0, ControlType.kVelocity);

        SmartDashboard.putNumber("Indexer/desiredspeed", 0);
        SmartDashboard.putNumber("Indexer/actualspeed", indexerEncoder.getVelocity());
    }
}