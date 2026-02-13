package frc.robot.subsystems.spindexer;

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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class SpindexerHardwarePrototype implements SpindexerIO {

    public void defaultBehavior() {}

    public void updateStates(SpindexerIOState state) {}

    private SparkFlex spindexerSparkFlex;
    private final SparkClosedLoopController spindexerPidController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double SPINDEXER_P = 0;
    private final double SPINDEXER_D = 0;
    private final double SPINDEXER_KS = 0.1;
    private final double SPINDEXER_KV = 0;

    private final ClosedLoopConfig spindexClosedLoopConfig = new ClosedLoopConfig();
    private final RelativeEncoder spindexerEncoder;

    public SpindexerHardwarePrototype() {
        SparkMaxConfig spindexSparkMaxConfig = new SparkMaxConfig();

        spindexSparkMaxConfig.idleMode(IdleMode.kBrake);
        spindexSparkMaxConfig.inverted(true);
        spindexSparkMaxConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

        spindexSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
        spindexSparkMaxConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        spindexSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        spindexSparkMaxConfig.closedLoop.pid(SPINDEXER_P, 0, SPINDEXER_D);
        spindexSparkMaxConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        spindexClosedLoopConfig.feedForward.sva(SPINDEXER_KS, SPINDEXER_KV, 0);

        spindexSparkMaxConfig.apply(spindexClosedLoopConfig);

        spindexerSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.SPINDEXER_CAN_ID, MotorType.kBrushless);
        spindexerSparkFlex.configure(
                spindexSparkMaxConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        spindexerPidController = spindexerSparkFlex.getClosedLoopController();

        spindexerEncoder = spindexerSparkFlex.getEncoder();
    }

    public void runSpindexer() {
        spindexerPidController.setSetpoint(
                0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond), ControlType.kVelocity);

        SmartDashboard.putNumber("c", 0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        SmartDashboard.putNumber("Spindexer/actualspeed", spindexerEncoder.getVelocity());
    }

    public void setZero() {
        spindexerPidController.setSetpoint(0, ControlType.kVelocity);

        SmartDashboard.putNumber("Indexer/desiredspeed", 0);
        SmartDashboard.putNumber("Spindexer/actualspeed", spindexerEncoder.getVelocity());
    }
}
