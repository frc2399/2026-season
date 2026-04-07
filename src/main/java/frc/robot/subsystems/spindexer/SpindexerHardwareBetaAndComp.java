package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

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
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class SpindexerHardwareBetaAndComp implements SpindexerIO {

    private SparkFlex spindexerSparkFlex;
    private final SparkClosedLoopController spindexerPidController;

    private final Angle ENCODER_POSITION_FACTOR;
    private final AngularVelocity ENCODER_VELOCITY_FACTOR;
    private final int SPINDEXER_GEAR_RATIO;

    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double SPINDEXER_P = 0.00006;
    private final double SPINDEXER_D = 0;
    private final double SPINDEXER_KS = 0.13;
    private final double SPINDEXER_KV = 0.017;
    private final boolean SPINDEXER_INVERTED = false;

    private final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.1);

    private final ClosedLoopConfig spindexClosedLoopConfig = new ClosedLoopConfig();
    private final RelativeEncoder spindexerEncoder;

    private AngularVelocity desiredVelocity = RadiansPerSecond.of(0);

    SparkFlexConfig spindexerSparkFlexConfig = new SparkFlexConfig();

    public SpindexerHardwareBetaAndComp(int spindexerGearRatio) {

        this.SPINDEXER_GEAR_RATIO = spindexerGearRatio;
        ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI / spindexerGearRatio);
        ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60 / spindexerGearRatio);

        spindexerSparkFlexConfig.idleMode(IdleMode.kCoast);
        spindexerSparkFlexConfig.inverted(SPINDEXER_INVERTED);
        spindexerSparkFlexConfig.smartCurrentLimit(
                (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        spindexerSparkFlexConfig.closedLoopRampRate(CLOSED_LOOP_RAMP_RATE.in(Seconds));

        spindexerSparkFlexConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        spindexerSparkFlexConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        spindexerSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        spindexerSparkFlexConfig.closedLoop.pid(SPINDEXER_P, 0, SPINDEXER_D);
        spindexerSparkFlexConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        spindexClosedLoopConfig.feedForward.sva(SPINDEXER_KS, SPINDEXER_KV, 0);

        spindexerSparkFlexConfig.apply(spindexClosedLoopConfig);

        spindexerSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.SPINDEXER_CAN_ID, MotorType.kBrushless);

        var spindexerStatus =
                spindexerSparkFlex.configure(
                        spindexerSparkFlexConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        spindexerPidController = spindexerSparkFlex.getClosedLoopController();

        spindexerEncoder = spindexerSparkFlex.getEncoder();

        if (spindexerStatus != REVLibError.kOk) {
            System.err.println("Failed to configure spindexer motor: " + spindexerStatus);
        }
    }

    public void runSpindexer() {
        desiredVelocity =
                RadiansPerSecond.of(0.6 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));

        spindexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void runSpindexerBackwards() {
        desiredVelocity =
                RadiansPerSecond.of(-0.10 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        spindexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void defaultBehavior() {
        desiredVelocity = RadiansPerSecond.of(0);
        spindexerPidController.setSetpoint(0, ControlType.kVelocity);
    }

    public void updateStates(SpindexerIOState state) {
        state.spindexerDesiredSpeedRad_P_S =
                desiredVelocity.in(RadiansPerSecond) / SPINDEXER_GEAR_RATIO;
        state.spindexerActualSpeedRad_P_S = spindexerEncoder.getVelocity();
        state.spindexerCurrent = spindexerSparkFlex.getOutputCurrent();
        state.spindexerAppliedVoltage =
                spindexerSparkFlex.getAppliedOutput() * spindexerSparkFlex.getBusVoltage();
    }
}
