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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class SpindexerHardwareBeta implements SpindexerIO {

    private SparkFlex spindexerSparkFlex;
    private final SparkClosedLoopController spindexerPidController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI / 6); // 6 : 1 gear ratio
    private final AngularVelocity ENCODER_VELOCITY_FACTOR =
            RadiansPerSecond.of(2 * Math.PI / 60 / 6);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double SPINDEXER_P = 0;
    private final double SPINDEXER_D = 0;
    private final double SPINDEXER_KS = 0.1;
    private final double SPINDEXER_KV =
            12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
    private final boolean SPINDEXER_INVERTED = false;

    //     private static final TunableNumber TUNABLE_SPINDEXER_KS =
    //             new TunableNumber("Spindexer/spindexer_ks", 0.1, true);

    private final ClosedLoopConfig spindexClosedLoopConfig = new ClosedLoopConfig();
    private final RelativeEncoder spindexerEncoder;

    private AngularVelocity desiredVelocity = RadiansPerSecond.of(0);

    SparkFlexConfig spindexerSparkFlexConfig = new SparkFlexConfig();

    public SpindexerHardwareBeta() {

        spindexerSparkFlexConfig.idleMode(IdleMode.kCoast);
        spindexerSparkFlexConfig.inverted(SPINDEXER_INVERTED);
        spindexerSparkFlexConfig.smartCurrentLimit(
                (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        spindexerSparkFlexConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        spindexerSparkFlexConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        spindexerSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        spindexerSparkFlexConfig.closedLoop.pid(SPINDEXER_P, 0, SPINDEXER_D);
        spindexerSparkFlexConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        spindexClosedLoopConfig.feedForward.sva(SPINDEXER_KS, SPINDEXER_KV, 0);

        spindexerSparkFlexConfig.apply(spindexClosedLoopConfig);

        spindexerSparkFlex.configure(
                spindexerSparkFlexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        spindexerPidController = spindexerSparkFlex.getClosedLoopController();

        spindexerEncoder = spindexerSparkFlex.getEncoder();
    }

    public void runSpindexer() {
        desiredVelocity =
                RadiansPerSecond.of(0.1474 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        spindexerPidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void defaultBehavior() {
        desiredVelocity = RadiansPerSecond.of(0);
        spindexerPidController.setSetpoint(0, ControlType.kVelocity);
    }

    public void updateStates(SpindexerIOState state) {
        state.spindexerDesiredSpeed = desiredVelocity.in(RadiansPerSecond);
        state.spindexerActualSpeed = spindexerEncoder.getVelocity();
        state.spindexerCurrent = spindexerSparkFlex.getOutputCurrent();
        state.spindexerAppliedVoltage =
                spindexerSparkFlex.getAppliedOutput() * spindexerSparkFlex.getBusVoltage();

        // if (TUNABLE_SPINDEXER_KS.hasChanged()) {
        //   spindexClosedLoopConfig.feedForward.kS(TUNABLE_SPINDEXER_KS.get());
        // spindexerSparkFlexConfig.apply(spindexClosedLoopConfig);
        //  spindexerSparkFlex.configure(
        //     spindexerSparkFlexConfig,
        //     ResetMode.kResetSafeParameters,
        //    PersistMode.kPersistParameters);
    }
}
