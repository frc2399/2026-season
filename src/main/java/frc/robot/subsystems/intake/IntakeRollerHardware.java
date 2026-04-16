package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
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

public class IntakeRollerHardware implements IntakeRollerIO {

    private SparkFlex intakeLeaderSparkFlex;
    private SparkFlex intakeFollowerSparkFlex;

    private final SparkClosedLoopController intakeLeaderPidController;
    private final SparkClosedLoopController intakeFollowerPidController;

    private static final double ROLLER_GEAR_RATIO = 3.0 / 2.0;
    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI / ROLLER_GEAR_RATIO);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR =
            RadiansPerSecond.of((2 * Math.PI / 60) / ROLLER_GEAR_RATIO);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double INTAKE_LEADER_D = 0.0;
    private final double INTAKE_FOLLOWER_D = 0.0;

    private static final double DEFAULT_INTAKE_LEADER_P = 0.001;
    private static final double DEFAULT_INTAKE_LEADER_KS = 0.1;
    private static final double DEFAULT_INTAKE_LEADER_KV =
            12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
    private static final double DEFAULT_INTAKE_FOLLOWER_P = 0.001;
    private static final double DEFAULT_INTAKE_FOLLOWER_KS = 0.1;
    private static final double DEFAULT_INTAKE_FOLLOWER_KV =
            12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);

    // private static final TunableNumber TUNABLE_INTAKE_P =
    // new TunableNumber("Intake/intake_p", DEFAULT_INTAKE_P, true);
    // private static final TunableNumber TUNABLE_INTAKE_KS =
    // new TunableNumber("Intake/intake_ks", DEFAULT_INTAKE_KS, true);
    // private static final TunableNumber TUNABLE_INTAKE_KV =
    // new TunableNumber("Intake/intake_kv", DEFAULT_INTAKE_KV, true);

    private final SparkFlexConfig intakeLeaderMotorConfig = new SparkFlexConfig();
    private final SparkFlexConfig intakeFollowerMotorConfig = new SparkFlexConfig();

    private final ClosedLoopConfig closedLoopConfigIntakeLeader = new ClosedLoopConfig();
    private final ClosedLoopConfig closedLoopConfigIntakeFollower = new ClosedLoopConfig();

    private final RelativeEncoder intakeLeaderEncoder;
    private final RelativeEncoder intakeFollowerEncoder;

    private AngularVelocity desiredLeaderVelocity = RadiansPerSecond.of(0);
    private AngularVelocity desiredFollowerVelocity = RadiansPerSecond.of(0);

    public IntakeRollerHardware(int rollerCANId) {
        intakeLeaderMotorConfig.idleMode(IdleMode.kBrake);
        intakeFollowerMotorConfig.idleMode(IdleMode.kBrake);
        intakeLeaderMotorConfig.inverted(true);
        intakeFollowerMotorConfig.inverted(true);
        intakeLeaderMotorConfig.smartCurrentLimit(
                (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        intakeFollowerMotorConfig.smartCurrentLimit(
                (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        intakeLeaderMotorConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        intakeFollowerMotorConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));

        intakeLeaderMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));
        intakeFollowerMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        intakeLeaderMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeFollowerMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        intakeLeaderMotorConfig.closedLoop.pid(DEFAULT_INTAKE_LEADER_P, 0, INTAKE_LEADER_D);
        intakeFollowerMotorConfig.closedLoop.pid(DEFAULT_INTAKE_FOLLOWER_P, 0, INTAKE_FOLLOWER_D);
        intakeLeaderMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
        intakeFollowerMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
        closedLoopConfigIntakeLeader.feedForward.sva(
                DEFAULT_INTAKE_LEADER_KS, DEFAULT_INTAKE_LEADER_KV, 0);
        closedLoopConfigIntakeFollower.feedForward.sva(
                DEFAULT_INTAKE_FOLLOWER_KS, DEFAULT_INTAKE_FOLLOWER_KV, 0);
        intakeLeaderMotorConfig.apply(closedLoopConfigIntakeLeader);
        intakeFollowerMotorConfig.apply(closedLoopConfigIntakeFollower);

        intakeLeaderSparkFlex = new SparkFlex(rollerCANId, MotorType.kBrushless);
        intakeFollowerSparkFlex = new SparkFlex(rollerCANId, MotorType.kBrushless);

        var intakeLeaderStatus =
                intakeLeaderSparkFlex.configure(
                        intakeLeaderMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        intakeLeaderPidController = intakeLeaderSparkFlex.getClosedLoopController();
        var intakeFollowerStatus =
                intakeFollowerSparkFlex.configure(
                        intakeFollowerMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        intakeFollowerPidController = intakeFollowerSparkFlex.getClosedLoopController();

        intakeLeaderEncoder = intakeLeaderSparkFlex.getEncoder();
        if (intakeLeaderStatus != REVLibError.kOk) {
            System.err.println("Failed to configure intake roller motor: " + intakeLeaderStatus);
        }
        intakeFollowerEncoder = intakeFollowerSparkFlex.getEncoder();
        if (intakeFollowerStatus != REVLibError.kOk) {
            System.err.println("Failed to configure intake roller motor: " + intakeFollowerStatus);
        }
    }

    public void runIntake() {
        desiredLeaderVelocity = MotorConstants.VORTEX_FREE_SPEED.times(1);
        desiredFollowerVelocity = MotorConstants.VORTEX_FREE_SPEED.times(1);

        intakeLeaderPidController.setSetpoint(
                desiredLeaderVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        intakeFollowerPidController.setSetpoint(
                desiredFollowerVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void runIntakeBackwards() {
        desiredLeaderVelocity = MotorConstants.VORTEX_FREE_SPEED.times(1).unaryMinus();
        desiredFollowerVelocity = MotorConstants.VORTEX_FREE_SPEED.times(1).unaryMinus();

        intakeLeaderPidController.setSetpoint(
                desiredLeaderVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        intakeFollowerPidController.setSetpoint(
                desiredFollowerVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void setZero() {
        desiredLeaderVelocity = RadiansPerSecond.of(0);
        intakeLeaderPidController.setSetpoint(
                desiredLeaderVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        desiredFollowerVelocity = RadiansPerSecond.of(0);
        intakeFollowerPidController.setSetpoint(
                desiredFollowerVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void updateState(IntakeRollerIOState state) {
        state.actualSpeedRadiansPerSecond = intakeLeaderEncoder.getVelocity();
        state.actualSpeedRadiansPerSecond = intakeFollowerEncoder.getVelocity();
        state.desiredSpeedRadiansPerSecond =
                desiredLeaderVelocity.in(RadiansPerSecond) / ROLLER_GEAR_RATIO;
        state.desiredSpeedRadiansPerSecond =
                desiredFollowerVelocity.in(RadiansPerSecond) / ROLLER_GEAR_RATIO;
        state.current = intakeLeaderSparkFlex.getOutputCurrent();
        state.current = intakeFollowerSparkFlex.getOutputCurrent();
        state.appliedVoltage =
                intakeLeaderSparkFlex.getBusVoltage() * intakeLeaderSparkFlex.getAppliedOutput();
        state.appliedVoltage =
                intakeFollowerSparkFlex.getBusVoltage()
                        * intakeFollowerSparkFlex.getAppliedOutput();
        // if tuning a value, update this chunk for that motor's p, i, OR d
        // attempting to have this logic running with multiple causes a loop overrun :)
        // if (TUNABLE_INTAKE_KS.hasChanged()) {
        // closedLoopConfigIntake.feedForward.kS(TUNABLE_INTAKE_KS.get());
        // intakeMotorConfig.apply(closedLoopConfigIntake);
        // intakeSparkFlex.configure(
        // intakeMotorConfig,
        // ResetMode.kResetSafeParameters,
        // PersistMode.kPersistParameters); }

    }
}
