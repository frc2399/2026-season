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

    private final SparkClosedLoopController intakePidController;

    private static final double ROLLER_GEAR_RATIO = 3.0 / 2.0;
    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI / ROLLER_GEAR_RATIO);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR =
            RadiansPerSecond.of((2 * Math.PI / 60) / ROLLER_GEAR_RATIO);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double INTAKE_D = 0.0;

    private final boolean LEADER_MOTOR_INVERTED = false;
    private final boolean FOLLOWER_MOTOR_INVERTED_RELATIVE_TO_LEADER = true;

    private static final double DEFAULT_INTAKE_P = .00000;
    private static final double DEFAULT_INTAKE_KS = .28;
    private static final double DEFAULT_INTAKE_KV = 0.035;

    private static final AngularVelocity ROLLER_FREE_SPEED = RadiansPerSecond.of(405);

    // private static final TunableNumber TUNABLE_INTAKE_P =
    // new TunableNumber("Intake/intake_p", DEFAULT_INTAKE_P, true);
    // private static final TunableNumber TUNABLE_INTAKE_KS =
    // new TunableNumber("Intake/intake_ks", DEFAULT_INTAKE_KS, true);
    // private static final TunableNumber TUNABLE_INTAKE_KV =
    // new TunableNumber("Intake/intake_kv", DEFAULT_INTAKE_KV, true);

    private final SparkFlexConfig globalMotorConfig = new SparkFlexConfig();

    private final ClosedLoopConfig globalClosedLoopConfig = new ClosedLoopConfig();

    private final RelativeEncoder intakeLeaderEncoder;
    private final RelativeEncoder intakeFollowerEncoder;

    private AngularVelocity desiredVelocity = RadiansPerSecond.of(0);

    public IntakeRollerHardware() {
        globalMotorConfig.idleMode(IdleMode.kBrake);
        globalMotorConfig.inverted(LEADER_MOTOR_INVERTED);
        globalMotorConfig.smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        globalMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));

        globalMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        globalMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        globalMotorConfig.closedLoop.pid(DEFAULT_INTAKE_P, 0, INTAKE_D);
        globalMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
        globalClosedLoopConfig.feedForward.sva(DEFAULT_INTAKE_KS, DEFAULT_INTAKE_KV, 0);
        globalMotorConfig.apply(globalClosedLoopConfig);

        intakeLeaderSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.INTAKE_ROLLER_BETA_AND_COMP_LEADER_CAN_ID,
                        MotorType.kBrushless);
        intakeFollowerSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.INTAKE_ROLLER_BETA_AND_COMP_FOLLOWER_CAN_ID,
                        MotorType.kBrushless);

        var intakeLeaderStatus =
                intakeLeaderSparkFlex.configure(
                        globalMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        intakePidController = intakeLeaderSparkFlex.getClosedLoopController();
        var intakeFollowerStatus =
                intakeFollowerSparkFlex.configure(
                        globalMotorConfig.follow(
                                RobotConstants.MotorIdConstants
                                        .INTAKE_ROLLER_BETA_AND_COMP_LEADER_CAN_ID,
                                FOLLOWER_MOTOR_INVERTED_RELATIVE_TO_LEADER),
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

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
        desiredVelocity = ROLLER_FREE_SPEED.times(1);

        intakePidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void runIntakeBackwards() {
        desiredVelocity = ROLLER_FREE_SPEED.times(1).unaryMinus();

        intakePidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    @Override
    public void runIntakeForShooting() {
        desiredVelocity = ROLLER_FREE_SPEED.times(.1);

        intakePidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void setZero() {
        desiredVelocity = RadiansPerSecond.of(0);
        intakePidController.setSetpoint(
                desiredVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void updateState(IntakeRollerIOState state) {
        state.leaderActualSpeedRadiansPerSecond = intakeLeaderEncoder.getVelocity();
        state.followerActualSpeedRadiansPerSecond = intakeFollowerEncoder.getVelocity();
        state.desiredSpeedRadiansPerSecond = desiredVelocity.in(RadiansPerSecond);
        state.leaderCurrent = intakeLeaderSparkFlex.getOutputCurrent();
        state.followerCurrent = intakeFollowerSparkFlex.getOutputCurrent();
        state.leaderAppliedVoltage =
                intakeLeaderSparkFlex.getBusVoltage() * intakeLeaderSparkFlex.getAppliedOutput();
        state.followerAppliedVoltage =
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
