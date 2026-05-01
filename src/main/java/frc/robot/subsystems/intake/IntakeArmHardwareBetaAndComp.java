package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;

public class IntakeArmHardwareBetaAndComp implements IntakeArmIO {
    // ratios: motor to mechanism if 45:1
    // encoder to mechanism is 1:1
    private final SparkFlex intakeArmSparkFlex;
    private final AbsoluteEncoder intakeArmAbsoluteEncoder;
    private final SparkClosedLoopController intakeArmClosedLoop;

    private final SparkFlexConfig intakeArmSparkFlexConfig = new SparkFlexConfig();
    private final ClosedLoopConfig intakeArmClosedLoopConfig = new ClosedLoopConfig();

    // inversions
    private static final boolean INTAKE_ARM_MOTOR_INVERTED = true;
    private static final boolean INTAKE_ARM_ENCODER_INVERTED = false;

    // conversion factors for the absolute encoder - that's why they're 1:1 rather
    // than 45:1
    private static final Angle INTAKE_ARM_POSITION_CONVERSION_FACTOR = Radians.of(2 * Math.PI);
    private static final AngularVelocity INTAKE_ARM_VELOCITY_CONVERSION_FACTOR =
            RadiansPerSecond.of(2 * Math.PI / 60);

    // pid
    private static final double INTAKE_ARM_P = .9;
    private static final double INTAKE_ARM_I = 0;
    private static final double INTAKE_ARM_D = 0;

    // feedforward
    private static final double INTAKE_ARM_KS = 0.0;
    private static final double INTAKE_ARM_KV = 1.03;
    private static final double INTAKE_ARM_KA = 0.0;
    private static final double INTAKE_ARM_KCOS = .33;

    // we use an outside feedforward, because arms also have a cosine factor (the
    // impact of gravity changes), and revlib's is poorly documented so it's not
    // properly working
    private static final ArmFeedforward intakeArmFeedforward =
            new ArmFeedforward(INTAKE_ARM_KS, INTAKE_ARM_KCOS, INTAKE_ARM_KV, INTAKE_ARM_KA);

    // output
    private static final double INTAKE_ARM_MIN_OUTPUT = -1;
    private static final double INTAKE_ARM_MAX_OUTPUT = 1;

    // soft limits
    private static final Angle INTAKE_ARM_FORWARD_MAX_ANGLE = Degrees.of(112);
    private static final boolean INTAKE_ARM_FORWARD_SOFTLIMIT_ENABLED = true;
    private static final Angle INTAKE_ARM_REVERSE_MAX_ANGLE = Degrees.of(-37);
    private static final boolean INTAKE_ARM_REVERSE_SOFTLIMIT_ENABLED = true;

    private Angle desiredAngle = Degrees.of(0);
    private AngularVelocity desiredAngularVelocity = DegreesPerSecond.of(0);

    private Angle desiredArmAngleBelowTrench = Degrees.of(30);

    // profiled PID control
    private AngularVelocity ARM_MAX_VEL = RadiansPerSecond.of(4.0);
    private AngularAcceleration ARM_MAX_ACCEL = RadiansPerSecondPerSecond.of(2.5);
    private TrapezoidProfile intakeArmTrapezoidProfile =
            new TrapezoidProfile(
                    new Constraints(
                            ARM_MAX_VEL.in(RadiansPerSecond),
                            ARM_MAX_ACCEL.in(RadiansPerSecondPerSecond)));
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State();
    private TrapezoidProfile.State intermediateSetpointState = new TrapezoidProfile.State();
    private Angle pidDeadband =
            Degrees.of(3); // due to significant backlash on the mechanism, we don't want to adjust

    // as

    // much if we are within 1 degree of the goal

    public IntakeArmHardwareBetaAndComp(Angle pidDeadband) {
        this.pidDeadband = pidDeadband;
        intakeArmSparkFlexConfig
                .inverted(INTAKE_ARM_MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(
                        (int) RobotConstants.MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        intakeArmSparkFlexConfig
                .absoluteEncoder
                .inverted(INTAKE_ARM_ENCODER_INVERTED)
                .positionConversionFactor(INTAKE_ARM_POSITION_CONVERSION_FACTOR.in(Radians))
                .velocityConversionFactor(
                        INTAKE_ARM_VELOCITY_CONVERSION_FACTOR.in(RadiansPerSecond))
                .zeroCentered(true);

        intakeArmSparkFlexConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(INTAKE_ARM_P, INTAKE_ARM_I, INTAKE_ARM_D)
                .outputRange(INTAKE_ARM_MIN_OUTPUT, INTAKE_ARM_MAX_OUTPUT)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-Math.PI, Math.PI);

        /*
         * soft limits are code-enforced limits on where the mechanism can go
         * they're called SOFT limits because the mechanism can still technically go
         * past it
         * if the mechanism can't physically go past it, that's a HARD limit/stop
         */
        intakeArmSparkFlexConfig
                .softLimit
                .forwardSoftLimit(INTAKE_ARM_FORWARD_MAX_ANGLE.in(Radians))
                .forwardSoftLimitEnabled(INTAKE_ARM_FORWARD_SOFTLIMIT_ENABLED)
                .reverseSoftLimit(INTAKE_ARM_REVERSE_MAX_ANGLE.in(Radians))
                .reverseSoftLimitEnabled(INTAKE_ARM_REVERSE_SOFTLIMIT_ENABLED);

        intakeArmSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.INTAKE_ARM_BETA_CAN_ID,
                        MotorType.kBrushless);

        intakeArmSparkFlex.configure(
                intakeArmSparkFlexConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        intakeArmAbsoluteEncoder = intakeArmSparkFlex.getAbsoluteEncoder();
        intakeArmClosedLoop = intakeArmSparkFlex.getClosedLoopController();
    }

    @Override
    public void setSetpoint(IntakeArmSetpoint setpoint) {
        if (setpoint == IntakeArmSetpoint.DEPLOYED) {
            desiredAngle = Degrees.of(-25.5);
            goalState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
        } else if (setpoint == IntakeArmSetpoint.STOWED) {
            desiredAngle = Degrees.of(-21.4);
            goalState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
        } else if (setpoint == IntakeArmSetpoint.FEED_FUEL_UPPER_BOUND) {
            desiredAngle = Degrees.of(90);
            goalState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
        } else if (setpoint == IntakeArmSetpoint.FEED_FUEL_LOWER_BOUND) {
            desiredAngle = Degrees.of(45);
            goalState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
        } else if (setpoint == IntakeArmSetpoint.MID) {
            desiredAngle = Degrees.of(-16);
            goalState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
        }
    }

    @Override
    public void runOffVolts() {
        intakeArmClosedLoop.setSetpoint(0.5, ControlType.kVoltage);
    }

    @Override
    public void runIntakeArmOutVelocity() {
        desiredAngularVelocity = RadiansPerSecond.of(-1);
        double setpointFF =
                intakeArmFeedforward.calculate(
                        intakeArmAbsoluteEncoder.getPosition(),
                        desiredAngularVelocity.in(RadiansPerSecond));
        intakeArmClosedLoop.setSetpoint(
                desiredAngularVelocity.in(RadiansPerSecond),
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot1,
                setpointFF);
    }

    @Override
    public void runIntakeArmInVelocity() {
        desiredAngularVelocity = RadiansPerSecond.of(1);
        double setpointFF =
                intakeArmFeedforward.calculate(
                        intakeArmAbsoluteEncoder.getPosition(),
                        desiredAngularVelocity.in(RadiansPerSecond));
        intakeArmClosedLoop.setSetpoint(
                desiredAngularVelocity.in(RadiansPerSecond),
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot1,
                setpointFF);
    }

    @Override
    public void runIntakeArmZeroVelocity() {
        desiredAngularVelocity = RadiansPerSecond.of(0);
        double setpointFF =
                intakeArmFeedforward.calculate(
                        intakeArmAbsoluteEncoder.getPosition(),
                        desiredAngularVelocity.in(RadiansPerSecond));
        intakeArmClosedLoop.setSetpoint(
                desiredAngularVelocity.in(RadiansPerSecond),
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot1,
                setpointFF);
    }

    @Override
    public void calculateNextIntermediateSetpoint() {
        intermediateSetpointState =
                intakeArmTrapezoidProfile.calculate(
                        RobotConstants.SpeedConstants.MAIN_LOOP_FREQUENCY.in(Seconds),
                        intermediateSetpointState,
                        goalState);
        double calculatedFeedforward;
        if (Math.abs(goalState.position - intakeArmAbsoluteEncoder.getPosition())
                < pidDeadband.in(Radians)) {
            calculatedFeedforward =
                    intakeArmFeedforward.calculate(intermediateSetpointState.position, 0);
        } else {
            calculatedFeedforward =
                    intakeArmFeedforward.calculate(
                            intermediateSetpointState.position, intermediateSetpointState.velocity);
        }
        intakeArmClosedLoop.setSetpoint(
                intermediateSetpointState.position,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                calculatedFeedforward);
    }

    @Override
    public void resetSetpointsToCurrentPosition() {
        goalState.position = intakeArmAbsoluteEncoder.getPosition();
        goalState.velocity = 0;

        intermediateSetpointState.position = intakeArmAbsoluteEncoder.getPosition();
        intermediateSetpointState.velocity = 0;
    }

    @Override
    public void updateState(IntakeArmIOState state) {
        state.desiredAngleDegrees = desiredAngle.in(Degrees);
        state.intermediateAngleDegrees = intermediateSetpointState.position * (180 / Math.PI);
        state.actualAngleDegrees = intakeArmAbsoluteEncoder.getPosition() * (180 / Math.PI);
        state.velocityDegreesPerSecond = intakeArmAbsoluteEncoder.getVelocity() * (180 / Math.PI);
        state.intermediateVelocityDegreesPerSecond =
                intermediateSetpointState.velocity * (180 / Math.PI);
        state.desiredVelocityDegreesPerSecond = desiredAngularVelocity.in(DegreesPerSecond);
        state.appliedVoltage =
                intakeArmSparkFlex.getBusVoltage() * intakeArmSparkFlex.getAppliedOutput();
        state.current = intakeArmSparkFlex.getOutputCurrent();
    }

    @Override
    public boolean isArmBelowTrench() {
        return (desiredArmAngleBelowTrench.in(Degrees)
                >= intakeArmAbsoluteEncoder.getPosition() * (180 / Math.PI));
    }
}
