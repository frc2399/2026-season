package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;
import frc.robot.util.TunableNumber;

public class IntakeArmHardwareBeta implements IntakeArmIO {
    // 45:1 ratio
    // sparkflex
    // vortex
    // abs enc
    // 1:1 AFTER enc :)))
    private final SparkFlex intakeArmSparkFlex;
    private final AbsoluteEncoder intakeArmAbsoluteEncoder;
    private final SparkClosedLoopController intakeArmClosedLoop;

    private final SparkFlexConfig intakeArmSparkFlexConfig = new SparkFlexConfig();
    private final ClosedLoopConfig intakeArmClosedLoopConfig = new ClosedLoopConfig();

    // inversions
    private static final boolean INTAKE_ARM_MOTOR_INVERTED = true;
    private static final boolean INTAKE_ARM_ENCODER_INVERTED = false;

    // conversion factors (45:1 is our gear ratio)
    private static final Angle INTAKE_ARM_POSITION_CONVERSION_FACTOR =
            Radians.of(2 * Math.PI); // this subsystem is
    // in
    // radians
    private static final AngularVelocity INTAKE_ARM_VELOCITY_CONVERSION_FACTOR =
            RadiansPerSecond.of(
                    2 * Math.PI / 60); // math more explicit to make conversion more understandable

    // pid
    private static final double INTAKE_ARM_P = 0.00;
    private static final double INTAKE_ARM_I = 0;
    private static final double INTAKE_ARM_D = 0;
    // feedforward (values calculated using the recalc link kevin shared in the
    // discord in
    // #programming on 2/26)
    private static final double INTAKE_ARM_KS = 0;
    private static final double INTAKE_ARM_KV = 3.20;
    private static final double INTAKE_ARM_KA = 0.00011;
    private static final double INTAKE_ARM_KCOS = .14;

    private static final ArmFeedforward intakeArmFeedforward =
            new ArmFeedforward(INTAKE_ARM_KS, INTAKE_ARM_KCOS, INTAKE_ARM_KV, INTAKE_ARM_KA);

    private static final TunableNumber TUNABLE_INTAKE_KCOS =
            new TunableNumber("intake/arm", INTAKE_ARM_KCOS, true);

    /*
     * our arm is rotational, so the impact of gravity changes as we rotate, and our
     * feedforward needs to compensate. the kcos is the factor to compensate by
     * revlib demands that it can get from what we're logging in (rad/s) to
     * rotations of mechanism / second to accurately compensate. that's kcosratio
     */
    private static final double INTAKE_ARM_KCOS_RATIO =
            1 / (2 * Math.PI); // convert from mechanism radians to rotations

    // output
    private static final double INTAKE_ARM_MIN_OUTPUT = -1;
    private static final double INTAKE_ARM_MAX_OUTPUT = 1;

    // soft limits
    private static final Angle INTAKE_ARM_FORWARD_MAX_ANGLE = Degrees.of(3);
    private static final boolean INTAKE_ARM_FORWARD_SOFTLIMIT_ENABLED = false;
    private static final Angle INTAKE_ARM_REVERSE_MAX_ANGLE = Degrees.of(-60);
    private static final boolean INTAKE_ARM_REVERSE_SOFTLIMIT_ENABLED = false;

    private Angle desiredAngle = Degrees.of(0);

    // tunable numbers - for testing only! delete before pr
    // private double DEFAULT_KS = .001;
    // private TunableNumber TUNABLE_KS = new TunableNumber("intake/arm/voltage",
    // DEFAULT_KS,
    // true);

    public IntakeArmHardwareBeta() {
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

        // soft limits are code-enforced limits on where the mechanism can go
        // they're called SOFT limits because the mechanism can still technically go
        // past it
        // if the mechanism can't physically go past it, that's a HARD limit/stop
        intakeArmSparkFlexConfig
                .softLimit
                .forwardSoftLimit(INTAKE_ARM_FORWARD_MAX_ANGLE.in(Radians))
                .forwardSoftLimitEnabled(INTAKE_ARM_FORWARD_SOFTLIMIT_ENABLED)
                .reverseSoftLimit(INTAKE_ARM_REVERSE_MAX_ANGLE.in(Radians))
                .reverseSoftLimitEnabled(INTAKE_ARM_REVERSE_SOFTLIMIT_ENABLED);

        // intakeArmClosedLoopConfig.feedForward.sva(
        // INTAKE_ARM_KS,
        // INTAKE_ARM_KV,
        // INTAKE_ARM_KA,
        // INTAKE_ARM_KCOS,
        // INTAKE_ARM_KCOS_RATIO);

        // intakeArmSparkFlexConfig.apply(intakeArmClosedLoopConfig);

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
        // STRUCTURE SAVED FOR LATER IT HAS GOOD STUFF BUT NOT NECESSARY RN
        if (setpoint == IntakeArmSetpoint.DEPLOYED) {
            desiredAngle = Degrees.of(-10);
            // commented because i dont want something crazy to happen
            // intakeArmClosedLoop.setSetpoint(-10,ControlType.kPosition);
        } else if (setpoint == IntakeArmSetpoint.STOWED) {
            desiredAngle = Degrees.of(80);
            // intakeArmClosedLoop.setSetpoint(80, ControlType.kPosition);
        }
    }

    @Override
    public void runOffVolts() {
        System.out.println("im a volt!");
        intakeArmClosedLoop.setSetpoint(0.5, ControlType.kVoltage);
        // for testing, add backward voltage setpoint bc idk how to do that rn :)
    }

    @Override
    public void runVelocity(IntakeArmSetpoint setpoint) {
        if (setpoint == IntakeArmSetpoint.DEPLOYED) {
            // intakeArmClosedLoop.setSetpoint(-6, ControlType.kVelocity);
            AngularVelocity desiredSpeed =
                    RadiansPerSecond.of(
                            -0.1
                                    + intakeArmFeedforward.calculate(
                                            intakeArmAbsoluteEncoder.getPosition(), -0.1));
            intakeArmSparkFlex.set(desiredSpeed.in(RotationsPerSecond));
        } else if (setpoint == IntakeArmSetpoint.STOWED) {
            // intakeArmClosedLoop.setSetpoint(6, ControlType.kVelocity);
            AngularVelocity desiredSpeed =
                    RadiansPerSecond.of(
                            0.1
                                    + intakeArmFeedforward.calculate(
                                            intakeArmAbsoluteEncoder.getPosition(), 0.1));
            intakeArmSparkFlex.set(desiredSpeed.in(RotationsPerSecond));

        } else {
            // intakeArmClosedLoop.setSetpoint(0, ControlType.kVelocity);
            AngularVelocity desiredSpeed =
                    RadiansPerSecond.of(
                            0
                                    + intakeArmFeedforward.calculate(
                                            intakeArmAbsoluteEncoder.getPosition(), 0));
            intakeArmSparkFlex.set(desiredSpeed.in(RotationsPerSecond));
        }
    }

    @Override
    public void updateState(IntakeArmIOState state) {
        // if (TUNABLE_INTAKE_KCOS.hasChanged()) {
        //     intakeArmClosedLoopConfig.feedForward.kCos(TUNABLE_INTAKE_KCOS.get());
        //     System.out.println("allegedly changing ks to " + TUNABLE_INTAKE_KCOS.get());
        //     intakeArmSparkFlexConfig.apply(intakeArmClosedLoopConfig);
        //     intakeArmSparkFlex.configure(
        //             intakeArmSparkFlexConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }

        state.desiredAngleDegrees = desiredAngle.in(Degrees);
        state.actualAngleDegrees = intakeArmAbsoluteEncoder.getPosition() * (180 / Math.PI);
        state.velocityDegreesPerSecond = intakeArmAbsoluteEncoder.getVelocity() * (180 / Math.PI);
        state.appliedVoltage =
                intakeArmSparkFlex.getBusVoltage() * intakeArmSparkFlex.getAppliedOutput();
        state.current = intakeArmSparkFlex.getOutputCurrent();
    }
}
