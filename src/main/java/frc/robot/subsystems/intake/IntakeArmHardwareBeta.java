package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;

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
    private static final boolean INTAKE_ARM_MOTOR_INVERTED = false;
    private static final boolean INTAKE_ARM_ENCODER_INVERTED = false;

    // conversion factors (45:1 is our gear ratio)
    private static final Angle INTAKE_ARM_POSITION_CONVERSION_FACTOR =
            Degrees.of(360.0); // this subsystem is in degrees because it's a rotational wrist,
    // and this makes it easier to tune :)
    private static final AngularVelocity INTAKE_ARM_VELOCITY_CONVERSION_FACTOR =
            DegreesPerSecond.of(
                    360.0 / 60); // math more explicit to make conversion more understandable

    // pid
    private static final double INTAKE_ARM_P = 0.00;
    private static final double INTAKE_ARM_I = 0;
    private static final double INTAKE_ARM_D = 0;
    // feedforward
    private static final double INTAKE_ARM_KS = 0;
    private static final double INTAKE_ARM_KV = 0.0; // .25
    //     12 / (RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(DegreesPerSecond) / 45.0);
    private static final double INTAKE_ARM_KA = 0;
    /*
     * our arm is rotational, so the impact of gravity changes as we rotate, and our feedforward needs to compensate. the kcos is the factor to compensate by
     * revlib demands that it can get from what we're logging in (degree/s) to rotations of mechanism / second to accurately compensate. that's kcosratio
     */
    private static final double INTAKE_ARM_KCOS = 3;
    private static final double INTAKE_ARM_KCOS_RATIO =
            1 / 360; // convert from mechanism degrees to rotation

    // output
    private static final double INTAKE_ARM_MIN_OUTPUT = -1;
    private static final double INTAKE_ARM_MAX_OUTPUT = 1;

    // soft limits
    private static final Angle INTAKE_ARM_FORWARD_MAX_ANGLE = Degrees.of(80);
    private static final boolean INTAKE_ARM_FORWARD_SOFTLIMIT_ENABLED = false;
    private static final Angle INTAKE_ARM_REVERSE_MAX_ANGLE = Degrees.of(10);
    private static final boolean INTAKE_ARM_REVERSE_SOFTLIMIT_ENABLED = false;

    private Angle desiredAngle = Degrees.of(0);

    // tunable numbers - for testing only! delete before pr
    //     private double DEFAULT_KS = .001;
    //     private TunableNumber TUNABLE_KS = new TunableNumber("intake/arm/voltage", DEFAULT_KS,
    // true);

    public IntakeArmHardwareBeta() {
        intakeArmSparkFlexConfig
                .inverted(INTAKE_ARM_MOTOR_INVERTED)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(
                        (int) RobotConstants.MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        intakeArmSparkFlexConfig
                .absoluteEncoder
                .inverted(INTAKE_ARM_ENCODER_INVERTED)
                .positionConversionFactor(INTAKE_ARM_POSITION_CONVERSION_FACTOR.in(Degrees))
                .velocityConversionFactor(
                        INTAKE_ARM_VELOCITY_CONVERSION_FACTOR.in(DegreesPerSecond));

        intakeArmSparkFlexConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(INTAKE_ARM_P, INTAKE_ARM_I, INTAKE_ARM_D)
                .outputRange(INTAKE_ARM_MIN_OUTPUT, INTAKE_ARM_MAX_OUTPUT)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(-180, 180);

        // soft limits are code-enforced limits on where the mechanism can go
        // they're called SOFT limits because the mechanism can still technically go past it
        // if the mechanism can't physically go past it, that's a HARD limit/stop
        intakeArmSparkFlexConfig
                .softLimit
                .forwardSoftLimit(INTAKE_ARM_FORWARD_MAX_ANGLE.in(Degrees))
                .forwardSoftLimitEnabled(INTAKE_ARM_FORWARD_SOFTLIMIT_ENABLED)
                .reverseSoftLimit(INTAKE_ARM_REVERSE_MAX_ANGLE.in(Degrees))
                .reverseSoftLimitEnabled(INTAKE_ARM_REVERSE_SOFTLIMIT_ENABLED);

        intakeArmClosedLoopConfig.feedForward.svacr(
                INTAKE_ARM_KS,
                INTAKE_ARM_KV,
                INTAKE_ARM_KA,
                INTAKE_ARM_KCOS,
                INTAKE_ARM_KCOS_RATIO);

        intakeArmSparkFlexConfig.apply(intakeArmClosedLoopConfig);

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
        // if (setpoint == IntakeArmSetpoint.DEPLOYED) {
        //     //     desiredAngle = Degrees.of(-10);
        //     intakeArmClosedLoop.setSetpoint(-6, ControlType.kVelocity);
        //     //     intakeArmClosedLoop.setSetpoint(-10,ControlType.kPosition);
        // } else if (setpoint == IntakeArmSetpoint.STOWED) {
        //     //     desiredAngle = Degrees.of(80);
        //     intakeArmClosedLoop.setSetpoint(6, ControlType.kVelocity);
        //     //     intakeArmClosedLoop.setSetpoint(80, ControlType.kPosition);
        // }
        if (setpoint == IntakeArmSetpoint.DEPLOYED) {
            System.out.println("hi");
            intakeArmClosedLoop.setSetpoint(6, ControlType.kVelocity);
        } else if (setpoint == IntakeArmSetpoint.STOWED) {
            intakeArmClosedLoop.setSetpoint(-6, ControlType.kVelocity);
        } else {
            intakeArmClosedLoop.setSetpoint(0, ControlType.kVelocity);
        }
    }

    @Override
    public void runOffVolts() {
        System.out.println("im a volt!");
        intakeArmClosedLoop.setSetpoint(1, ControlType.kVoltage);
        // for testing, add backward voltage setpoint bc idk how to do that rn :)
    }

    @Override
    public void updateState(IntakeArmIOState state) {
        // if (TUNABLE_KS.hasChanged()) {
        //     intakeArmClosedLoopConfig.feedForward.kS(TUNABLE_KS.get());
        //     System.out.println("allegedly changing ks to " + TUNABLE_KS.get());
        //     intakeArmSparkFlexConfig.apply(intakeArmClosedLoopConfig);
        //     intakeArmSparkFlex.configure(
        //             intakeArmSparkFlexConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kNoPersistParameters);
        // }

        state.desiredAngleDegrees = desiredAngle.in(Degrees);
        state.actualAngleDegrees = intakeArmAbsoluteEncoder.getPosition();
        state.velocityDegreesPerSecond = intakeArmAbsoluteEncoder.getVelocity();
        state.appliedVoltage =
                intakeArmSparkFlex.getBusVoltage() * intakeArmSparkFlex.getAppliedOutput();
        state.current = intakeArmSparkFlex.getOutputCurrent();
    }
}
