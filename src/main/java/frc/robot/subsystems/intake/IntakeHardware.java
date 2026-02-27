package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class IntakeHardware implements IntakeIO {

    private SparkFlex intakeSparkFlex;
    private final SparkClosedLoopController intakePidController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;
    private final double INTAKE_D = 0.0;

    private static final double DEFAULT_INTAKE_P = 0.001;
    private static final double DEFAULT_INTAKE_KS = 0.1;
    private static final double DEFAULT_INTAKE_KV =
            12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);

    // private static final TunableNumber TUNABLE_INTAKE_P =
    // new TunableNumber("Intake/intake_p", DEFAULT_INTAKE_P, true);
    // private static final TunableNumber TUNABLE_INTAKE_KS =
    // new TunableNumber("Intake/intake_ks", DEFAULT_INTAKE_KS, true);
    // private static final TunableNumber TUNABLE_INTAKE_KV =
    // new TunableNumber("Intake/intake_kv", DEFAULT_INTAKE_KV, true);

    private final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
    private final ClosedLoopConfig closedLoopConfigIntake = new ClosedLoopConfig();
    private final RelativeEncoder intakeEncoder;

    public IntakeHardware() {
        SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

        intakeMotorConfig.idleMode(IdleMode.kCoast);
        intakeMotorConfig.inverted(true);
        intakeMotorConfig.smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        intakeMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
        intakeMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        intakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeMotorConfig.closedLoop.pid(DEFAULT_INTAKE_P, 0, INTAKE_D);
        intakeMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        closedLoopConfigIntake.feedForward.sva(DEFAULT_INTAKE_KS, DEFAULT_INTAKE_KV, 0);

        intakeMotorConfig.apply(closedLoopConfigIntake);

        intakeSparkFlex =
                new SparkFlex(RobotConstants.MotorIdConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        intakeSparkFlex.configure(
                intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakePidController = intakeSparkFlex.getClosedLoopController();

        intakeEncoder = intakeSparkFlex.getEncoder();
    }

    public void runIntake() {
        intakePidController.setSetpoint(
                0.75 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond),
                ControlType.kVelocity);

        SmartDashboard.putNumber(
                "Intake/desiredspeed",
                0.75 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        SmartDashboard.putNumber("Intake/actualspeed", intakeEncoder.getVelocity());
    }

    public void setZero() {
        intakePidController.setSetpoint(0, ControlType.kVelocity);

        SmartDashboard.putNumber("Intake/desiredspeed", 0);
        SmartDashboard.putNumber("Intake/actualspeed", intakeEncoder.getVelocity());
    }

    public void periodicUpdate() {
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
