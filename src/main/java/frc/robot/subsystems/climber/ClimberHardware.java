package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
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
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberHardware implements ClimberIO {
    private SparkFlex climberSparkFlex;
    private final SparkClosedLoopController climberPidController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    //GEAR REDUCTION for climber motor:
    //1:1 brake, 5:1, 5:1 --> 25 reduction
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final int MIN_OUTPUT_RANGE = -1;
    private final int MAX_OUTPUT_RANGE = 1;

    private static final double DEFAULT_CLIMBER_P = 0;
    private static final double DEFAULT_CLIMBER_D = 0;
    private static final double DEFAULT_CLIMBER_KS = 0;
    private static final double DEFAULT_CLIMBER_KV = 
            (12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));

    private static final Distance MAX_CLIMBER_EXTENSION = Inches.of(28.75);



    private SparkFlexConfig climberMotorConfig = new SparkFlexConfig();
    private final ClosedLoopConfig closedLoopConfigClimber = new ClosedLoopConfig();
    private final RelativeEncoder climberEncoder;

    public ClimberHardware() {
        SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

        climberMotorConfig.inverted(false);
        climberMotorConfig.idleMode(IdleMode.kCoast);
        climberMotorConfig.smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        climberMotorConfig.voltageCompensation(12);

        climberMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
        climberMotorConfig.encoder.velocityConversionFactor(ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        climberMotorConfig.closedLoop.pid(DEFAULT_CLIMBER_P, 0, DEFAULT_CLIMBER_D);
        climberMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        climberMotorConfig.softLimit.forwardSoftLimit(MAX_CLIMBER_EXTENSION.in(Inches));
        climberMotorConfig.softLimit.reverseSoftLimit(0);

        closedLoopConfigClimber.feedForward.sva(DEFAULT_CLIMBER_KS, DEFAULT_CLIMBER_KV, 0);

        climberMotorConfig.apply(closedLoopConfigClimber);

        climberSparkFlex =
                new SparkFlex(RobotConstants.MotorIdConstants.CLIMBER_CAN_ID, MotorType.kBrushless);
        climberSparkFlex.configure(
                climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberPidController = climberSparkFlex.getClosedLoopController();

        climberEncoder = climberSparkFlex.getEncoder();
    }

    public void extend() {
        climberPidController.setSetpoint(MAX_CLIMBER_EXTENSION.in(Inches), ControlType.kPosition);
    }

    public void retract() {
        climberPidController.setSetpoint(0, ControlType.kPosition);
    }

    public void runForwards() {
        climberPidController.setSetpoint(
                0.75 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond),
                ControlType.kVelocity);
    }

    public void runBackwards() {
        climberPidController.setSetpoint(
                0.75 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond),
                ControlType.kVelocity);
    }
    
}
