package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants.MotorConstants;

public class IntakeHardware implements IntakeIO {

    private SparkFlex intakeSparkFlex;
    private final SparkClosedLoopController intakePidController;
    
    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2*Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2*Math.PI/60);

    public IntakeHardware(){
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.inverted(true);
        intakeMotorConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
        
        intakeMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
        intakeMotorConfig.encoder.velocityConversionFactor(ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        intakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeMotorConfig.closedLoop.pid(1, 0, 0);
        intakeMotorConfig.closedLoop.outputRange(-1,1);


        intakeSparkFlex = new SparkFlex(6, MotorType.kBrushless);
        intakeSparkFlex.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakePidController = intakeSparkFlex.getClosedLoopController();

    }

    public void runIntake(){
        intakePidController.setSetpoint(0.5*MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void setZero(){
        intakePidController.setSetpoint(0, ControlType.kVelocity);
    }
}