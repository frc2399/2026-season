package frc.robot.subsystems.shooter;

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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class ShooterHardwarePrototype implements ShooterIO {
    private SparkFlex shooterSparkFlex;
    private SparkMax shooterSparkMax;
    private SparkClosedLoopController shooterPidController;
    
        private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
        private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
        private final int MIN_OUTPUT_RANGE = -1;
        private final int MAX_OUTPUT_RANGE = 1;
        private final double SHOOTER_P = 0;
        private final double SHOOTER_D = 0;
        private final double SHOOTER_KS = 0.1;
        private final double SHOOTER_KV =
                12 / RobotConstants.MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond);
    
        private final ClosedLoopConfig closedLoopConfigShooter = new ClosedLoopConfig();
        private RelativeEncoder shooterEncoder;
            
                public ShooterHardwarePrototype() {
                    SparkFlexConfig shooterBottomMotorConfig = new SparkFlexConfig();
                    SparkMaxConfig shooterTopMotorConfig = new SparkMaxConfig();
            
                    shooterBottomMotorConfig.idleMode(IdleMode.kBrake);
                    shooterTopMotorConfig.idleMode(IdleMode.kBrake);
                    shooterBottomMotorConfig.inverted(true);
                    shooterTopMotorConfig.inverted(true);
                    shooterBottomMotorConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
                    shooterTopMotorConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
            
            
                    shooterBottomMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
                    shooterTopMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));
            
                    shooterBottomMotorConfig.encoder.velocityConversionFactor(
                            ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));
                    shooterTopMotorConfig.encoder.velocityConversionFactor(
                            ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));
            
                    shooterBottomMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                    shooterTopMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                    shooterBottomMotorConfig.closedLoop.pid(SHOOTER_P, 0, SHOOTER_D);
                    shooterTopMotorConfig.closedLoop.pid(SHOOTER_P, 0, SHOOTER_D);
                    shooterBottomMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
                    shooterTopMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
            
            
                    closedLoopConfigShooter.feedForward.sva(SHOOTER_KS, SHOOTER_KV, 0);
            
                    shooterBottomMotorConfig.apply(closedLoopConfigShooter);
                    shooterTopMotorConfig.apply(closedLoopConfigShooter);
            
            
                    shooterSparkFlex = new SparkFlex(RobotConstants.MotorIdConstants.SHOOTER_BOTTOM_CAN_ID, MotorType.kBrushless);
                    shooterSparkMax = new SparkMax(RobotConstants.MotorIdConstants.SHOOTER_TOP_CAN_ID, MotorType.kBrushless);
                    shooterSparkFlex.configure(
                            shooterBottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                    shooterSparkMax.configure(
                            shooterTopMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
                    shooterPidController = shooterSparkFlex.getClosedLoopController();
                    shooterPidController = shooterSparkMax.getClosedLoopController();
        
                shooterEncoder = shooterSparkFlex.getEncoder();
                shooterEncoder = shooterSparkMax.getEncoder();
    }

    public void runShooter() {
        shooterPidController.setSetpoint(
                0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond), ControlType.kVelocity);

        SmartDashboard.putNumber(
                "Shooter/desiredspeed",
                0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        SmartDashboard.putNumber("Shooter/actualspeed", shooterEncoder.getVelocity());
    }

    public void defaultBehavior() {
        shooterPidController.setSetpoint(0, ControlType.kVelocity);

        SmartDashboard.putNumber("Intake/desiredspeed", 0);
        SmartDashboard.putNumber("Intake/actualspeed", shooterEncoder.getVelocity());
    }
}