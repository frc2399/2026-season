package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants.RobotConstants;

public class IntakeArmHardwareBeta implements IntakeArmIO{
    //45:1 ratio 
    //sparkflex
    //vortex
    //abs enc
    //1:1 AFTER enc :)))
    private final SparkFlex intakeArmSparkFlex;
    private final AbsoluteEncoder intakeArmAbsoluteEncoder;
    private final SparkClosedLoopController intakeArmClosedLoop;

    private final SparkFlexConfig intakeArmSparkFlexConfig = new SparkFlexConfig();
    private final ClosedLoopConfig intakeArmClosedLoopConfig = new ClosedLoopConfig();

    public IntakeArmHardwareBeta() {
        intakeArmSparkFlexConfig.inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) RobotConstants.MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

        intakeArmSparkFlexConfig.absoluteEncoder.inverted(false)
            .positionConversionFactor(0)
            .velocityConversionFactor(0);

        intakeArmSparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0, 0, 0)
            .outputRange(0, 0);

        // soft limits are code-enforced limits on where the mechanism can go
        // they're called SOFT limits because the mechanism can still technically go past it
        // if the mechanism can't physically go past it, that's a HARD limit/stop
        intakeArmSparkFlexConfig.softLimit.forwardSoftLimit(0)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimit(0)
            .reverseSoftLimitEnabled(false);

        intakeArmClosedLoopConfig.feedForward.sva(0,0,0);

        intakeArmSparkFlexConfig.apply(intakeArmClosedLoopConfig);

        intakeArmSparkFlex = new SparkFlex(RobotConstants.MotorIdConstants.INTAKE_ARM_CAN_ID, MotorType.kBrushless);

        intakeArmSparkFlex.configure(intakeArmSparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeArmAbsoluteEncoder = intakeArmSparkFlex.getAbsoluteEncoder();
        intakeArmClosedLoop = intakeArmSparkFlex.getClosedLoopController();
    }

    @Override
    public void setSetpoint(IntakeArmSetpoint setpoint) {
        
    }
}
