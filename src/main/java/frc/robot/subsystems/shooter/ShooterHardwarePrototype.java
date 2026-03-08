package frc.robot.subsystems.shooter;

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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.MotorConstants;

public class ShooterHardwarePrototype implements ShooterIO {
    private SparkFlex shooterBottomSparkFlex;
    private SparkMax shooterTopSparkMax;
    private SparkClosedLoopController shooterBottomPIDController;
    private SparkClosedLoopController shooterTopPIDController;

    private final Angle ENCODER_POSITION_FACTOR = Radians.of(2 * Math.PI);
    private final AngularVelocity ENCODER_VELOCITY_FACTOR = RadiansPerSecond.of(2 * Math.PI / 60);
    private final double MIN_OUTPUT_RANGE = -1;
    private final double MAX_OUTPUT_RANGE = 1;
    private final double SHOOTER_TOP_P = 0.001;
    private final double SHOOTER_TOP_D = 0;
    private final double SHOOTER_TOP_KS = 0.118;
    private final double SHOOTER_TOP_KV = 0.01700044443192286;
    private final double SHOOTER_BOTTOM_P = 0.001;
    private final double SHOOTER_BOTTOM_D = 0;
    private final double SHOOTER_BOTTOM_KS = 0.154;
    private final double SHOOTER_BOTTOM_KV = 0.019691444431922856;

    // tunable numbers, supposed to be saved
    // private final TunableNumber TUNABLE_SHOOTER_TOP_D =
    // new TunableNumber("Shooter/shooter_top_p", SHOOTER_TOP_D, true);
    //     private final TunableNumber TUNABLE_SHOOTER_TOP_P =
    //             new TunableNumber("Shooter/shooter_top_p", .001, true);
    //     private final TunableNumber TUNABLE_SHOOTER_TOP_KS =
    //             new TunableNumber("Shooter/shooter_top_ks", 0.118, true);
    //     private final TunableNumber TUNABLE_SHOOTER_TOP_KV =
    //             new TunableNumber("Shooter/shooter_top_kv", 0.01700044443192286, true);
    // private final TunableNumber TUNABLE_SHOOTER_BOTTOM_D =
    // new TunableNumber("Shooter/shooter_bottom_p", SHOOTER_BOTTOM_D, true);
    //     private final TunableNumber TUNABLE_SHOOTER_BOTTOM_P =
    //             new TunableNumber("Shooter/shooter_bottom_p", .001, true);
    //     private final TunableNumber TUNABLE_SHOOTER_BOTTOM_KS =
    //             new TunableNumber("Shooter/shooter_bottom_ks", 0.154, true);
    //     private final TunableNumber TUNABLE_SHOOTER_BOTTOM_KV =
    //             new TunableNumber("Shooter/shooter_bottom_kv", 0.019691444431922856, true);

    private final ClosedLoopConfig closedLoopConfigShooterTop = new ClosedLoopConfig();
    private final ClosedLoopConfig closedLoopConfigShooterBottom = new ClosedLoopConfig();

    private RelativeEncoder shooterBottomEncoder;
    private RelativeEncoder shooterTopEncoder;

    private AngularVelocity desiredBottomVelocity = RadiansPerSecond.of(0);
    private AngularVelocity desiredTopVelocity = RadiansPerSecond.of(0);

    private SparkFlexConfig shooterBottomMotorConfig = new SparkFlexConfig();
    private SparkMaxConfig shooterTopMotorConfig = new SparkMaxConfig();

    public ShooterHardwarePrototype() {

        shooterBottomMotorConfig.idleMode(IdleMode.kCoast);
        shooterTopMotorConfig.idleMode(IdleMode.kCoast);
        shooterBottomMotorConfig.inverted(true);
        shooterTopMotorConfig.inverted(true);
        shooterBottomMotorConfig.smartCurrentLimit(
                (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        shooterTopMotorConfig.smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

        shooterBottomMotorConfig.encoder.positionConversionFactor(
                ENCODER_POSITION_FACTOR.in(Radians));
        shooterTopMotorConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR.in(Radians));

        shooterBottomMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));
        shooterTopMotorConfig.encoder.velocityConversionFactor(
                ENCODER_VELOCITY_FACTOR.in(RadiansPerSecond));

        shooterBottomMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterTopMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterBottomMotorConfig.closedLoop.pid(SHOOTER_BOTTOM_P, 0, SHOOTER_BOTTOM_D);
        shooterTopMotorConfig.closedLoop.pid(SHOOTER_TOP_P, 0, SHOOTER_TOP_D);
        shooterBottomMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);
        shooterTopMotorConfig.closedLoop.outputRange(MIN_OUTPUT_RANGE, MAX_OUTPUT_RANGE);

        closedLoopConfigShooterTop.feedForward.sva(SHOOTER_TOP_KS, SHOOTER_TOP_KV, 0);
        closedLoopConfigShooterBottom.feedForward.sva(SHOOTER_BOTTOM_KS, SHOOTER_BOTTOM_KV, 0);

        shooterBottomMotorConfig.apply(closedLoopConfigShooterBottom);
        shooterTopMotorConfig.apply(closedLoopConfigShooterTop);

        shooterBottomSparkFlex =
                new SparkFlex(
                        RobotConstants.MotorIdConstants.SHOOTER_BOTTOM_CAN_ID,
                        MotorType.kBrushless);
        shooterTopSparkMax =
                new SparkMax(
                        RobotConstants.MotorIdConstants.SHOOTER_TOP_CAN_ID, MotorType.kBrushless);

        var shooterBottomStatus =
                shooterBottomSparkFlex.configure(
                        shooterBottomMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
        var shooterTopStatus =
                shooterTopSparkMax.configure(
                        shooterTopMotorConfig,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        shooterBottomPIDController = shooterBottomSparkFlex.getClosedLoopController();
        shooterTopPIDController = shooterTopSparkMax.getClosedLoopController();

        shooterBottomEncoder = shooterBottomSparkFlex.getEncoder();
        shooterTopEncoder = shooterTopSparkMax.getEncoder();

        if (shooterBottomStatus != REVLibError.kOk) {
            System.err.println("Failed to configure shooter bottom motor: " + shooterBottomStatus);
        }
        if (shooterBottomStatus != REVLibError.kOk) {
            System.err.println("Failed to configure shooter top motor: " + shooterBottomStatus);
        }

        if (shooterTopStatus != REVLibError.kOk) {
            System.err.println("Failed to configure shooter bottom motor: " + shooterTopStatus);
        }
        if (shooterTopStatus != REVLibError.kOk) {
            System.err.println("Failed to configure shooter top motor: " + shooterTopStatus);
        }
    }

    public void runShooter() {
        desiredBottomVelocity =
                RadiansPerSecond.of(0.5 * MotorConstants.VORTEX_FREE_SPEED.in(RadiansPerSecond));
        desiredTopVelocity =
                RadiansPerSecond.of(0.5 * MotorConstants.NEO_FREE_SPEED.in(RadiansPerSecond));

        shooterBottomPIDController.setSetpoint(
                desiredBottomVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        shooterTopPIDController.setSetpoint(
                desiredTopVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public void defaultBehavior() {
        desiredBottomVelocity = RadiansPerSecond.of(0);
        desiredTopVelocity = RadiansPerSecond.of(0);

        shooterBottomPIDController.setSetpoint(
                desiredBottomVelocity.in(RadiansPerSecond), ControlType.kVelocity);
        shooterTopPIDController.setSetpoint(
                desiredTopVelocity.in(RadiansPerSecond), ControlType.kVelocity);
    }

    public boolean isUpToSpeed() {
        boolean isTopRollerDesiredSpeed =
                Math.abs(
                                (shooterTopEncoder.getVelocity())
                                        - (desiredTopVelocity.in(RadiansPerSecond)))
                        < 25;
        boolean isBottomRollerDesiredSpeed =
                Math.abs(
                                (shooterBottomEncoder.getVelocity())
                                        - (desiredBottomVelocity.in(RadiansPerSecond)))
                        < 25;

        return isTopRollerDesiredSpeed && isBottomRollerDesiredSpeed;
    }

    @Override
    public ShooterSpeeds getCurrentTopAndBottomSpeeds() {
        return new ShooterSpeeds(0, 0);
    }

    public void updateStates(ShooterIOState state) {
        state.topRollerDesiredSpeed = desiredTopVelocity.in(RadiansPerSecond);
        state.topRollerActualSpeed = shooterTopEncoder.getVelocity();
        state.topRollerCurrent = shooterTopSparkMax.getOutputCurrent();
        state.topRollerAppliedVoltage =
                shooterTopSparkMax.getAppliedOutput() * shooterTopSparkMax.getBusVoltage();
        state.bottomRollerDesiredSpeed = desiredBottomVelocity.in(RadiansPerSecond);
        state.bottomRollerCurrent = shooterBottomSparkFlex.getOutputCurrent();
        state.bottomRollerActualSpeed = shooterBottomEncoder.getVelocity();
        state.bottomRollerAppliedVoltage =
                shooterBottomSparkFlex.getAppliedOutput() * shooterBottomSparkFlex.getBusVoltage();
    }

    public void periodicUpdate() {
        // if tuning a value, update this chunk for that motor's p, i, OR d
        // attempting to have this logic running with multiple causes a loop overrun :)
        // if (TUNABLE_SHOOTER_BOTTOM_KS.hasChanged()) {
        //     closedLoopConfigShooterBottom.feedForward.kS(TUNABLE_SHOOTER_BOTTOM_KS.get());
        //     shooterBottomMotorConfig.apply(closedLoopConfigShooterBottom);
        //     shooterBottomSparkFlex.configure(
        //             shooterBottomMotorConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }
        // if (TUNABLE_SHOOTER_TOP_KS.hasChanged()) {
        //     closedLoopConfigShooterTop.feedForward.kS(TUNABLE_SHOOTER_TOP_KS.get());
        //     shooterTopMotorConfig.apply(closedLoopConfigShooterTop);
        //     shooterTopSparkMax.configure(
        //             shooterTopMotorConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }
        // if (TUNABLE_SHOOTER_BOTTOM_KV.hasChanged()) {
        //     closedLoopConfigShooterBottom.feedForward.kV(TUNABLE_SHOOTER_BOTTOM_KV.get());
        //     shooterBottomMotorConfig.apply(closedLoopConfigShooterBottom);
        //     shooterBottomSparkFlex.configure(
        //             shooterBottomMotorConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }
        // if (TUNABLE_SHOOTER_TOP_KV.hasChanged()) {
        //     closedLoopConfigShooterTop.feedForward.kV(TUNABLE_SHOOTER_TOP_KV.get());
        //     shooterTopMotorConfig.apply(closedLoopConfigShooterTop);
        //     shooterTopSparkMax.configure(
        //             shooterTopMotorConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }
        // if (TUNABLE_SHOOTER_BOTTOM_P.hasChanged()) {
        //     shooterBottomMotorConfig.closedLoop.p(TUNABLE_SHOOTER_BOTTOM_P.get());
        //     //     shooterBottomMotorConfig.apply(closedLoopConfigShooterBottom);
        //     shooterBottomSparkFlex.configure(
        //             shooterBottomMotorConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }
        // if (TUNABLE_SHOOTER_TOP_P.hasChanged()) {
        //     shooterTopMotorConfig.closedLoop.p(TUNABLE_SHOOTER_TOP_P.get());
        //     //     shooterTopMotorConfig.apply(closedLoopConfigShooterTop);
        //     shooterTopSparkMax.configure(
        //             shooterTopMotorConfig,
        //             ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }
    }

    @Override
    public void runTunableNumberSetpoints() {}
}
