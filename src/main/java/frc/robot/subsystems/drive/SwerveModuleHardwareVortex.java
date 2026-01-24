package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.util.TunableNumber;

public class SwerveModuleHardwareVortex implements SwerveModuleIO {

    private SparkFlex drivingSparkFlex;
    private SparkMax turningSparkMax;

    private final RelativeEncoder drivingRelativeEncoder;
    private final SparkAbsoluteEncoder turningAbsoluteEncoder;

    private final SparkClosedLoopController drivingPidController;
    private final SparkClosedLoopController turningPidController;

    private double chassisAngularOffset;
    private String name;
    private double desiredAngle;
    private double driveDesiredVelocity;
    private static final SparkFlexConfig sparkFlexConfigDriving = new SparkFlexConfig();
    private static final ClosedLoopConfig sparkFlexClosedLoopConfigDriving = new ClosedLoopConfig();
    private static final SparkMaxConfig sparkMaxConfigTurning = new SparkMaxConfig();
    private static final ClosedLoopConfig sparkMaxClosedLoopConfigTurning = new ClosedLoopConfig();

    // drivings are NEO Vortex, turnings are NEO 550s
    private static final int DRIVING_MOTOR_PINION_TEETH = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    private static final boolean TURNING_ENCODER_INVERTED = true;
    private static final boolean DRIVING_MOTOR_INVERTED = false;
    private static final boolean TURNING_MOTOR_INVERTED = false;

    // Calculations required for driving motor conversion factors and feed forward
    private static final Distance WHEEL_DIAMETER = Inches.of(3);
    private static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // This is also the gear ratio (14T)

    private static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);

    private static final LinearVelocity DRIVE_WHEEL_FREE_SPEED = MetersPerSecond
            .of((MotorConstants.VORTEX_FREE_SPEED.in(RotationsPerSecond) *
                    WHEEL_CIRCUMFERENCE.in(Meters)) / (DRIVING_MOTOR_REDUCTION));

    private static final Distance DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER.times(Math.PI))
            .div(DRIVING_MOTOR_REDUCTION); // meters
    private static final Distance DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR.div(60); // meters
                                                                                                             // per
                                                                                                             // second

    private static final double TURNING_ENCODER_POSITION_FACTOR = Units.rotationsToRadians(1);
    private static final double TURNING_ENCODER_VELOCITY_FACTOR = Units.rotationsToRadians(1) / 60.0;

    private static final boolean TURNING_ENCODER_POSITION_WRAPPING = true;
    private static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    private static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

    private static final double DEFAULT_DRIVING_P = 0.15;
    private static final double DEFAULT_DRIVING_I = 0;
    private static final double DEFAULT_DRIVING_D = 0.1;
    private static final TunableNumber TUNABLE_DRIVING_P = new TunableNumber("Swerve/driving_p", DEFAULT_DRIVING_P, true);
    private static final TunableNumber TUNABLE_DRIVING_D = new TunableNumber("Swerve/driving_d", DEFAULT_DRIVING_D, true);
    private static final double DRIVING_KS = 0.1;
    // private static final double DRIVING_KS = (0.35944 + 0.36478 + 0.36578 + 0.34661)/4;
    private static final double DRIVING_KV = (2.29053 + 2.28747 + 2.28556 + 2.29925)/4; 
    private static final double DRIVING_KA = 0;
    private static final double DRIVING_MIN_OUTPUT = -1;
    private static final double DRIVING_MAX_OUTPUT = 1;

    private static final double DEFAULT_TURNING_P = 1.75;
    private static final double DEFAULT_TURNING_I = 0;
    private static final double DEFAULT_TURNING_D = 0.016;

    private static final TunableNumber TUNABLE_TURNING_P = new TunableNumber("Swerve/turning_p", DEFAULT_TURNING_P,
            true);
    private static final TunableNumber TUNABLE_TURNING_I = new TunableNumber("Swerve/turning_i", DEFAULT_TURNING_I,
            true);
    private static final TunableNumber TUNABLE_TURNING_D = new TunableNumber("Swerve/turning_d", DEFAULT_TURNING_D,
            true);

    private static final double TURNING_FF = 0;
    private static final double TURNING_MIN_OUTPUT = -1;
    private static final double TURNING_MAX_OUTPUT = 1;

    private static final double VOLTAGE_COMPENSATION = 12;

    private static final SparkBaseConfig.IdleMode DRIVING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final SparkBaseConfig.IdleMode TURNING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

    public SwerveModuleHardwareVortex(int drivingCanId, int turningCanId, double chassisAngularOffset, String name) {
        this.chassisAngularOffset = chassisAngularOffset;
        this.name = name;
        drivingSparkFlex = new SparkFlex(drivingCanId, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);

        sparkFlexConfigDriving.inverted(DRIVING_MOTOR_INVERTED).idleMode(DRIVING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(
                        (int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(VOLTAGE_COMPENSATION);
        sparkFlexConfigDriving.encoder.positionConversionFactor(DRIVING_ENCODER_POSITION_FACTOR.in(Meters))
                .velocityConversionFactor(DRIVING_ENCODER_VELOCITY_FACTOR.in(Meters));
        sparkFlexConfigDriving.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(DRIVING_MIN_OUTPUT, DRIVING_MAX_OUTPUT);

        sparkFlexClosedLoopConfigDriving.pid(TUNABLE_DRIVING_P.get(), DEFAULT_DRIVING_I, TUNABLE_DRIVING_D.get()).feedForward.sva(DRIVING_KS, DRIVING_KV,
                DRIVING_KA);

        sparkFlexConfigDriving.apply(sparkFlexClosedLoopConfigDriving);

        sparkMaxConfigTurning.inverted(TURNING_MOTOR_INVERTED).idleMode(TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit(
                        (int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(VOLTAGE_COMPENSATION);
        sparkMaxConfigTurning.absoluteEncoder.positionConversionFactor(TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);
        sparkMaxConfigTurning.absoluteEncoder.inverted(TURNING_ENCODER_INVERTED);
        sparkMaxConfigTurning.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .outputRange(TURNING_MIN_OUTPUT, TURNING_MAX_OUTPUT)
                .positionWrappingEnabled(TURNING_ENCODER_POSITION_WRAPPING)
                .positionWrappingInputRange(
                        TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        TURNING_ENCODER_POSITION_PID_MAX_INPUT);
        sparkMaxConfigTurning.signals.absoluteEncoderPositionPeriodMs(Constants.SpeedConstants.MAIN_LOOP_FREQUENCY_MS);

        sparkMaxClosedLoopConfigTurning.pid(TUNABLE_TURNING_P.get(), TUNABLE_TURNING_I.get(),
                TUNABLE_TURNING_D.get()).feedForward.sva(0, TURNING_FF, 0);

        sparkFlexConfigDriving.apply(sparkFlexClosedLoopConfigDriving);
        sparkMaxConfigTurning.apply(sparkMaxClosedLoopConfigTurning);

        sparkFlexConfigDriving.apply(sparkFlexClosedLoopConfigDriving);
        sparkMaxConfigTurning.apply(sparkMaxClosedLoopConfigTurning);

        drivingSparkFlex.configure(sparkFlexConfigDriving, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        turningSparkMax.configure(sparkMaxConfigTurning, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        drivingRelativeEncoder = drivingSparkFlex.getEncoder();
        turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder();

        drivingPidController = drivingSparkFlex.getClosedLoopController();
        turningPidController = turningSparkMax.getClosedLoopController();
    }

    public void setDriveEncoderPosition(double position) {
        drivingRelativeEncoder.setPosition(position);
    };

    public double getDriveEncoderPosition() {
        double driveEncoderPosition = drivingRelativeEncoder.getPosition();
        if (Double.isNaN(driveEncoderPosition)) {
            return 0.0;
        } else {
            return driveEncoderPosition;
        }

    };

    public void setDesiredDriveSpeedMPS(double speed) {
        drivingPidController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        this.driveDesiredVelocity = speed;
    };

    public double getDriveEncoderSpeedMPS() {
        double driveVelocity = drivingRelativeEncoder.getVelocity();
        if (Double.isNaN(driveVelocity)) {
            return 0.0;
        } else {
            return driveVelocity;
        }
    };

    public double getTurnEncoderPosition() {
        double drivePosition = turningAbsoluteEncoder.getPosition();

        if (Double.isNaN(drivePosition)) {
            return 0.0;
        } else {
            return drivePosition;
        }
    };

    public void setDesiredTurnAngle(double angle) {
        turningPidController.setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        this.desiredAngle = angle;

        //  double setpoint = MathUtil.inputModulus(
        //         rotation.plus(Rotation2d.fromRadians(chassisAngularOffset)).getRadians(), TURNING_ENCODER_POSITION_PID_MIN_INPUT, TURNING_ENCODER_POSITION_PID_MAX_INPUT);
        // turningPidController.setSetpoint(setpoint, ControlType.kPosition);
    };


    public double getChassisAngularOffset() {
        return chassisAngularOffset;
    }

    public Distance getWheelDiameter() {
        return WHEEL_DIAMETER;
    }

    public double getTurnAngleCharacterization() {
        return getTurnEncoderPosition() - chassisAngularOffset;
    }

    public void updateStates(SwerveModuleIOStates states) {
        // if tuning a value, update this chunk for that motor's p, i, OR d
        // attempting to have this logic running with multiple causes a loop overrun :)
        if (TUNABLE_DRIVING_P.hasChanged()) {
            sparkFlexClosedLoopConfigDriving.p(TUNABLE_DRIVING_P.get());
            sparkFlexConfigDriving.apply(sparkFlexClosedLoopConfigDriving);
            drivingSparkFlex.configure(sparkFlexConfigDriving, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }
        
        // turning version
        // if (TUNABLE_TURNING_P.hasChanged()) {
        //     sparkMaxClosedLoopConfigTurning.p(TUNABLE_TURNING_P.get());
        //     sparkMaxConfigTurning.apply(sparkMaxClosedLoopConfigTurning);
        //     turningSparkMax.configure(sparkMaxConfigTurning, ResetMode.kResetSafeParameters,
        //             PersistMode.kPersistParameters);
        // }

        states.desiredAngle = Units.radiansToDegrees(MathUtil.angleModulus(this.desiredAngle));
        states.turnAngle = Units.radiansToDegrees(MathUtil.angleModulus(getTurnEncoderPosition()));
        states.driveDesiredVelocity = this.driveDesiredVelocity;
        states.driveVelocity = getDriveEncoderSpeedMPS();
        states.driveEncoderPos = getDriveEncoderPosition();
        states.driveVoltage = drivingSparkFlex.getBusVoltage() * drivingSparkFlex.getAppliedOutput();
        states.turnVoltage = turningSparkMax.getBusVoltage() * turningSparkMax.getAppliedOutput();
        states.driveCurrent = drivingSparkFlex.getOutputCurrent();
        states.turnCurrent = turningSparkMax.getOutputCurrent();

        SmartDashboard.putNumber("Swerve/module " + name + "/turn desired angle(deg)", states.desiredAngle);
        SmartDashboard.putNumber("Swerve/module " + name + "/turn angle(deg)",
                states.turnAngle);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive desired velocity(mps)",
                states.driveDesiredVelocity);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive velocity(mps)", states.driveVelocity);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive encoder position(m)",
                states.driveEncoderPos);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive voltage(volt)", states.driveVoltage);
        SmartDashboard.putNumber("Swerve/module " + name + "/turn voltage(volt)", states.turnVoltage);
        SmartDashboard.putNumber("Swerve/module " + name + "/drive current(amps)", states.driveCurrent);
        SmartDashboard.putNumber("Swerve/module " + name + "/turn current(amps)", states.turnCurrent);
    }

    // the rest of this file comes from AdvantageKit's SparkSwerveTemplate; per the
    // license, here is their disclaimer
    // Copyright (c) 2021-2026 Littleton Robotics
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by a BSD
    // license that can be found in the LICENSE file
    // at the root directory of this project.
    @Override
    public void setDriveOpenLoop(double output) {
        drivingSparkFlex.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turningSparkMax.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = DRIVING_KS * Math.signum(velocityRadPerSec) + DRIVING_KV * velocityRadPerSec;
        drivingPidController.setSetpoint(
                velocityRadPerSec,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.plus(Rotation2d.fromRadians(chassisAngularOffset)).getRadians(), TURNING_ENCODER_POSITION_PID_MIN_INPUT, TURNING_ENCODER_POSITION_PID_MAX_INPUT);
        turningPidController.setSetpoint(setpoint, ControlType.kPosition);
    }

}