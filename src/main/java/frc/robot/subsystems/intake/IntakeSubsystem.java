package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeArmIO.IntakeArmIOState;
import frc.robot.subsystems.intake.IntakeArmIO.IntakeArmSetpoint;
import frc.robot.subsystems.intake.IntakeRollerIO.IntakeRollerIOState;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeRollerIO rollerIO;
    private IntakeArmIO armIO;
    private IntakeRollerIOState rollerState = new IntakeRollerIOState();
    private IntakeArmIOState armState = new IntakeArmIOState();

    public boolean armProfiledPidEnabled = false;

    public IntakeSubsystem(IntakeRollerIO rollerIO, IntakeArmIO armIO) {
        this.rollerIO = rollerIO;
        this.armIO = armIO;
    }

    public Command deployAndRunIntake() {
        return this.run(
                        () -> {
                            // armProfiledPidEnabled = true;
                            armIO.setSetpoint(IntakeArmSetpoint.DEPLOYED);
                            rollerIO.runIntake();
                        })
                .withName("intake: deploy arm and run roller");
    }

    public Command defaultBehavior() {
        return this.run(
                        () -> {
                            // armProfiledPidEnabled = true;
                            rollerIO.setZero();
                            armIO.runIntakeArmZeroVelocity();
                        })
                .withName("intake defaultBehavior (arm stowed + roller at 0)");
    }

    public Command runIntakeArmOutVelocity() {
        return this.run(() -> armIO.runIntakeArmOutVelocity()).withName("arm out (manual)");
    }

    public Command runIntakeArmInVelocity() {
        return this.run(() -> armIO.runIntakeArmInVelocity()).withName("arm in (manual)");
    }

    public Command runArmOffVolts() {
        return this.run(() -> armIO.runOffVolts());
    }

    public Command runRoller() {
        return this.run(() -> rollerIO.runIntake());
    }

    public Command deployArm() {
        return this.runOnce(
                () -> {
                    // armProfiledPidEnabled = true;
                    armIO.setSetpoint(IntakeArmSetpoint.DEPLOYED);
                });
    }

    public Command stowArm() {
        return this.runOnce(
                () -> {
                    // armProfiledPidEnabled = true;
                    armIO.setSetpoint(IntakeArmSetpoint.STOWED);
                });
    }

    // public Command feedFuelToSpindexer() {
    //      return this.run(
    //             () -> {
    //                 armProfiledPidEnabled = true;
    //                 armIO.setSetpoint(IntakeArmSetpoint.STOWED);

    //             });

    // }

    @Override
    public void periodic() {

        if (!armProfiledPidEnabled) {
            armIO.resetSetpointsToCurrentPosition();
        } else {
            armIO.calculateNextIntermediateSetpoint();
        }

        rollerIO.updateState(rollerState);
        armIO.updateState(armState);

        SmartDashboard.putNumber(
                "intake/roller/desired speed (radians per second)",
                rollerState.desiredSpeedRadiansPerSecond);
        SmartDashboard.putNumber(
                "intake/roller/actual speed (radians per second)",
                rollerState.actualSpeedRadiansPerSecond);
        SmartDashboard.putNumber("intake/roller/current", rollerState.current);
        SmartDashboard.putNumber("intake/roller/applied output", rollerState.appliedVoltage);

        SmartDashboard.putNumber("intake/arm/desired angle (deg)", armState.desiredAngleDegrees);
        SmartDashboard.putNumber(
                "intake/arm/intermediate angle (deg)", armState.intermediateAngleDegrees);
        SmartDashboard.putNumber("intake/arm/actual angle (deg)", armState.actualAngleDegrees);
        SmartDashboard.putNumber(
                "intake/arm/velocity (deg/sec)", armState.velocityDegreesPerSecond);
        SmartDashboard.putNumber(
                "intake/arm/intermediate velocity (deg/s)",
                armState.intermediateVelocityDegreesPerSecond);
        SmartDashboard.putNumber(
                "intake/arm/desired velocity (deg/sec)", armState.desiredVelocityDegreesPerSecond);
        SmartDashboard.putNumber("intake/arm/current", armState.current);
        SmartDashboard.putNumber("intake/arm/applied voltage", armState.appliedVoltage);
    }
}
