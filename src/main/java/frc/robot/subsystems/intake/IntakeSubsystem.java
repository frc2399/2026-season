package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
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

    private Time feedFuelTimeoutSeconds = Seconds.of(1);
    public boolean armProfiledPidEnabled = false;

    public IntakeSubsystem(IntakeRollerIO rollerIO, IntakeArmIO armIO) {
        this.rollerIO = rollerIO;
        this.armIO = armIO;
    }

    public Command deployAndRunIntake() {
        return this.run(
                        () -> {
                            armProfiledPidEnabled = true;
                            armIO.setSetpoint(IntakeArmSetpoint.DEPLOYED);
                            rollerIO.runIntake();
                        })
                .withName("intake: deploy arm and run roller");
    }

    public Command defaultBehavior() {
        return this.run(
                        () -> {
                            armProfiledPidEnabled = true;
                            rollerIO.setZero();
                            armIO.setSetpoint(IntakeArmSetpoint.STOWED);
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
                    armProfiledPidEnabled = true;
                    armIO.setSetpoint(IntakeArmSetpoint.DEPLOYED);
                });
    }

    public Command stowArmSetpoint() {
        return this.runOnce(
                () -> {
                    armProfiledPidEnabled = true;
                    armIO.setSetpoint(IntakeArmSetpoint.STOWED);
                });
    }

    public Command feedFuelSetpoint() {
        return this.runOnce(
                () -> {
                    armProfiledPidEnabled = true;
                    armIO.setSetpoint(IntakeArmSetpoint.FEED_FUEL);
                });
    }

    public Command feedFuel() {
        Command feedFuelCommand =
                this.run(
                                () -> {
                                    armProfiledPidEnabled = true;
                                    armIO.setSetpoint(IntakeArmSetpoint.FEED_FUEL);
                                })
                        .withTimeout(Seconds.of(1))
                        .andThen(
                                this.run(
                                                () -> {
                                                    armIO.setSetpoint(IntakeArmSetpoint.STOWED);
                                                })
                                        .withTimeout(Seconds.of(1)))
                        .repeatedly()
                        .withName("feed fuel (arm)");

        return feedFuelCommand;
    }

    public boolean isArmBelowTrench() {
        return armIO.isArmBelowTrench();
    }

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
        SmartDashboard.putNumber("intake/roller/current (amps)", rollerState.current);
        SmartDashboard.putNumber("intake/roller/applied output (volt)", rollerState.appliedVoltage);

        SmartDashboard.putNumber("intake/arm/desired angle (deg)", armState.desiredAngleDegrees);
        SmartDashboard.putNumber(
                "intake/arm/intermediate angle (deg)", armState.intermediateAngleDegrees);
        SmartDashboard.putNumber("intake/arm/actual angle (deg)", armState.actualAngleDegrees);
        SmartDashboard.putNumber(
                "intake/arm/velocity (deg per s)", armState.velocityDegreesPerSecond);
        SmartDashboard.putNumber(
                "intake/arm/intermediate velocity (deg per s)",
                armState.intermediateVelocityDegreesPerSecond);
        SmartDashboard.putNumber(
                "intake/arm/desired velocity (deg per s)",
                armState.desiredVelocityDegreesPerSecond);
        SmartDashboard.putNumber("intake/arm/current (amps)", armState.current);
        SmartDashboard.putNumber("intake/arm/applied voltage (volt)", armState.appliedVoltage);
        SmartDashboard.putBoolean("intake/arm/is below trench height", isArmBelowTrench());
    }
}
