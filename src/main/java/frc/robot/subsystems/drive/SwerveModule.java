package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOStates;

public class SwerveModule {

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private SwerveModuleIOStates states = new SwerveModuleIOStates();

    private SwerveModuleIO io;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
        io.setDriveEncoderPosition(0);
        desiredState.angle = new Rotation2d(getTurnEncoderPosition());
    }

    public double getDriveEncoderSpeedMPS() {
        return io.getDriveEncoderSpeedMPS();
    }

    public double getTurnEncoderPosition() {
        return io.getTurnEncoderPosition();
    }

    public double getDriveEncoderPosition() {
        return io.getDriveEncoderPosition();
    }

    public void resetEncoders() {
        io.setDriveEncoderPosition(0);
    }

    public void updateStates() {
        io.updateStates(states);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(getDriveEncoderSpeedMPS(),
                new Rotation2d((getTurnEncoderPosition()) - io.getChassisAngularOffset()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                getDriveEncoderPosition(),
                new Rotation2d(getTurnEncoderPosition() - io.getChassisAngularOffset()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param newDesiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState newDesiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = newDesiredState.speedMetersPerSecond;
        correctedDesiredState.angle = newDesiredState.angle.plus(Rotation2d.fromRadians(io.getChassisAngularOffset()));

        // Optimize the reference state to avoid spinning further than 90 degrees.

        correctedDesiredState.optimize(new Rotation2d(getTurnEncoderPosition()));
        io.setDesiredDriveSpeedMPS(correctedDesiredState.speedMetersPerSecond);
        io.setDesiredTurnAngle(correctedDesiredState.angle.getRadians());
        desiredState = newDesiredState;
    }

    public Distance getWheelRadius() {
        return io.getWheelDiameter().div(2.0);
    }

    // the rest of this file comes from AdvantageKit's SparkSwerveTemplate; per the
    // license, here is their disclaimer
    // Copyright (c) 2021-2026 Littleton Robotics
    // http://github.com/Mechanical-Advantage
    //
    // Use of this source code is governed by a BSD
    // license that can be found in the LICENSE file
    // at the root directory of this project.
    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(Rotation2d.fromRadians(io.getTurnAngleCharacterization()));
        state.cosineScale(Rotation2d.fromRadians(io.getTurnEncoderPosition()));

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / (io.getWheelDiameter().in(Meters) / 2.0));
        io.setTurnPosition(state.angle);
    }

    /** Returns the module velocity in rad/sec. */
    public double getFFCharacterizationVelocity() {
        return io.getDriveEncoderSpeedMPS() * Math.PI;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return io.getDriveEncoderPosition() * Math.PI;
    }
}