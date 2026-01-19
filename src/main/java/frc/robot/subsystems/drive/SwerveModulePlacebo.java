package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class SwerveModulePlacebo implements SwerveModuleIO {
    @Override
    public void setDriveEncoderPosition(double position) {
    }

    @Override
    public double getTurnEncoderPosition() {
        return 0.0;
    }

    @Override
    public void setDesiredDriveSpeedMPS(double speed) {
    }

    @Override
    public double getDriveEncoderSpeedMPS() {
        return 0.0;
    }

    @Override
    public void setDesiredTurnAngle(double angle) {
    }

    @Override
    public double getDriveEncoderPosition() {
        return 0.0;
    }

    @Override
    public double getChassisAngularOffset() {
        return 0.0;
    }

    @Override
    public void updateStates(SwerveModuleIOStates states) {

    }

    @Override
    public void setDriveOpenLoop(double output) {
        
    }

    @Override
    public void setTurnOpenLoop(double output) {
        
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
       
    }

    @Override
    public Distance getWheelDiameter() {
        return Inches.of(0);
    }

    @Override
    public double getTurnAngleCharacterization() {
        return 0;
    }
}