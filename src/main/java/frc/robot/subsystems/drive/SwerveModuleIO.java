package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public interface SwerveModuleIO {

    public static class SwerveModuleIOStates {

        public double driveVoltage = 0.0;
        public double turnVoltage = 0.0;
        public double driveVelocity = 0.0;
        public double driveDesiredVelocity = 0.0;
        public double turnAngle = 0.0;
        public double desiredAngle = 0.0;
        public double driveCurrent = 0.0;
        public double turnCurrent = 0.0;
        public double driveEncoderPos = 0.0;
        public double turningEncoderPos = 0.0;

    }

    public void setDriveEncoderPosition(double position);

    public void setDesiredDriveSpeedMPS(double speed);

    public double getDriveEncoderSpeedMPS();

    public void setDesiredTurnAngle(double angle);

    public double getTurnEncoderPosition();

    public double getDriveEncoderPosition();

    public void updateStates(SwerveModuleIOStates states);

    public double getChassisAngularOffset();
    
    public void setDriveOpenLoop(double output);

    public void setTurnOpenLoop(double output);

    public void setDriveVelocity(double velocityRadPerSec);

    public void setTurnPosition(Rotation2d rotation);

    public Distance getWheelDiameter();

    public double getTurnAngleCharacterization();
}