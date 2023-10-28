package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final Pigeon2 gyro = new Pigeon2(7);

    public SwerveSubsystem() {
        SmartDashboard.putNumber("Front Left Offset", frontLeft.angleOffset);
        SmartDashboard.putNumber("Rear Left Offset", backLeft.angleOffset);
        SmartDashboard.putNumber("Front Right Offset", frontRight.angleOffset);
        SmartDashboard.putNumber("Rear Right Offset", backRight.angleOffset);

        SmartDashboard.putBoolean("Front Left Drive Inverted", false);
        SmartDashboard.putBoolean("Front Left Drive Inverted", false);
        SmartDashboard.putBoolean("Front Left Drive Inverted", false);
        SmartDashboard.putBoolean("Front Left Drive Inverted", false);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public void smartDashboardPeriodic() {
        frontLeft.angleOffset = SmartDashboard.getNumber("Front Left Offset", 0);
        frontRight.angleOffset = SmartDashboard.getNumber("Front Right Offset", 0);
        backLeft.angleOffset = SmartDashboard.getNumber("Rear Left Offset", 0);
        backRight.angleOffset = SmartDashboard.getNumber("Rear Right Offset", 0);

        SmartDashboard.putNumber("Gyro Angle", getHeading());

        SmartDashboard.putNumber("Front Right Angle", frontRight.getTurningPosition() + frontRight.angleOffset);
        SmartDashboard.putNumber("Front Left Angle", frontLeft.getTurningPosition() + frontLeft.angleOffset);
        SmartDashboard.putNumber("Rear Right Angle", backRight.getTurningPosition() + backRight.angleOffset);
        SmartDashboard.putNumber("Rear Left Angle", backLeft.getTurningPosition() + backLeft.angleOffset);

        frontLeft.driveMotor.setInverted(SmartDashboard.getBoolean("Front Left Drive Inverted", false));
        frontRight.driveMotor.setInverted(SmartDashboard.getBoolean("Front Right Drive Inverted", false));
        backLeft.driveMotor.setInverted(SmartDashboard.getBoolean("Rear Left Drive Inverted", false));
        backRight.driveMotor.setInverted(SmartDashboard.getBoolean("Rear Right Drive Inverted", false));
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        smartDashboardPeriodic();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
