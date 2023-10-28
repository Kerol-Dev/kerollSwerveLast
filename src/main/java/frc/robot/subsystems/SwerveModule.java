package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Conversions;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    public final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final AbsoluteEncoder turningEncoder;

    private final SparkMaxPIDController turningPidController;

    public double angleOffset;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

        angleOffset = absoluteEncoderOffset;

        turningEncoder.setInverted(absoluteEncoderReversed);

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = turningMotor.getPIDController();

        // Set the PID gains for the turning motor. Note these are example gains, and
        // you
        // may need to tune them for your own robot!
        turningPidController.setP(ModuleConstants.kTurningP);
        turningPidController.setI(ModuleConstants.kTurningI);
        turningPidController.setD(ModuleConstants.kTurningD);
        turningPidController.setFF(ModuleConstants.kTurningFF);
        turningPidController.setOutputRange(ModuleConstants.kTurningMinOutput,
                ModuleConstants.kTurningMaxOutput);

        resetEncoders();
    }

    public double getDrivePosition() {
        return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), ModuleConstants.kWheelDiameterMeters,
                ModuleConstants.kDriveMotorGearRatio);
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), ModuleConstants.kWheelDiameterMeters,
                ModuleConstants.kDriveMotorGearRatio);
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                new Rotation2d(getTurningPosition()).plus(Rotation2d.fromDegrees(angleOffset)));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput,
                state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(0);
    }
}
