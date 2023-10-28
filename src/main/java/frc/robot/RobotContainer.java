package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getLeftY() / 1.4f,
                () -> driverJoytick.getLeftX() / 1.4f,
                () -> driverJoytick.getRightX() / 1.4f,
                () -> false));

        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
