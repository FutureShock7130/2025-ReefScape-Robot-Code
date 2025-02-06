package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    
    private static CommandXboxController mController = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT);

    private final Swerve mSwerve = Swerve.getInstance();

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(mSwerve, mController);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        mSwerve.setDefaultCommand(teleopSwerve);

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static CommandXboxController getXboxController() {
        return mController;
    }
}
