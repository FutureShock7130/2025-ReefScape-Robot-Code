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
    
    public CommandXboxController controller = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT);

    private final Swerve swerve = new Swerve();

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerve, controller);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        swerve.setDefaultCommand(teleopSwerve);

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
