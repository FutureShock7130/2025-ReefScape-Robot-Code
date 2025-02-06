package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwervePoseEstimator;

public class RobotContainer {
    
    public CommandXboxController controller = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT);

    private final SwerveDrive swerveD = new SwerveDrive();
    private final SwervePoseEstimator swervePE = new SwervePoseEstimator();

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(swerveD, swervePE, controller);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        swerveD.setDefaultCommand(teleopSwerve);
        swervePE.setDefaultCommand(teleopSwerve);

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
