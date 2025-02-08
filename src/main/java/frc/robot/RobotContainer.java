package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.GeneratePath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    public CommandXboxController m_Driver = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
    public CommandXboxController m_Operator = new CommandXboxController(RobotConstants.OPERATOR_CONTROLLER_PORT);

    private final Swerve m_Swerve = new Swerve();

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, m_Driver);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        m_Swerve.setDefaultCommand(teleopSwerve);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
