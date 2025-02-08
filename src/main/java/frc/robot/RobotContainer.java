package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.commands.TeleopSuperstructure;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PathPlanner;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwervePoseEstimator;

public class RobotContainer {
    
    private CommandXboxController m_Controller;
    private Joystick joystick;
    private static ButtonBox m_buttonbox;
    private final Elevator m_elevator;
    private final Grabber m_grabber;
    private final Intake m_intake;
    private final LED m_led;
    private final Superstructure m_superstructure;
    private final SwerveDrive m_swerveD;
    private final SwervePoseEstimator m_swervePE;
    private final PathPlanner m_pathplanner;

    private final TeleopSwerve teleopSwerve;
    private final TeleopSuperstructure teleopSuperstructure;
    private final PathPlannerCommand pathPlannerCommand;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        m_Controller = new CommandXboxController(RobotConstants.DRIVE_CONTROLLER_PORT);
        joystick = new Joystick(RobotConstants.OPERATOR_BUTTONBOX_PORT);
        m_buttonbox = new ButtonBox(joystick);

        m_swerveD = SwerveDrive.getInstance();
        m_swervePE = SwervePoseEstimator.getInstance();
        m_pathplanner = PathPlanner.getInstance();
        m_elevator = Elevator.getInstance();
        m_grabber = Grabber.getInstance();
        m_intake = Intake.getInstance();
        m_led = LED.getInstance();
        m_superstructure = Superstructure.getInstance();
        
        teleopSwerve = new TeleopSwerve(m_swerveD, m_swervePE, m_Controller);
        teleopSuperstructure = new TeleopSuperstructure();
        pathPlannerCommand = new PathPlannerCommand(m_pathplanner);

        m_swerveD.setDefaultCommand(teleopSwerve);
        m_swervePE.setDefaultCommand(teleopSwerve);
        m_superstructure.setDefaultCommand(teleopSuperstructure);
        m_pathplanner.setDefaultCommand(pathPlannerCommand);

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtonBindings() {
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static ButtonBox geButtonBox() {
        return m_buttonbox;
    }
}
