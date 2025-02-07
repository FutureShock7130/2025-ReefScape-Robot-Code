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

    private final GeneratePath m_GeneratePath = new GeneratePath();

    public RobotContainer() {
        m_Swerve.setDefaultCommand(teleopSwerve);

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private BooleanSupplier isDriverControll = () -> {
        return m_Driver.getHID().getLeftX() >= 0.02 
                || m_Driver.getHID().getLeftY() >= 0.02
                || m_Driver.getHID().getRightX() >= 0.02 
                || m_Driver.getHID().getAButton()
                || m_Driver.getHID().getBButton() 
                || m_Driver.getHID().getXButton() 
                || m_Driver.getHID().getYButton();
    };

    private void configureButtonBindings() {
        /***** SOME EXAMPLE BELOW, ADD LATER*****/
        // run a path to certain pose automatically
        m_Operator.a().and(m_Operator.b()).onTrue(m_GeneratePath.PathFindToPose(FieldConstants.A).until(isDriverControll));
        m_Operator.a().and(m_Operator.x()).onTrue(m_GeneratePath.PathFindToPose(FieldConstants.CSL).until(isDriverControll));
        m_Operator.a().and(m_Operator.y()).onTrue(m_GeneratePath.PathFindToPose(FieldConstants.CSR).until(isDriverControll));
        // driver can reset odometry while operator is scoring/intaking
        m_Driver.back().and(m_Driver.leftBumper()).onTrue(Commands.runOnce(() -> m_Swerve.setOdometryPosition(FieldConstants.CSL), m_Swerve));
        m_Driver.back().and(m_Operator.rightBumper()).onTrue(Commands.runOnce(() -> m_Swerve.setOdometryPosition(FieldConstants.CSR), m_Swerve));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
