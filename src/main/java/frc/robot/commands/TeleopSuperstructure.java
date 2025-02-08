package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ButtonBox;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class TeleopSuperstructure extends Command {

    ButtonBox m_buttonbox;

    Superstructure m_superstructure;

    public TeleopSuperstructure() {
        m_buttonbox = RobotContainer.geButtonBox();
        m_superstructure = Superstructure.getInstance();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean isInterrupted) {

    }
    
}
