package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {

    private final Swerve m_Swerve;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rotSpeed = 0.0;

    private double reduction = 1;

    private CommandXboxController m_Driver;

    public TeleopSwerve(Swerve m_Swerve, CommandXboxController m_Driver) {
        this.m_Swerve = m_Swerve;
        this.m_Driver = m_Driver;
        addRequirements(m_Swerve);
    }

    @Override
    public void execute() {

        if (m_Driver.getHID().getBackButtonPressed()) {
            m_Swerve.setOdometryPosition(new Pose2d());
            m_Swerve.setGyroYaw(new Rotation2d());
        }

        double slow = 1;
        if (m_Driver.getHID().getLeftBumperButton() || m_Driver.getHID().getRightBumperButton())  {
            slow = 0.5;
        }

        double deadband = 0.04;
        double xyMultiplier = 1;
        double zMultiplier = 0.75;

        xSpeed = xLimiter.calculate(MathUtil.applyDeadband(-m_Driver.getLeftY(), deadband));
        ySpeed = yLimiter.calculate(MathUtil.applyDeadband(-m_Driver.getLeftX(), deadband));
        rotSpeed = rotLimiter.calculate(MathUtil.applyDeadband(-m_Driver.getRightX(), deadband));
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
        // rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);
        m_Swerve.drive(
                new Translation2d(xSpeed, ySpeed).times(SwerveConstants.MAX_MODULE_SPEED).times(xyMultiplier).times(slow),
                rotSpeed * SwerveConstants.MAX_MODULE_ROTATIONAL_SPEED * zMultiplier * slow,
                // 0,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        m_Swerve.drive(new Translation2d(), 0, false);
    }
}
