package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwervePoseEstimator;

public class TeleopSwerve extends Command {

    private final SwerveDrive swerve;
    private final SwervePoseEstimator poseEstimator;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rotSpeed = 0.0;

    private double reduction = 1;

    private CommandXboxController controller;

    public TeleopSwerve(SwerveDrive swerve, SwervePoseEstimator poseEstimator, CommandXboxController controller) {
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.controller = controller;
        addRequirements(swerve);
        addRequirements(poseEstimator);
    }

    @Override
    public void execute() {

        if (controller.getHID().getAButtonPressed()) {
            poseEstimator.setPoseEstimatorPose(new Pose2d());
        }

        if (controller.getHID().getRightBumperButton()) {
            reduction = 0.3;
        } else {
            reduction = 1;
        }

        xSpeed = xLimiter.calculate(-controller.getLeftY() * reduction);
        ySpeed = yLimiter.calculate(-controller.getLeftX() * reduction);
        rotSpeed = rotLimiter.calculate(-controller.getRightX() * reduction);

        // square the input to inprove driving experience
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);        
        rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

        swerve.drive(
                new Translation2d(xSpeed, ySpeed).times(SwerveConstants.MAX_MODULE_SPEED),
                rotSpeed * SwerveConstants.MAX_MODULE_ROTATIONAL_SPEED,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0, false);
    }
}
