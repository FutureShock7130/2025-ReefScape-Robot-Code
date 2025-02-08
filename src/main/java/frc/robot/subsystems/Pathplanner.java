// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathPlanner extends SubsystemBase {

  private static PathPlanner m_instance = null;

  public static PathPlanner getInstance() {
      if (m_instance == null) {
          m_instance = new PathPlanner();
      }
      return m_instance;
  }
  
  private final SwerveDrive m_swerve = SwerveDrive.getInstance();
  private final SwervePoseEstimator m_poseEstimator = SwervePoseEstimator.getInstance();

  private Field2d m_field = new Field2d();

  public PathPlanner() {

    RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (config == null) {
            throw new RuntimeException("Failed to load config");
        }

    AutoBuilder.configure(
      m_poseEstimator::getPoseEstimatorPose,
      m_poseEstimator::setPoseEstimatorPose,
      m_poseEstimator::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> m_swerve.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(10.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          } 
          return false;
      }
    );

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_field.getObject("target").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_field.getObject("path").setPoses(poses);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
