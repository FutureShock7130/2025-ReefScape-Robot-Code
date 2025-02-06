// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib2025.vision.LimelightHelpers;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwervePoseEstimator extends SubsystemBase {

  private static SwervePoseEstimator m_instance = null;

  public static SwervePoseEstimator getInstance() {
    if (m_instance == null) {
      m_instance = new SwervePoseEstimator();
    }
    return m_instance;
  }
  
  private SwerveDrive m_Swerve = SwerveDrive.getInstance();
  private SwerveVision m_SwerveVision = SwerveVision.getInstance();

  private Pigeon2 pigeon = new Pigeon2(SwerveConstants.PIGEON_ID, RobotConstants.CANBUS_NAME);

  private Rotation2d m_gyroYaw = getGyroYaw();
  private Supplier<SwerveModulePosition[]> m_modulePos = () -> m_Swerve.getModulePositions();
  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATOIN_METERS);
  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    m_kinematics,
    m_gyroYaw,
    m_modulePos.get(),
    m_SwerveVision.getVisionEstimatedPose().get().estimatedPose.toPose2d(),
    VecBuilder.fill(0.1, 0.1, 0.1),
    VecBuilder.fill(0.9, 0.9, 0.9)
  );

  private Field2d m_field = new Field2d();

  public SwervePoseEstimator() {
    m_poseEstimator.resetPosition(getGyroYaw(), m_Swerve.getModulePositions(), getPoseEstimatorPose());
    pigeon.reset();
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  public void setGyroYaw(Rotation2d yaw) {
      pigeon.setYaw(yaw.getDegrees());
  }

  public Pose2d getPoseEstimatorPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setPoseEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroYaw(), m_Swerve.getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(m_kinematics.toChassisSpeeds(m_Swerve.getModuleStates()), getGyroYaw());
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(getGyroYaw(), m_Swerve.getModulePositions());
    if (m_SwerveVision.getVisionEstimatedPose().isPresent()) m_poseEstimator.addVisionMeasurement(m_SwerveVision.getVisionEstimatedPose().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    m_field.setRobotPose(getPoseEstimatorPose());

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("gyro (deg)", getGyroYaw().getDegrees());
    SmartDashboard.putNumber("swerve pose estimator x", getPoseEstimatorPose().getX());
    SmartDashboard.putNumber("swerve pose estimator y", getPoseEstimatorPose().getY());
  }
}
