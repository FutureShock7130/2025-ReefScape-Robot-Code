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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.IO.GyroIO;
import frc.robot.IO.GyroIO.GyroIOInputs;

public class SwervePoseEstimator extends SubsystemBase {

	private GyroIO gyroIO;
	private final GyroIOInputs gyroInputs = new GyroIOInputs();

	private static SwervePoseEstimator m_instance = null;

	public static SwervePoseEstimator getInstance(GyroIO gyroIO) {
		if (m_instance == null) {
			m_instance = new SwervePoseEstimator(gyroIO);
		}
		return m_instance;
	}

	private SwerveDrive m_SwerveDrive = SwerveDrive.getInstance();
	private SwerveVision m_SwerveVision = SwerveVision.getInstance();

	private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATOIN_METERS);
	private Rotation2d m_gyroYaw = new Rotation2d();
	private Supplier<SwerveModulePosition[]> m_modulePos = () -> m_SwerveDrive.getModulePositions();
	private SwerveModulePosition[] m_lastModulePos = // For delta tracking
		new SwerveModulePosition[] {
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition(),
			new SwerveModulePosition()
		};
	private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
			m_kinematics,
			m_gyroYaw,
			m_lastModulePos,
			new Pose2d()
	);

	private Field2d m_field = new Field2d();

	public SwervePoseEstimator(GyroIO gyroIO) {
		this.gyroIO = gyroIO;
		m_poseEstimator.resetPosition(getGyroYaw(), m_SwerveDrive.getModulePositions(), getPoseEstimatorPose());
	}

	public Rotation2d getGyroYaw() {
		return m_gyroYaw;
	}

	public Pose2d getPoseEstimatorPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	public void setPoseEstimatorPose(Pose2d pose) {
		m_poseEstimator.resetPosition(getGyroYaw(), m_SwerveDrive.getModulePositions(), pose);
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return ChassisSpeeds.fromFieldRelativeSpeeds(m_kinematics.toChassisSpeeds(m_SwerveDrive.getModuleStates()),
				getGyroYaw());
	}

	@Override
	public void periodic() {
		gyroIO.updateInputs(gyroInputs);

		SwerveModulePosition[] modulePos = m_modulePos.get();
		SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
		for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
			moduleDeltas[moduleIndex] =
				new SwerveModulePosition(
					modulePos[moduleIndex].distanceMeters
						- m_lastModulePos[moduleIndex].distanceMeters,
						modulePos[moduleIndex].angle);
			m_lastModulePos[moduleIndex] = modulePos[moduleIndex];
		}

		if (gyroInputs.connected) {
			m_gyroYaw = gyroInputs.yawPosition;
			} else {
			Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
			m_gyroYaw = m_gyroYaw.plus(new Rotation2d(twist.dtheta));
		}

		m_poseEstimator.update(getGyroYaw(), m_SwerveDrive.getModulePositions());
		m_field.setRobotPose(getPoseEstimatorPose());
		SmartDashboard.putData("Field", m_field);

		Pose2d visionPose = m_SwerveVision.getLatestPose();
		if (visionPose != null) {
			m_poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
		}	
		
		SmartDashboard.putNumber("gyro (deg)", getGyroYaw().getDegrees());
		SmartDashboard.putNumber("swerve pose estimator x", getPoseEstimatorPose().getX());
		SmartDashboard.putNumber("swerve pose estimator y", getPoseEstimatorPose().getY());
	}
}