// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class SwerveVision extends SubsystemBase {
  
  private static SwerveVision m_instance = null;

  public static SwerveVision getInstance() {
    if (m_instance == null) {
      m_instance = new SwerveVision();
    }
    return m_instance;
  }

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private Matrix<N3, N1> curStdDevs;

  public SwerveVision() {
    camera = new PhotonCamera(VisionConstants.CHASSIS_CAM_NAME);

    photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<EstimatedRobotPose> getVisionEstimatedPose() {
    Optional<EstimatedRobotPose> visionEstPose = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
      visionEstPose = photonPoseEstimator.update(change);
      updateEstimationStdDevs(visionEstPose, change.getTargets());
    }
    return visionEstPose;
  }

  private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
          var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
                  tagPose
                          .get()
                          .toPose2d()
                          .getTranslation()
                          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
              estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
      return curStdDevs;
  }

  public Pose2d getLatestPose() {
      Optional<EstimatedRobotPose> visionEst = getVisionEstimatedPose();
      
      // Check if we have a valid vision estimate >w<
      if (visionEst.isPresent()) {
          return visionEst.get().estimatedPose.toPose2d();
      }
      
      return null; // No valid pose found ʕ•ᴥ•ʔ
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
