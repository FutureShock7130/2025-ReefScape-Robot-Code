// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class GeneratePath {

  PathConstraints constraints = new PathConstraints(
      4.0, 
      4.0, 
      Rotation2d.fromDegrees(180).getRotations(), 
      Rotation2d.fromDegrees(360).getRotations());

  public Command PathFindToPose(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(targetPose, constraints);
  }

  public Command PathFindThenFollow(String path) throws FileVersionException, IOException, ParseException {
    return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(path), constraints);
  }

  public Command FollowPath(String path) throws FileVersionException, IOException, ParseException {
    return AutoBuilder.followPath(PathPlannerPath.fromPathFile(path));
  }

  public Command FollowChoreoPath(String path) throws FileVersionException, IOException, ParseException {
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(path));
  }
}
