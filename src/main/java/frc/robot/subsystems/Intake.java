// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperstructureConstants;

public class Intake extends SubsystemBase {

  private final SparkMax mTilter;
  private final SparkMax mRoller;

  private static Intake mInstance = null;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }
  /** Creates a new Intake. */
  public Intake() {
    mTilter = new SparkMax(SuperstructureConstants.INTAKE_TILTER_MOTOR_ID, MotorType.kBrushless);
    mRoller = new SparkMax(SuperstructureConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    mTilter.configure(SuperstructureConstants.INTAKE_TILTER_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    mRoller.configure(SuperstructureConstants.INTAKE_ROLLER_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters, 
                PersistMode.kNoPersistParameters);
  }

  public void setTilterSpeed(double spd) {
    mTilter.set(spd);
  }

  public void setRollerSpeed(double spd) {
    mRoller.set(spd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake Tilter pos", mTilter.getEncoder().getPosition());
  }
}
