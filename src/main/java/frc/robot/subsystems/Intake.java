// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SuperstructureConstants;

public class Intake extends SubsystemBase {
    /*
     * a simple intake with a row of jelly wheels
     * can pick up ALGAE from the floor
     * it can help the robot climb
     * when folded into the chassis with DEEP CAGE
     * 
     * control jelly wheels by NEO 550
     * control its angle by NEO Brushless (gear ratio : 61.875)
     */

    // motor for jelly wheels
    private final SparkMax intakeMotor;

    // motor for angle
    private final SparkMax angleMotor;

    // cancoder
    private final CANcoder cancoder;

    // shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    GenericEntry intakeSpeed = tab.add("Intake Speed (RPM)", 0)
            .withPosition(0, 0).withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    GenericEntry intakeAngle = tab.add("Intake Angle (degrees)", 0)
            .withPosition(1, 0).withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

    private static Intake mInstance = null;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public Intake() {
        intakeMotor = new SparkMax(SuperstructureConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.configure(SuperstructureConstants.INTAKE_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        angleMotor = new SparkMax(SuperstructureConstants.INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
        angleMotor.configure(SuperstructureConstants.INTAKE_ANGLE_MOTOR_CONFIGURATION, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        cancoder = new CANcoder(SuperstructureConstants.INTAKE_CANCODER_ID, RobotConstants.CANBUS_NAME);
        cancoder.getConfigurator().apply(SuperstructureConstants.INTAKE_CANCODER_CONFIGURATION);
    }

    public void setIntakeVolt(double volts) {
        intakeMotor.setVoltage(volts);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void setAngleMotorVolt(double volts) {
        angleMotor.setVoltage(volts);
    }

    public double getIntakeSpeed() {
        return intakeMotor.getEncoder().getVelocity();
    }

  public double getAngle() {
    return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).getDegrees();
  }

    @Override
    public void periodic() {
        intakeSpeed.setDouble(getIntakeSpeed());
        intakeAngle.setDouble(getAngle());
    }
}
