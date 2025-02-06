// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.statemachines.StateMachine;
import frc.robot.statemachines.SuperstructureState;

public class Superstructure extends SubsystemBase {

  Arm mArm;
  Elevator mElevator;
  Grabber mGrabber;
  Intake mIntake;
  
  StateMachine mStateMachine;
  SuperstructureState mCommandedState;

  CommandXboxController mController;

  private static Superstructure mInstance = null;

  public static Superstructure getInstance() {
    if (mInstance == null) {
      mInstance = new Superstructure();
    }
    return mInstance;
  }

  /** Creates a new StateMachine. */
  public Superstructure() {
    mArm = Arm.getInstance();
    mElevator = Elevator.getInstance();
    mGrabber = Grabber.getInstance();
    mIntake = Intake.getInstance();
    mStateMachine = StateMachine.getInstance();
    mController = RobotContainer.getXboxController();
  }

  public void idle() {

  }

  public void intakeAlgae() {

  }

  public void scoreProcessor() {

  }

  public void intakeCoral() {

  }

  public void scoreL1() {

  }

  public void scoreL2() {

  }

  public void scoreL3() {

  }

  public void scoreL4() {

  }

  public void shoot() {

  }

  public void updateState() {
    switch (mCommandedState) {
      case IDLE:
        break;
      case INTAKE_ALGAE:
        break;
      case SCORE_PROCESSOR:
        break;
      case INTAKE_CORAL:
        break;
      case SCORE_L1:
        break;
      case SCORE_L2:
        break;
      case SCORE_L3:
        break;
      case SCORE_L4:
        break;
      case SHOOT:
        break;
      default:
        break;
    }
  }

  public void updateDriverInput() {
    // if Driver Press sth, wanted state = hahaha
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mCommandedState = mStateMachine.getCommandedState();
    updateDriverInput();
    updateState();

    SmartDashboard.putString("Commanded State", mCommandedState.toString());
  }
}
