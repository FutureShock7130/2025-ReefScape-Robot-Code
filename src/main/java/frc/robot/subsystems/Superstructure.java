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
  SuperstructureState mWantedState = SuperstructureState.IDLE;

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

  public void updateState() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mCommandedState = mStateMachine.getCommandedState();

    SmartDashboard.putString("Commanded State", mCommandedState.toString());
    SmartDashboard.putString("Wanted State", mWantedState.toString());
  }
}
