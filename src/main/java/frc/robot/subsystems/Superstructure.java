// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.statemachines.StateMachine;
import frc.robot.statemachines.SuperstructureState;

public class Superstructure extends SubsystemBase {

    Arm mArm;
    Elevator mElevator;
    Grabber mGrabber;
    Intake mIntake;

    StateMachine mStateMachine;
    SuperstructureState mCommandedState;

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
    }

    private void idle() {

    }

    private void intakeAlgae() {

    }

    private void scoreProcessor() {

    }

    private void intakeCoral() {

    }

    private void scoreL1() {

    }

    private void scoreL2() {

    }

    private void scoreL3() {

    }

    private void scoreL4() {

    }

    private void shoot() {

    }

    public void updateState() {
        switch (mCommandedState) {
            case IDLE:
                idle();
                break;
            case INTAKE_ALGAE:
                intakeAlgae();
                break;
            case SCORE_PROCESSOR:
                scoreProcessor();
                break;
            case INTAKE_CORAL:
                intakeCoral();
                break;
            case SCORE_L1:
                scoreL1();
                break;
            case SCORE_L2:
                scoreL2();
                break;
            case SCORE_L3:
                scoreL3();
                break;
            case SCORE_L4:
                scoreL4();
                break;
            case SHOOT:
                shoot();
                break;
            default:
                idle();
                break;
        }
    }

    public void setState(SuperstructureState state) {
        mStateMachine.setCommandedState(state);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        mCommandedState = mStateMachine.getCommandedState();
        updateState();

        SmartDashboard.putString("Commanded State", mCommandedState.toString());
    }
}
