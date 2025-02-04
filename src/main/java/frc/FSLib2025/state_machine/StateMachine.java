package frc.FSLib2025.state_machine;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class StateMachine {
    private static StateMachine instance;

    private SuperstructureState superstructureState;

    public StateMachine() {
        Shuffleboard.getTab("state machine")
                .add("superstructure state", getSuperstructureState().getValue());
    }

    public static synchronized StateMachine getInstance() {
        if (instance == null) {
            instance = new StateMachine();
        }
        return instance;
    }

    public SuperstructureState getSuperstructureState() {
        return superstructureState;
    }

    public synchronized void update(XboxController controller) {
        // normal state change
        switch (superstructureState) {
            case DEFAULT:
                if (controller.getRightBumperButton()) {
                    superstructureState = SuperstructureState.DEFAULT;
                }
            default:
                break;
        }
    }
}
