package frc.robot.statemachines;

public class StateMachine {
    private static StateMachine instance = null;

    private SuperstructureState commandedState = SuperstructureState.IDLE;

    public StateMachine() {}

    public static synchronized StateMachine getInstance() {
        if (instance == null) {
            instance = new StateMachine();
        }
        return instance;
    }

    public void setCommandedState(SuperstructureState wantedState) {
        commandedState = wantedState;
    }

    public SuperstructureState getCommandedState() {
        return commandedState;
    }
}
