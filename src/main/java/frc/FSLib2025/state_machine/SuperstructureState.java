package frc.FSLib2025.state_machine;

public enum SuperstructureState {
    DEFAULT(0);

    private double value;

    private SuperstructureState(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}
