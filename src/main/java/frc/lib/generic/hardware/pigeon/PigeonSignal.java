package frc.lib.generic.hardware.pigeon;

public enum PigeonSignal {
    YAW(0),
    PITCH(1),
    ROLL(2);

    private final int id;

    PigeonSignal(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}
