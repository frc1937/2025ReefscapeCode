package frc.lib.generic.hardware.encoder;

public enum EncoderSignal {
    POSITION(0),
    VELOCITY(1);

    final int id;

    EncoderSignal(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}
