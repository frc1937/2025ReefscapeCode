package frc.lib.generic.hardware.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class KeyboardController {
    public Trigger one() {
        return createKeyTrigger("1");
    }

    public Trigger two() {
        return createKeyTrigger("2");
    }

    public Trigger three() {
        return createKeyTrigger("3");
    }

    public Trigger four() {
        return createKeyTrigger("4");
    }

    public Trigger five() {
        return createKeyTrigger("5");
    }

    public Trigger six() {
        return createKeyTrigger("6");
    }

    public Trigger seven() {
        return createKeyTrigger("7");
    }

    public Trigger eight() {
        return createKeyTrigger("8");
    }

    public Trigger nine() {
        return createKeyTrigger("9");
    }

    public Trigger zero() {
        return createKeyTrigger("0");
    }

    public Trigger q() {
        return createKeyTrigger("q");
    }

    public Trigger w() {
        return createKeyTrigger("w");
    }

    public Trigger e() {
        return createKeyTrigger("e");
    }

    public Trigger r() {
        return createKeyTrigger("r");
    }

    public Trigger t() {
        return createKeyTrigger("t");
    }

    public Trigger y() {
        return createKeyTrigger("y");
    }

    public Trigger u() {
        return createKeyTrigger("u");
    }

    public Trigger i() {
        return createKeyTrigger("i");
    }

    public Trigger o() {
        return createKeyTrigger("o");
    }

    public Trigger p() {
        return createKeyTrigger("p");
    }

    public Trigger a() {
        return createKeyTrigger("a");
    }

    public Trigger s() {
        return createKeyTrigger("s");
    }

    public Trigger d() {
        return createKeyTrigger("d");
    }

    public Trigger f() {
        return createKeyTrigger("f");
    }

    public Trigger g() {
        return createKeyTrigger("g");
    }

    public Trigger h() {
        return createKeyTrigger("h");
    }

    public Trigger j() {
        return createKeyTrigger("j");
    }

    public Trigger k() {
        return createKeyTrigger("k");
    }

    public Trigger l() {
        return createKeyTrigger("l");
    }

    public Trigger z() {
        return createKeyTrigger("z");
    }

    public Trigger x() {
        return createKeyTrigger("x");
    }

    public Trigger c() {
        return createKeyTrigger("c");
    }

    public Trigger v() {
        return createKeyTrigger("v");
    }

    public Trigger b() {
        return createKeyTrigger("b");
    }

    public Trigger n() {
        return createKeyTrigger("n");
    }

    public Trigger m() {
        return createKeyTrigger("m");
    }

    private Trigger createKeyTrigger(String key) {
        return new Trigger(new LoggedNetworkBoolean("/SmartDashboard/keyboard/" + key)::get);
    }
}
