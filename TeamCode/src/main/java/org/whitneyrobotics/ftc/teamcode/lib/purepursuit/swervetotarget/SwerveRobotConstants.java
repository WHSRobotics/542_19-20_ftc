package org.whitneyrobotics.ftc.teamcode.lib.purepursuit.swervetotarget;

public class SwerveRobotConstants {

    // Based on robot physical properties
    private double kV, kA, kP;

    public double getkV() {
        return kV;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public double getkA() {
        return kA;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public SwerveRobotConstants(double kV, double kA, double kP) {
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
    }

}
