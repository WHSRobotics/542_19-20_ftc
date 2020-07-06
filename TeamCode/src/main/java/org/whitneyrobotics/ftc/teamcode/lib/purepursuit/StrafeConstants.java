package org.whitneyrobotics.ftc.teamcode.lib.purepursuit;

public class StrafeConstants extends SwerveConstants {

    private double maxAngularAcceleration, hKP, hKI, hKD;

    public StrafeConstants(double spacing, double weightSmooth, double turnSpeed, double pathMaxVelocity, double maxAcceleration, double lookaheadDistance, double kV, double kA, double maxAngularAcceleration, double hKP, double hKI, double hKD) {
        super(spacing, weightSmooth, turnSpeed, pathMaxVelocity, maxAcceleration, lookaheadDistance, kV, kA);
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.hKP = hKP;
        this.hKI = hKI;
        this.hKD = hKD;
    }

    public StrafeConstants(SwerveConstants swerveConstants, double maxAngularAcceleration, double hKP, double hKI, double hKD){
        super(swerveConstants.getSpacing(), swerveConstants.getWeightSmooth(), swerveConstants.getTurnSpeed(), swerveConstants.getPathMaxVelocity(), swerveConstants.getMaxAcceleration(), swerveConstants.getLookaheadDistance(), swerveConstants.getkV(), swerveConstants.getkA());
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.hKP = hKP;
        this.hKI = hKI;
        this.hKD = hKD;
    }

    public double getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }

    public void setMaxAngularAcceleration(double maxAngularAcceleration) {
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    public double gethKP() {
        return hKP;
    }

    public void sethKP(double hKP) {
        this.hKP = hKP;
    }

    public double gethKI() {
        return hKI;
    }

    public void sethKI(double hKI) {
        this.hKI = hKI;
    }

    public double gethKD() {
        return hKD;
    }

    public void sethKD(double hKD) {
        this.hKD = hKD;
    }
}
