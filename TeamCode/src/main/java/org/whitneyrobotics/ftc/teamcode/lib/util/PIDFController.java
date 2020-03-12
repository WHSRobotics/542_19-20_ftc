package org.whitneyrobotics.ftc.teamcode.lib.util;

public class PIDFController {
    double kA = 0;
    double kV = 0;
    double kP = 0;
    double kI = 0;
    double kD = 0;

    double lastKnownTime;
    double lastKnownError;
    double integral;
    MotionProfile motionProfile;

    public PIDFController (MotionProfile motionProfile){
        this.motionProfile = motionProfile;
    }

    public void setConstants(double kA, double kV, double kP, double kI, double kD){
        this.kA = kA;
        this.kV = kV;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void init(double initialError) {
        lastKnownTime = System.nanoTime() / 1E9;
        lastKnownError = initialError;
        integral = 0;
    }

    public void calculate(double currentVelocity){
        double[] differenceArray = new double[motionProfile.getPoints().length];
        for(int i = 0; i < motionProfile.getPoints().length; i++){
            differenceArray[i] = currentVelocity - motionProfile.getPoints()[i];
        }
        int indexOfClosestPoint = Functions.calculateIndexOfSmallestValue(differenceArray);
        double targetVelocity = motionProfile.getTargetVelocities()[indexOfClosestPoint];
        double feedforwardVel = kV * targetVelocity;
    }

}
