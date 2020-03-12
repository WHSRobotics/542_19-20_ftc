package org.whitneyrobotics.ftc.teamcode.lib.util;

public class MotionProfile {
    int numOfPoints;
    double[] pointArray;
    double MAXIMUM_ACCELERATION = 900;
    double MAXIMUM_VELOCITY = 2700;
    double increment;
    double[] targetVelocities;
    double[] targetAccelerations;
    double initalPos;

    public MotionProfile(double initialPos, double finalPos, int numOfPoints){
        this.numOfPoints = numOfPoints;
        increment = (finalPos - initialPos)/ this.numOfPoints;
        pointArray = getPoints();
        targetVelocities = generateTargetVelocities();
        this.initalPos = initialPos;
    }


    public double[] generateTargetVelocities(){
        double[] targetVelocities = new double[numOfPoints];
        targetVelocities[numOfPoints-1] = 0;
        for (int i =numOfPoints - 2; i < 0; i--){
            targetVelocities[i] = Math.sqrt(targetVelocities[i+2]*targetVelocities[i+2] - 2*MAXIMUM_ACCELERATION*increment);
            targetVelocities[i] = Functions.constrain(targetVelocities[i], -1, 1 );
        }
        return targetVelocities;
    }

    public double[] generateTargetAccelerations(){

    }

    public double[] getTargetVelocities() {
        return targetVelocities;
    }
    public double[] getTargetAccelerations() {
        return targetAccelerations;
    }

    public double[] getPoints(){
        double[] pointArray = new double[numOfPoints];
        double previousPos = initalPos;
        for(int i = 0; i < numOfPoints; i++){
            pointArray[i] = previousPos + increment;
            previousPos = pointArray[i];
        }

        return pointArray;
    }
}
