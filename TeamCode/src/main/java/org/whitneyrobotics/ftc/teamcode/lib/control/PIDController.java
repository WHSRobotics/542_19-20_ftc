package org.whitneyrobotics.ftc.teamcode.lib.control;

public class PIDController {

    private double KP = 0;
    private double KI = 0;
    private double KD = 0;

    private double lastKnownTime = 0;
    private double lastKnownError = 0;

    private double error = 0;
    private double integral = 0;
    private double derivative = 0;

    public PIDController(double KP, double KI, double KD) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public void init(double initialError) {
        lastKnownTime = System.nanoTime() / 1E9;
        lastKnownError = initialError;
        integral = 0;
    }

    public void setConstants(double KP, double KI, double KD) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    public void calculate(double error) {
        this.error = error;

        //Integral & Derivative
        double currentTime = System.nanoTime() / 1E9;
        double deltaTime = currentTime - lastKnownTime;
        lastKnownTime = currentTime;

        //Integral
        integral += error * deltaTime;

        //Derivative
        double deltaError = error - lastKnownError;
        lastKnownError = error;
        derivative = deltaError / deltaTime;

    }

    public double getOutput() {
        return KP * error + KI * integral + KD * derivative;

    }

    public double getIntegral(){
        return integral;
    }

    public double getDerivative(){
        return derivative;
    }

}
