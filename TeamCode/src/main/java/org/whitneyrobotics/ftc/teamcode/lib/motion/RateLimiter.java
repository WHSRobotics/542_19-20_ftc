package org.whitneyrobotics.ftc.teamcode.lib.motion;

import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;

public class RateLimiter {
    public double lastKnownOutput;
    public double rateLimitedOutput;
    public double maxRate;
    public boolean firstCall = true;
    public double lastKnownTime;

    public RateLimiter(double maxRate, double firstInput) {
        this.maxRate = maxRate;
        rateLimitedOutput = firstInput;
    }

    /**
     * limits how fast the input can change
     *
     * @param input                   the thing you want to limit
     * @return Returns the new limited rate
     */
    public double calculateOutput(double input) {
        if (firstCall) {
            lastKnownTime = System.nanoTime() / 1E9;
            lastKnownOutput = rateLimitedOutput;
            firstCall = false;
        }
        double time = System.nanoTime() / 1E9;
        double maxChange = (time - lastKnownTime) * maxRate;
        rateLimitedOutput += Functions.constrain(input - lastKnownOutput, -maxChange, maxChange);
        lastKnownOutput = rateLimitedOutput;
        lastKnownTime = time;
        return rateLimitedOutput;
    }

}
