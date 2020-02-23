package org.whitneyrobotics.ftc.teamcode.lib.util;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;

public class StrafeToTarget {

    private Coordinate currentCoord;

    private static final double MAXIMUM_ACCELERATION = SwerveConstants.MAX_ACCELERATION; // mm/s^2
    private static final double MAXIMUM_VELOCITY = SwerveConstants.MAX_VELOCITY;
    private double pathMaximumVelocity;
    public int lastClosestPointIndex = 0;
    private int lastIndex = 0;
    private double currentTValue = 0;

    public Coordinate[] smoothedPath;
    private double[] targetCurvatures;
    public double[] targetVelocities;

    private double[] currentTargetWheelVelocities = {0.0, 0.0, 0.0, 0.0};
    private double[] lastTargetWheelVelocities = {0.0, 0.0, 0.0, 0.0};
    private double lastTime;
    private RateLimiter targetVelocityRateLimiter;
    private double kP;
    private double kV;
    private double kA;
    private double lookaheadDistance;
    private double trackWidth;
    private double wheelBase;

    public Position lookaheadPoint;
    private boolean inProgress;

    public StrafeToTarget(double kP, double kV, double kA, Position[] targetPositions, double spacing, double weightSmooth, double tolerance, double velocityConstant, double lookaheadDistance, double pathMaximumVelocity) {
        this.pathMaximumVelocity = pathMaximumVelocity;
        this.kP = kP;
        this.kV = 1 / MAXIMUM_VELOCITY;
        this.kA = kA;
        this.lookaheadDistance = lookaheadDistance;
        targetVelocityRateLimiter = new RateLimiter(MAXIMUM_ACCELERATION, 0);
        trackWidth = Drivetrain.getTrackWidth();
        wheelBase = Drivetrain.getWheelBase();
        PathGenerator pathGenerator = new PathGenerator();
        smoothedPath = pathGenerator.generateCoordPath(targetPositions, spacing, weightSmooth);
        targetVelocities = pathGenerator.calculateTargetVelocities(velocityConstant, pathMaximumVelocity, MAXIMUM_ACCELERATION);
        lastTime = System.nanoTime() / 1E9;
    }

    public double[] calculateMotorPowers(Coordinate currentCoord, double[] currentWheelVelocities) {
        this.currentCoord = currentCoord;

        boolean tFound = false;
        for (int i = lastIndex; i < smoothedPath.length - 1; i++) {
            Double nextTValue = new Double(calculateT(smoothedPath[i], smoothedPath[i + 1], lookaheadDistance));

            if (!tFound && !nextTValue.isNaN() && (nextTValue + i) > (currentTValue + lastIndex)) {
                tFound = true;
                currentTValue = nextTValue;
                lastIndex = i;
            }
        }

        Position calculatedTStartPoint = smoothedPath[lastIndex];
        Position calculatedTEndPoint = smoothedPath[lastIndex + 1];
        lookaheadPoint = Functions.Positions.add(calculatedTStartPoint, Functions.Positions.scale(currentTValue, Functions.Positions.subtract(calculatedTEndPoint, calculatedTStartPoint)));

        int indexOfClosestPoint = calculateIndexOfClosestPoint(smoothedPath);

        Position vectorToLookaheadPoint = Functions.Positions.subtract(lookaheadPoint, currentCoord);
        vectorToLookaheadPoint = Functions.field2body(vectorToLookaheadPoint, currentCoord);
        double angleToLookaheadPoint = Math.atan2(vectorToLookaheadPoint.getY(), vectorToLookaheadPoint.getX());

        currentTargetWheelVelocities = calculateTargetWheelVelocities(targetVelocities[indexOfClosestPoint], angleToLookaheadPoint);

        double deltaTime = System.nanoTime() / 1E9 - lastTime;
        double[] targetWheelAccelerations = new double[4];
        for (int i = 0; i < targetWheelAccelerations.length; i++) {
            targetWheelAccelerations[i] = (currentTargetWheelVelocities[i] - lastTargetWheelVelocities[i]) / deltaTime;
        }

        if (indexOfClosestPoint != smoothedPath.length - 1) {
            double[] feedBack = {currentTargetWheelVelocities[0] - currentWheelVelocities[1], currentTargetWheelVelocities[1] - currentWheelVelocities[0], currentTargetWheelVelocities[2] - currentWheelVelocities[0], currentTargetWheelVelocities[3] - currentWheelVelocities[1]};
            for (int i = 0; i < feedBack.length; i++) {
                feedBack[i] *= kP;
            }

            double[] feedForwardVel = {kV * currentTargetWheelVelocities[0], kV * currentTargetWheelVelocities[1], kV * currentTargetWheelVelocities[2], kV * currentTargetWheelVelocities[3]};
            double[] feedForwardAccel = {kA * targetWheelAccelerations[0], kA * targetWheelAccelerations[1], kA * targetWheelAccelerations[2], kA * targetWheelAccelerations[3]};
            double[] feedForward = {feedForwardVel[0] + feedForwardAccel[0], feedForwardVel[1] + feedForwardAccel[1], feedForwardVel[2] + feedForwardAccel[2], feedForwardVel[3] + feedForwardAccel[3]};
            double[] motorPowers = {Functions.constrain(feedBack[0] + feedForward[0], -1, 1), Functions.constrain(feedBack[1] + feedForward[1], -1, 1), Functions.constrain(feedBack[2] + feedForward[2], -1, 1), Functions.constrain(feedBack[3] + feedForward[3], -1, 1)};
            lastTargetWheelVelocities = currentTargetWheelVelocities;
            inProgress = true;
            return motorPowers;
        } else {
            inProgress = false;
        }
        return new double[] {0.0, 0.0, 0.0, 0.0};
    }

    private double calculateT(Position lineStart, Position lineEnd, double lookaheadDistance) {
        // constants used throughout the method
        Position d = Functions.Positions.subtract(lineEnd, lineStart);
        Position f = Functions.Positions.subtract(lineStart, currentCoord);
        double r = lookaheadDistance;

        double a = Functions.Positions.dot(d, d);
        double b = 2 * Functions.Positions.dot(f, d);
        double c = Functions.Positions.dot(f, f) - r * r;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            // no intersection
        } else {
            // ray didn't totally miss sphere, so there is a solution to the equation.
            discriminant = Math.sqrt(discriminant);

            // either solution may be on or off the ray so need to test both
            // t1 is always the smaller value, because BOTH discriminant and a are nonnegative.
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);

            // 3x HIT cases:
            //          -o->             --|-->  |            |  --|->
            // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

            // 3x MISS cases:
            //       ->  o                     o ->              | -> |
            // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

            if (t1 >= 0 && t1 <= 1) {
                // t1 is the intersection, and it's closer than t2 (since t1 uses -b - discriminant)
                // Impale, Poke
                return t1;
            }

            if (t2 >= 0 && t2 <= 1) {
                return t2;
            }
        }
        return Double.NaN;
    }

    private int calculateIndexOfClosestPoint(Position[] smoothedPath) {
        // creates array in which we store the current distance to each point in our path
        double[] distances = new double[smoothedPath.length];
        for (int i = 0/*lastClosestPointIndex*/; i < smoothedPath.length; i++) {
            distances[i] = Functions.Positions.subtract(smoothedPath[i], currentCoord).getMagnitude();
        }

        // calculates the index of value in the array with the smallest value and returns that index
        lastClosestPointIndex = Functions.calculateIndexOfSmallestValue(distances);
        return lastClosestPointIndex;
    }

    private double[] calculateTargetWheelVelocities(double targetVelocity, double angleToLookaheadPoint) {
        double targetVelocityX = targetVelocity * Functions.cosd(angleToLookaheadPoint);
        double targetVelocityY = targetVelocity * Functions.sind(angleToLookaheadPoint);
        double k = (trackWidth + wheelBase) / 2;

        double vFL = targetVelocityX - targetVelocityY - k * angleToLookaheadPoint;
        double vFR = targetVelocityX + targetVelocityY + k * angleToLookaheadPoint;
        double vBL = targetVelocityX + targetVelocityY - k * angleToLookaheadPoint;
        double vBR = targetVelocityX - targetVelocityY + k * angleToLookaheadPoint;

        return new double[]{vFL, vFR, vBL, vBR};
    }
}