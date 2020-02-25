package org.whitneyrobotics.ftc.teamcode.lib.util;

import org.opencv.core.Mat;
import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;

public class StrafeToTarget {

    private Coordinate currentCoord;

    private static final double MAXIMUM_ACCELERATION = SwerveConstants.MAX_ACCELERATION; // mm/s^2
    private static final double MAXIMUM_VELOCITY = SwerveConstants.MAX_VELOCITY;
    private static final double MAXIMUM_ANGULAR_ACCELERATION = SwerveConstants.MAX_ANGULAR_ACCELERATION;
    private double pathMaximumVelocity;
    public int lastClosestPointIndex = 0;
    public int lastClosestHeadingIndex = 0;
    private int lastIndex = 0;
    private double currentTValue = 0;
    private static final double hKI = SwerveConstants.hKI;
    private static final double hKP = SwerveConstants.hKP;
    private static final double hKD = SwerveConstants.hKD;

    PIDController headingController = new PIDController(hKP, hKI, hKD);

    public Coordinate[] smoothedPath;
    private double[] targetCurvatures;
    public double[] targetVelocities;
    public double[] targetAngularVelocities;

    public boolean conditionMet = false;

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
    double angularVelocity;

    public double angleToLookaheadPointDebug = 0.0;

    public StrafeToTarget(double kP, double kV, double kA, Coordinate[] targetPositions, double spacing, double weightSmooth, double tolerance, double velocityConstant, double lookaheadDistance, double pathMaximumVelocity, double angularVelocity) {
        this.pathMaximumVelocity = pathMaximumVelocity;

        this.kP = kP;
        this.kV = 1 / MAXIMUM_VELOCITY;
        this.kA = kA;
        this.angularVelocity = angularVelocity;
        this.lookaheadDistance = lookaheadDistance;
        targetVelocityRateLimiter = new RateLimiter(MAXIMUM_ACCELERATION, 0);
        trackWidth = Drivetrain.getTrackWidth();
        wheelBase = Drivetrain.getWheelBase();
        PathGenerator pathGenerator = new PathGenerator();
        smoothedPath = pathGenerator.generateCoordPath(targetPositions, spacing, weightSmooth);
        targetVelocities = pathGenerator.calculateTargetVelocities(velocityConstant, pathMaximumVelocity, MAXIMUM_ACCELERATION);
        //targetAngularVelocities = pathGenerator.calculateTargetAngularVelocities(MAXIMUM_ANGULAR_ACCELERATION);
        targetAngularVelocities = new double[smoothedPath.length];
        for (int i = smoothedPath.length-2; i >= 0; i--){
            double deltaTheta = smoothedPath[i+1].getHeading() - smoothedPath[i].getHeading();
            double sign = 1;
            if (deltaTheta < 0){
                sign = -1;
            }
            targetAngularVelocities[i] = sign * Math.sqrt((targetAngularVelocities[i+1] * targetAngularVelocities[i+1]) + 2 * MAXIMUM_ANGULAR_ACCELERATION * Math.abs(deltaTheta));
        }
        lastTime = System.nanoTime() / 1E9;
    }

    public double[] calculateMotorPowers(Coordinate currentCoord, double[] currentBackVelocities, double frontRightVelocity) {
        this.currentCoord = currentCoord;
        double[] currentWheelVelocities = {currentBackVelocities[1] - (frontRightVelocity - currentBackVelocities[0]), frontRightVelocity, currentBackVelocities[0], currentBackVelocities[1]};

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
        int indexOfClosestHeading = calculateIndexOfClosestHeading(smoothedPath);

        Position vectorToLookaheadPoint = Functions.Positions.subtract(lookaheadPoint, currentCoord);
        vectorToLookaheadPoint = Functions.field2body(vectorToLookaheadPoint, currentCoord);
        double angleToLookaheadPoint = Math.toDegrees(Math.atan2(vectorToLookaheadPoint.getY(), vectorToLookaheadPoint.getX()));
        angleToLookaheadPointDebug = angleToLookaheadPoint;

        headingController.calculate(targetAngularVelocities[indexOfClosestHeading] - angularVelocity);
        double headingFeedback = headingController.getOutput();

        currentTargetWheelVelocities = calculateTargetWheelVelocities(targetVelocities[indexOfClosestPoint], angleToLookaheadPoint, targetAngularVelocities[indexOfClosestHeading]);

        double deltaTime = System.nanoTime() / 1E9 - lastTime;
        double[] targetWheelAccelerations = new double[4];
        for (int i = 0; i < targetWheelAccelerations.length; i++) {
            targetWheelAccelerations[i] = (currentTargetWheelVelocities[i] - lastTargetWheelVelocities[i]) / deltaTime;
        }
        if (indexOfClosestPoint != smoothedPath.length - 1) {
            double[] feedBack = {currentTargetWheelVelocities[0] - currentWheelVelocities[0], currentTargetWheelVelocities[1] - currentWheelVelocities[1], currentTargetWheelVelocities[2] - currentWheelVelocities[2], currentTargetWheelVelocities[3] - currentWheelVelocities[3]};
            for (int i = 0; i < feedBack.length; i++) {
                feedBack[i] *= kP;
            }

            double[] feedForwardVel = {kV * currentTargetWheelVelocities[0], kV * currentTargetWheelVelocities[1], kV * currentTargetWheelVelocities[2], kV * currentTargetWheelVelocities[3]};
            double[] feedForwardAccel = {kA * targetWheelAccelerations[0], kA * targetWheelAccelerations[1], kA * targetWheelAccelerations[2], kA * targetWheelAccelerations[3]};
            double[] feedForward = {feedForwardVel[0] + feedForwardAccel[0], feedForwardVel[1] + feedForwardAccel[1], feedForwardVel[2] + feedForwardAccel[2], feedForwardVel[3] + feedForwardAccel[3]};
            double[] motorPowers = {Functions.constrain(feedBack[0] + feedForward[0] - headingFeedback, -1, 1), Functions.constrain(feedBack[1] + feedForward[1] + headingFeedback, -1, 1), Functions.constrain(feedBack[2] + feedForward[2] - headingFeedback, -1, 1), Functions.constrain(feedBack[3] + feedForward[3] + headingFeedback, -1, 1)};
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

    private int calculateIndexOfClosestHeading(Position[] smoothedPath) {
        boolean closestHeadingFound = false;
        for (int i = lastClosestHeadingIndex; i < smoothedPath.length - 1; i++) {
            if (!closestHeadingFound && Math.abs(currentCoord.getHeading() - smoothedPath[i+1].getHeading()) < Math.abs(currentCoord.getHeading() - smoothedPath[lastClosestHeadingIndex].getHeading())) {
                lastClosestHeadingIndex = i + 1;
                if (i < smoothedPath.length - 2) {
                    if (Math.abs(smoothedPath[i + 2].getHeading() - currentCoord.getHeading()) >= Math.abs(smoothedPath[i+1].getHeading() - currentCoord.getHeading())) {
                        conditionMet = true;
                        closestHeadingFound = true;
                    }
                } else {
                    closestHeadingFound = true;
                }
            }
        }
        return lastClosestHeadingIndex;
    }

    public double[] calculateTargetTranslationalWheelVelocities(double targetVelocity, double angleToLookaheadPoint) {
        double targetVelocityX = targetVelocity * Functions.cosd(angleToLookaheadPoint);
        double targetVelocityY = targetVelocity * Functions.sind(angleToLookaheadPoint);
        double k = (trackWidth + wheelBase) / 2;

        double vFL = targetVelocityX - targetVelocityY;// - k * angleToLookaheadPoint;
        double vFR = targetVelocityX + targetVelocityY;// + k * angleToLookaheadPoint;
        double vBL = targetVelocityX + targetVelocityY;// - k * angleToLookaheadPoint;
        double vBR = targetVelocityX - targetVelocityY;// + k * angleToLookaheadPoint;

        return new double[]{vFL, vFR, vBL, vBR};
    }

    public double[] calculateTargetWheelVelocities(double targetVelocity, double angleToLookaheadPoint, double targetAngularVelocity) {
        targetAngularVelocity = Math.toRadians(targetAngularVelocity);
        double targetVelocityX = targetVelocity * Functions.cosd(angleToLookaheadPoint);
        double targetVelocityY = targetVelocity * Functions.sind(angleToLookaheadPoint);
        double k = (trackWidth + wheelBase) / 2;

        double vFL = targetVelocityX - targetVelocityY - k * targetAngularVelocity;
        double vFR = targetVelocityX + targetVelocityY + k * targetAngularVelocity;
        double vBL = targetVelocityX + targetVelocityY - k * targetAngularVelocity;
        double vBR = targetVelocityX - targetVelocityY + k * targetAngularVelocity;

        return new double[]{vFL, vFR, vBL, vBR};
    }


}