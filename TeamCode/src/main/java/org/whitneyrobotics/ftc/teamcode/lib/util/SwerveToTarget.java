package org.whitneyrobotics.ftc.teamcode.lib.util;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;

public class SwerveToTarget {

    private Coordinate currentCoord;

    private static final double MAXIMUM_ACCELERATION = SwerveConstants.MAX_ACCELERATION; // mm/s^2
    public int lastClosestPointIndex = 0;
    private int lastIndex = 0;
    private double currentTValue = 0;

    public Position[] smoothedPath;
    private double[] targetCurvatures;
    public double[] targetVelocities;

    private double[] currentTargetWheelVelocities = {0.0, 0.0};
    private double[] lastTargetWheelVelocities = {0.0, 0.0};
    private double lastTime;
    private double kP;
    private double kV;
    private double kA;
    private double lookaheadDistance;
    private double trackWidth;

    public Position lookaheadPoint;
    private boolean inProgress = false;

    /**
     *
     * @param kP                   Proportional constant for feedback loop
     * @param kV                   Velocity constant for feedforward loop
     * @param kA                   Acceleration constant for feedforward loop
     * @param targetPositions      Array of Positions to follow
     * @param spacing              Space between each injected point on the path (in mm)
     * @param weightSmooth         Determines the extent of smoothing done on the path (from 0-1)
     * @param turnSpeed            Determines the speed of the robot around turns (recommended 1-5)
     * @param lookaheadDistance    How far ahead along the path the robot will be "looking" (in mm)
     */
    public SwerveToTarget(double kP, double kV, double kA, Position[] targetPositions, double spacing, double weightSmooth, double turnSpeed, double lookaheadDistance) {
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.lookaheadDistance = lookaheadDistance;
        trackWidth = Drivetrain.getTrackWidth();
        smoothedPath = PathGenerator.generatePath(targetPositions, spacing, weightSmooth);
        targetCurvatures = calculateTargetCurvatures(smoothedPath);
        targetVelocities = calculateTargetVelocities(smoothedPath, turnSpeed);
        lastTime = System.nanoTime() / 1E9;
    }

    public double[] calculateMotorPowers(Coordinate currentCoord, double[] currentWheelVelocities, boolean backwards) {

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
        double curvature = calculateCurvature(lookaheadDistance, lookaheadPoint);
        currentTargetWheelVelocities = calculateTargetWheelVelocities(targetVelocities[indexOfClosestPoint], curvature);

        double deltaTime = System.nanoTime() / 1E9 - lastTime;
        double[] targetWheelAccelerations = {(currentTargetWheelVelocities[0] - lastTargetWheelVelocities[0]) / deltaTime, (currentTargetWheelVelocities[1] - lastTargetWheelVelocities[1]) / deltaTime};

        if (indexOfClosestPoint != smoothedPath.length - 1) {
            double[] feedBack = {currentTargetWheelVelocities[0] - currentWheelVelocities[0], currentTargetWheelVelocities[1] - currentWheelVelocities[1]};
            for (int i = 0; i < feedBack.length; i++) {
                feedBack[i] *= kP;
            }

            double[] feedForwardVel = {kV * currentTargetWheelVelocities[0], kV * currentTargetWheelVelocities[1]};
            double[] feedForwardAccel = {kA * targetWheelAccelerations[0], kA * targetWheelAccelerations[1]};
            double[] feedForward = {feedForwardVel[0] + feedForwardAccel[0], feedForwardVel[1] + feedForwardAccel[1]};
            double[] motorPowers = {Functions.constrain(feedBack[0] + feedForward[0], -1, 1), Functions.constrain(feedBack[1] + feedForward[1], -1, 1)};
            lastTargetWheelVelocities = currentTargetWheelVelocities;
            inProgress = true;
            return motorPowers;
        } else {
            inProgress = false;
        }
        return new double[]{0.0, 0.0};
    }

    private double[] calculateTargetCurvatures(Position[] smoothedPath) {
        // creates an array to store all the curvatures
        double[] curvatureArray = new double[smoothedPath.length];

        // sets the curvatures at the first and last point to 0
        curvatureArray[0] = 0;
        curvatureArray[smoothedPath.length - 1] = 0;

        // loops through the array to calculate the curvatures
        for (int i = 1; i < (smoothedPath.length - 2); i++) {

            // calculates the coordinates of the points directly ahead and behind point i
            double x1 = smoothedPath[i].getX() + 0.0001;
            double y1 = smoothedPath[i].getY();

            double x2 = smoothedPath[i - 1].getX();
            double y2 = smoothedPath[i - 1].getY();

            double x3 = smoothedPath[i + 1].getX();
            double y3 = smoothedPath[i + 1].getY();

            // calculates the curvatures and returns the array
            double k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2)) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);

            double b = 0.5 * (Math.pow(x2, 2) - 2 * x2 * k1 + Math.pow(y2, 2) - Math.pow(x3, 2) + 2 * x3 * k1 - Math.pow(y3, 2)) / (x3 * k2 - y3 + y2 - x2 * k2);
            double a = k1 - k2 * b;

            double r = Math.sqrt(Math.pow(x1 - a, 2) + (Math.pow(y1 - b, 2)));
            double curvature = 0.0;
            if (!Double.isNaN(r)) {
                curvature = 1 / r;
            }
            curvatureArray[i] = curvature;
        }
        return curvatureArray;
    }

    private double[] calculateTargetVelocities(Position[] smoothedPath, double k) {
        // creates array that holds all of the target velocities
        double[] targetVelocities = new double[smoothedPath.length];

        // calculates the target velocities for each point
        targetVelocities[smoothedPath.length - 1] = 0; // last point target velocity is zero
        for (int i = smoothedPath.length - 2; i >= 0; i--) { // works backwards as we need to know last point's velocity to calculate current point's

            // distance from this current point to next point
            double distance = Functions.Positions.subtract(smoothedPath[i+1], smoothedPath[i]).getMagnitude();

            // finds the smaller value between the velocity constant / the curvature and a new target velocity
            double targetVelocity = Math.min(k / targetCurvatures[i], Math.sqrt(Math.pow(targetVelocities[i + 1], 2) + 2 * MAXIMUM_ACCELERATION * distance));
            targetVelocities[i] = targetVelocity;
        }
        return targetVelocities;
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

    private double calculateCurvature(double lookaheadDistance, Position lookaheadPoint) {
        // robot line: ax + by + c = 0
        double a = -Functions.tand(currentCoord.getHeading());
        double b = 1;
        double c = Functions.tand(currentCoord.getHeading()) * currentCoord.getX() - currentCoord.getY();

        Position R = currentCoord.getPos();
        Position L = lookaheadPoint;
        // generate point B on robot line (for calculating sign)
        Position B = new Position(R.getX() + Functions.cosd(currentCoord.getHeading()), R.getY() + Functions.sind(currentCoord.getHeading()));

        Position RB = Functions.Positions.subtract(B, R);
        Position RL = Functions.Positions.subtract(L, R);

        // calculate which side of the robot line the lookahead point is on
        double side = Math.signum(Functions.Positions.getCross3dMagnitude(RL, RB));

        // distance from robot line to lookahead point: d = |ax + by + c| /√(a^2 + b^2)
        double distance = Math.abs(a * lookaheadPoint.getX() + b * lookaheadPoint.getY() + c) / Math.sqrt(a * a + b * b);

        double curvature = 2 * side * distance / (lookaheadDistance * lookaheadDistance);
        return curvature;
    }

    private double[] calculateTargetWheelVelocities(double targetVelocity, double curvature) {
        // calculates the wheel velocity at which the wheels should be
        double leftVelocity = targetVelocity * (2 + curvature * trackWidth) / 2;
        double rightVelocity = targetVelocity * (2 - curvature * trackWidth) / 2;

        return new double[]{leftVelocity, rightVelocity};
    }

    public boolean inProgress() {
        return inProgress;
    }

    public double[] getCurrentTargetWheelVelocities() {
        return currentTargetWheelVelocities;
    }
}