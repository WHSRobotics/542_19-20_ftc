package org.whitneyrobotics.ftc.teamcode.lib.purepursuit.swervetotarget;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.motion.RateLimiter;
import org.whitneyrobotics.ftc.teamcode.lib.purepursuit.PurePursuitRobotConstants;
import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;
import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;

public class SwerveFollower {

    SwervePath path;

    public int lastClosestPointIndex = 0;
    private int lastIndex = 0;
    private double currentTValue = 0;

    private double[] lastTargetWheelVelocities = {0.0, 0.0};
    private double[] currentTargetWheelVelocities = {0.0,0.0};

    private double lastTime;
    private RateLimiter targetVelocityRateLimiter;

    private final double KP = PurePursuitRobotConstants.SWERVE_KP;
    private final double KV = PurePursuitRobotConstants.SWERVE_KV;
    private final double KA = PurePursuitRobotConstants.SWERVE_KA;

    private double trackWidth = Drivetrain.getTrackWidth();

    private boolean inProgress = false;
    public SwerveFollower(SwervePath path) {
        this.path = path;
        targetVelocityRateLimiter = new RateLimiter(PurePursuitRobotConstants.MAX_ACCELERATION, 0);
        lastTime = System.nanoTime() / 1E9;
    }

    public double[] calculateMotorPowers(Coordinate currentCoord, double[] currentWheelVelocities) {
        Position lookaheadPoint;
        if (path.backwards()) currentCoord.setHeading(Functions.normalizeAngle(currentCoord.getHeading() + 180));

        boolean tFound = false;
        for (int i = lastIndex; i < path.size() - 1; i++) {
            Double nextTValue = new Double(calculateT(path.getPositionAtIndex(i), path.getPositionAtIndex(i + 1), path.getFollowerConstants(), currentCoord));

            if (!tFound && !nextTValue.isNaN() && (nextTValue + i) > (currentTValue + lastIndex)) {
                tFound = true;
                currentTValue = nextTValue;
                lastIndex = i;
            }
        }

        Position calculatedTStartPoint = path.getPositionAtIndex(lastIndex);
        Position calculatedTEndPoint = path.getPositionAtIndex(lastIndex + 1);
        lookaheadPoint = Functions.Positions.add(calculatedTStartPoint, Functions.Positions.scale(currentTValue, Functions.Positions.subtract(calculatedTEndPoint, calculatedTStartPoint)));

        int indexOfClosestPoint = calculateIndexOfClosestPoint(path,currentCoord);
        double curvature = calculateCurvature(path.getFollowerConstants(), lookaheadPoint, currentCoord);
        currentTargetWheelVelocities = calculateTargetWheelVelocities(path.getTargetVelocityAtIndex(indexOfClosestPoint), curvature);
        if (path.backwards()) {
            currentWheelVelocities = new double[] {-currentWheelVelocities[1], -currentWheelVelocities[0]};
        }

        double deltaTime = System.nanoTime() / 1E9 - lastTime;
        double[] targetWheelAccelerations = {(currentTargetWheelVelocities[0] - lastTargetWheelVelocities[0]) / deltaTime, (currentTargetWheelVelocities[1] - lastTargetWheelVelocities[1]) / deltaTime};

        if (indexOfClosestPoint != path.size() - 1) {
            double[] feedBack = {currentTargetWheelVelocities[0] - currentWheelVelocities[0], currentTargetWheelVelocities[1] - currentWheelVelocities[1]};
            for (int i = 0; i < feedBack.length; i++) {
                feedBack[i] *= KP;
            }

            double[] feedForwardVel = {KV * currentTargetWheelVelocities[0], KV * currentTargetWheelVelocities[1]};
            double[] feedForwardAccel = {KA * targetWheelAccelerations[0], KA * targetWheelAccelerations[1]};
            double[] feedForward = {feedForwardVel[0] + feedForwardAccel[0], feedForwardVel[1] + feedForwardAccel[1]};
            double[] motorPowers = {Functions.constrain(feedBack[0] + feedForward[0], -1, 1), Functions.constrain(feedBack[1] + feedForward[1], -1, 1)};
            lastTargetWheelVelocities = currentTargetWheelVelocities;
            inProgress = true;
            if (path.backwards()) {
                return new double[] {-motorPowers[1], -motorPowers[0]};
            }
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

    private double calculateT(Position lineStart, Position lineEnd, double lookaheadDistance, Coordinate currentCoord) {
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

    private int calculateIndexOfClosestPoint(SwervePath path, Coordinate currentCoord) {
        // creates array in which we store the current distance to each point in our path
        double[] distances = new double[path.size()];
        for (int i = 0/*lastClosestPointIndex*/; i < distances.length; i++) {
            distances[i] = Functions.Positions.subtract(path.getPositionAtIndex(i), currentCoord).getMagnitude();
        }

        // calculates the index of value in the array with the smallest value and returns that index
        lastClosestPointIndex = Functions.calculateIndexOfSmallestValue(distances);
        return lastClosestPointIndex;
    }

    private double calculateCurvature(double lookaheadDistance, Position lookaheadPoint, Coordinate currentCoord) {
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
        double rateLimitedTargetVelocity = targetVelocityRateLimiter.calculateOutput(targetVelocity);
        double leftVelocity = rateLimitedTargetVelocity * (2 + curvature * trackWidth) / 2;
        double rightVelocity = rateLimitedTargetVelocity * (2 - curvature * trackWidth) / 2;

        return new double[]{leftVelocity, rightVelocity};
    }

    public boolean inProgress() {
        return inProgress;
    }

    public double[] getCurrentTargetWheelVelocities() {
        return currentTargetWheelVelocities;
    }

}