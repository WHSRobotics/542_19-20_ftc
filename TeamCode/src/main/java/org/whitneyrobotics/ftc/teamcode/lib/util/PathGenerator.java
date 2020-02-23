package org.whitneyrobotics.ftc.teamcode.lib.util;

import java.util.ArrayList;

public class PathGenerator {

    private Position[] smoothedPath;
    int[][] anchorHeadings;

    public Position[] generatePosPath(Position[] targetPositions, double spacing, double weightSmooth) {
        smoothedPath = smoothPath(injectPoints(targetPositions, spacing), weightSmooth, false);

        return smoothedPath;
    }

    public Coordinate[] generateCoordPath(Position[] targetPositions, double spacing, double weightSmooth) {
        smoothedPath = smoothPath(injectPoints(targetPositions, spacing), weightSmooth, true);
        Coordinate[] smoothedCoordPath = new Coordinate[smoothedPath.length];
        for(int i = 0; i < smoothedPath.length; i++) {
            smoothedCoordPath[i] = new Coordinate(smoothedPath[i].getX(), smoothedPath[i].getY(), smoothedPath[i].getHeading());
        }

        int numAnchorHeadings = 0;
        for (int i = 0; i < smoothedCoordPath.length; i++) {
            if (!Double.isNaN(smoothedCoordPath[i].getHeading())) {
                numAnchorHeadings++;
            }
        }
        anchorHeadings = new int[numAnchorHeadings][2];
        int index = 0;
        for (int i = 0; i < smoothedCoordPath.length; i++) {
            if (!Double.isNaN(smoothedCoordPath[i].getHeading())) {
                anchorHeadings[index][0] = i;
                anchorHeadings[index][1] = (int)smoothedPath[i].getHeading();
                index++;
            }
        }

        for (int i = 0; i < numAnchorHeadings - 1; i++) {
            double distanceBetweenAnchors = calculateDistanceAtPoint()[anchorHeadings[i+1][0]] - calculateDistanceAtPoint()[anchorHeadings[i][0]];
            int headingBetweenAnchors = anchorHeadings[i+1][1] - anchorHeadings[i][1];
            for (int j = anchorHeadings[i][0]; j < anchorHeadings[i+1][0]; j++) {
                double scaleFactor = (calculateDistanceAtPoint()[j+1] - calculateDistanceAtPoint()[j])/distanceBetweenAnchors;
                smoothedCoordPath[j+1].setHeading(smoothedCoordPath[j].getHeading() + headingBetweenAnchors*scaleFactor);
            }
        }

        return smoothedCoordPath;
    }

    private Position[] injectPoints(Position[] orig, double spacing) {
        ArrayList<Position> morePoints = new ArrayList<Position>();

        for (int i = 0; i < orig.length - 1; i++) {
            Position segment = Functions.Positions.subtract(orig[i+1], orig[i]);
            double magnitude = segment.getMagnitude();
            int numSegmentsBetween = (int) Math.round(magnitude / spacing);
            segment.scale((double) 1 / numSegmentsBetween);

            morePoints.add(orig[i]);
            for (int j = 1; j < numSegmentsBetween; j++) {
                morePoints.add(Functions.Positions.add(orig[i], Functions.Positions.scale(j, segment)));
            }
        }
        morePoints.add(orig[orig.length - 1]);

        return morePoints.toArray(new Position[morePoints.size()]);
    }

    private Position[] smoothPath(Position[] orig, double weightSmooth, boolean includeHeadings) {
        Position[] smoothed;
        if (includeHeadings) {
            smoothed = new Coordinate[orig.length];
        } else {
            smoothed = new Position[orig.length];
        }
        for (int i = 0; i < orig.length; i++) {
            if (includeHeadings) {
                smoothed[i] = new Coordinate(orig[i].getX(), orig[i].getY(), orig[i].getHeading());
            } else {
                smoothed[i] = new Position(orig[i].getX(), orig[i].getY());
            }
        }

        double weightData = 1 - weightSmooth;
        double tolerance = 0.001;

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < orig.length - 1; i++) {
                double aux = smoothed[i].getX();
                smoothed[i].addX(weightData * (orig[i].getX() - smoothed[i].getX()) + weightSmooth * (smoothed[i - 1].getX() + smoothed[i + 1].getX() - (2.0 * smoothed[i].getX())));
                change += Math.abs(aux - smoothed[i].getX());

                aux = smoothed[i].getY();
                smoothed[i].addY(weightData * (orig[i].getY() - smoothed[i].getY()) + weightSmooth * (smoothed[i - 1].getY() + smoothed[i + 1].getY() - (2.0 * smoothed[i].getY())));
                change += Math.abs(aux - smoothed[i].getY());
            }
        }
        return smoothed;
    }

    private double[] calculateDistanceAtPoint() {
        //creates array to store the total distance that the robot should have traveled at that point
        double[] distanceArray = new double[smoothedPath.length];
        distanceArray[0] = 0;

        for (int i = 1; i < smoothedPath.length; i++) {
            distanceArray[i] = distanceArray[i - 1] + Math.hypot(Math.abs(smoothedPath[i].getX() - smoothedPath[i - 1].getX()), Math.abs(smoothedPath[i - 1].getY() - smoothedPath[i].getY()));
        }
        return distanceArray;
    }

    public double[] calculateTargetCurvatures() {
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

    public double[] calculateTargetVelocities(double k, double pathMaxVelocity, double maxAcceleration) {
        double[] targetCurvatures = calculateTargetCurvatures();

        // creates array that holds all of the target velocities
        double[] targetVelocities = new double[smoothedPath.length];

        // calculates the target velocities for each point
        targetVelocities[smoothedPath.length - 1] = 0; // last point target velocity is zero
        for (int i = smoothedPath.length - 2; i >= 0; i--) { // works backwards as we need to know last point's velocity to calculate current point's

            // distance from this current point to next point
            double distance = Functions.distanceFormula(smoothedPath[i].getX(), smoothedPath[i].getY(), smoothedPath[i + 1].getX(), smoothedPath[i + 1].getY());

            // finds the smaller value between the velocity constant / the curvature and a new target velocity
            double targetVelocity = Math.min(Math.min(pathMaxVelocity, k / targetCurvatures[i]), Math.sqrt(Math.pow(targetVelocities[i + 1], 2) + 2 * maxAcceleration * distance));
            targetVelocities[i] = targetVelocity;
        }
        return targetVelocities;
    }


    public double[] calculateTargetAngularVelocities(double maxAngularAcceleration){
        double[] targetAngularVelocities = new double[smoothedPath.length];
        targetAngularVelocities[smoothedPath.length-1] = 0;
        for (int i = smoothedPath.length-2; i >= 0; i--){
            double deltaTheta = smoothedPath[i+1].getHeading() - smoothedPath[i].getHeading();
            targetAngularVelocities[i] = (targetAngularVelocities[i+1] * targetAngularVelocities[i+1]) + 2 * maxAngularAcceleration * deltaTheta;
        }
        return targetAngularVelocities;
    }

}
