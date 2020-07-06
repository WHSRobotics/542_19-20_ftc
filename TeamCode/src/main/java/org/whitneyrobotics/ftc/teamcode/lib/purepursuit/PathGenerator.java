package org.whitneyrobotics.ftc.teamcode.lib.purepursuit;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;

import java.util.ArrayList;

public class PathGenerator {

    public static Path generatePath(ArrayList<Position> targetPositions, double spacing, double weightSmooth) {
        boolean coordinatePath = false;
        ArrayList<Position> positionList;
        for (Position pos : targetPositions) {
            if (pos instanceof Coordinate) {
                coordinatePath = true;
                break;
            }
        }
        if (coordinatePath) {
            positionList = generateCoordPath(targetPositions, spacing, weightSmooth);
        } else {
            positionList = generatePosPath(targetPositions, spacing, weightSmooth);
        }
    }

    private static ArrayList<Position> generatePosPath(ArrayList<Position> targetPositions, double spacing, double weightSmooth) {
        return smoothPath(injectPoints(targetPositions, spacing), weightSmooth);
    }

    private static ArrayList<Position> generateCoordPath(ArrayList<Position> targetPositions, double spacing, double weightSmooth) {
        ArrayList<Position> smoothedPosPath = smoothPath(injectPoints(targetPositions, spacing), weightSmooth);
        double[] distanceAtPoint = calculateDistanceAtPoint(smoothedPosPath);

        int numAnchorHeadings = 0;
        for (Position pos : smoothedPosPath) {
            if (pos instanceof Coordinate) {
                numAnchorHeadings++;
            }
        }
        int[][] anchorHeadings = new int[numAnchorHeadings][2];
        int index = 0;
        for (int i = 0; i < smoothedPosPath.size(); i++) {
            if (smoothedPosPath.get(i) instanceof Coordinate) {
                anchorHeadings[index][0] = i;
                anchorHeadings[index][1] = (int) ((Coordinate) smoothedPosPath.get(i)).getHeading();
                index++;
            }
        }

        for (int i = 0; i < numAnchorHeadings - 1; i++) {
            double distanceBetweenAnchors = distanceAtPoint[anchorHeadings[i + 1][0]] - distanceAtPoint[anchorHeadings[i][0]];
            int headingBetweenAnchors = anchorHeadings[i + 1][1] - anchorHeadings[i][1];
            for (int j = anchorHeadings[i][0]; j < anchorHeadings[i + 1][0]; j++) {
                double scaleFactor = (distanceAtPoint[j + 1] - distanceAtPoint[j]) / distanceBetweenAnchors;
                ((Coordinate) smoothedPosPath.get(j + 1)).setHeading(((Coordinate) smoothedPosPath.get(j)).getHeading() + headingBetweenAnchors * scaleFactor);
            }
        }

        return smoothedPosPath;
    }

    private static ArrayList<Position> injectPoints(ArrayList<Position> orig, double spacing) {
        ArrayList<Position> morePoints = new ArrayList<Position>();

        for (int i = 0; i < orig.size() - 1; i++) {
            Position segment = Functions.Positions.subtract(orig.get(i + 1), orig.get(i));
            double magnitude = segment.getMagnitude();
            int numSegmentsBetween = (int) Math.round(magnitude / spacing);
            segment.scale((double) 1 / numSegmentsBetween);

            morePoints.add(orig.get(i));
            for (int j = 1; j < numSegmentsBetween; j++) {
                morePoints.add(Functions.Positions.add(orig.get(i), Functions.Positions.scale(j, segment)));
            }
        }
        morePoints.add(orig.get(orig.size() - 1));

        return morePoints;
    }

    private static ArrayList<Position> smoothPath(ArrayList<Position> orig, double weightSmooth) {
        ArrayList<Position> smoothed = new ArrayList<Position>();
        for (Position pos : orig) {
            smoothed.add(pos);
        }

        double weightData = 1 - weightSmooth;
        double tolerance = 0.001;

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < orig.size() - 1; i++) {
                double aux = smoothed.get(i).getX();
                smoothed.get(i).addX(weightData * (orig.get(i).getX() - smoothed.get(i).getX()) + weightSmooth * (smoothed.get(i - 1).getX() + smoothed.get(i + 1).getX() - (2.0 * smoothed.get(i).getX())));
                change += Math.abs(aux - smoothed.get(i).getX());

                aux = smoothed.get(i).getY();
                smoothed.get(i).addY(weightData * (orig.get(i).getY() - smoothed.get(i).getY()) + weightSmooth * (smoothed.get(i - 1).getY() + smoothed.get(i + 1).getY() - (2.0 * smoothed.get(i).getY())));
                change += Math.abs(aux - smoothed.get(i).getY());
            }
        }
        return smoothed;
    }

    private static double[] calculateDistanceAtPoint(ArrayList<Position> smoothedPath) {
        //creates array to store the total distance that the robot should have traveled at that point
        double[] distanceArray = new double[smoothedPath.size()];
        distanceArray[0] = 0;

        for (int i = 1; i < smoothedPath.size(); i++) {
            distanceArray[i] = distanceArray[i - 1] + Math.hypot(Math.abs(smoothedPath.get(i).getX() - smoothedPath.get(i - 1).getX()), Math.abs(smoothedPath.get(i - 1).getY() - smoothedPath.get(i).getY()));
        }
        return distanceArray;
    }

    private static double[] calculateTargetCurvatures(ArrayList<Position> smoothedPath) {
        // creates an array to store all the curvatures
        double[] curvatureArray = new double[smoothedPath.size()];

        // sets the curvatures at the first and last point to 0
        curvatureArray[0] = 0;
        curvatureArray[smoothedPath.size() - 1] = 0;

        // loops through the array to calculate the curvatures
        for (int i = 1; i < (smoothedPath.size() - 2); i++) {

            // calculates the coordinates of the points directly ahead and behind point i
            double x1 = smoothedPath.get(i).getX() + 0.0001;
            double y1 = smoothedPath.get(i).getY();

            double x2 = smoothedPath.get(i - 1).getX();
            double y2 = smoothedPath.get(i - 1).getY();

            double x3 = smoothedPath.get(i - 1).getX();
            double y3 = smoothedPath.get(i - 1).getY();

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

    private static double[] calculateTargetVelocities(double k, double pathMaxVelocity, double maxAcceleration, ArrayList<Position> smoothedPath) {
        double[] targetCurvatures = calculateTargetCurvatures(smoothedPath);

        // creates array that holds all of the target velocities
        double[] targetVelocities = new double[smoothedPath.size()];

        // calculates the target velocities for each point
        targetVelocities[smoothedPath.size() - 1] = 0; // last point target velocity is zero
        for (int i = smoothedPath.size() - 2; i >= 0; i--) { // works backwards as we need to know last point's velocity to calculate current point's

            // distance from this current point to next point
            double distance = Functions.distanceFormula(smoothedPath.get(i).getX(), smoothedPath.get(i).getY(), smoothedPath.get(i + 1).getX(), smoothedPath.get(i + 1).getY());

            // finds the smaller value between the velocity constant / the curvature and a new target velocity
            double targetVelocity = Math.min(Math.min(pathMaxVelocity, k / targetCurvatures[i]), Math.sqrt(Math.pow(targetVelocities[i + 1], 2) + 2 * maxAcceleration * distance));
            targetVelocities[i] = targetVelocity;
        }
        return targetVelocities;
    }


    private static double[] calculateTargetAngularVelocities(double maxAngularAcceleration, ArrayList<Position> smoothedPath) {
        double[] targetAngularVelocities = new double[smoothedPath.size()];
        targetAngularVelocities[smoothedPath.size() - 1] = 0;
        for (int i = smoothedPath.size() - 2; i >= 0; i--) {
            double deltaTheta = ((Coordinate) smoothedPath.get(i + 1)).getHeading() - ((Coordinate) smoothedPath.get(i)).getHeading();
            targetAngularVelocities[i] = (targetAngularVelocities[i + 1] * targetAngularVelocities[i + 1]) + 2 * maxAngularAcceleration * deltaTheta;
        }
        return targetAngularVelocities;
    }

}
