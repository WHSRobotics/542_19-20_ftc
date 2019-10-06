package org.whitneyrobotics.ftc.teamcode.lib.util;

import java.util.ArrayList;


import java.util.ArrayList;

public class PathGenerator {
    public static int numSegmentsBetween;
    public static Position segment;

    public static Position[] generatePath(Position[] targetPositions, double spacing, double weightSmooth) {
        Position[] injectedPath = injectPoints(targetPositions, spacing);
        Position[] smoothedPath = smoothPath(injectedPath, weightSmooth);

        return smoothedPath;
    }


    public static Position[] injectPoints(Position[] orig, double spacing) {
        // create extended 2 Dimensional array to hold additional points

        ArrayList<Position> morePoints = new ArrayList<Position>();

        for (int i = 0; i < orig.length - 1; i++) {
            segment = Functions.Positions.subtract(orig[i+1], orig[i]);

            double magnitude = segment.getMagnitude();
            numSegmentsBetween = (int) Math.round(magnitude / spacing);
            segment.scale((double) 1 / numSegmentsBetween);
            morePoints.add(orig[i]);
            for (int j = 1; j < numSegmentsBetween; j++) {
                morePoints.add(Functions.Positions.add(orig[i], Functions.Positions.scale2d(j, segment)));
            }
        }

        morePoints.add(orig[orig.length - 1]);

        return morePoints.toArray(new Position[morePoints.size()]);
    }

    private static Position[] smoothPath(Position[] path, double weightSmooth) {
        double weightData = 1 - weightSmooth;
        double tolerance = 0.001;

        double[][] path1 = new double[path.length][2];
        for(int i = 0; i < path.length; i++) {
            path1[i][0] = path[i].getX();
            path1[i][1] = path[i].getY();
        }

        double[][] newPath = Functions.doubleArrayCopy(path1);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path1.length - 1; i++) {
                for (int j = 0; j < path1[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weightData * (path1[i][j] - newPath[i][j]) + weightSmooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }
        Position[] smoothedPath = new Position[path.length];
        for (int i = 0; i < path.length; i++) {
            smoothedPath[i] = new Position(newPath[i][0], newPath[i][1]);
        }


        return smoothedPath;
    }

}
