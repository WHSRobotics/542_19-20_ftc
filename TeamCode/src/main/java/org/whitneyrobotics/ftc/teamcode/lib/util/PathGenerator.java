package org.whitneyrobotics.ftc.teamcode.lib.util;

import java.util.ArrayList;

public class PathGenerator {

    public static Position[] generatePath(Position[] targetPositions, double spacing, double weightSmooth) {
        Position[] injectedPath = injectPoints(targetPositions, spacing);
        Position[] smoothedPath = smoothPath(injectedPath, weightSmooth);

        return smoothedPath;
    }


    private static Position[] injectPoints(Position[] orig, double spacing) {
        // create extended 2 Dimensional array to hold additional points

        ArrayList<Position> morePoints = new ArrayList<Position>();

        for (int i = 0; i < orig.length - 1; i++) {
            Position segment = Functions.Positions.subtract(orig[i+1], orig[i]);
            double magnitude = segment.get2dMagnitude();
            int numPointsBetween = (int) Math.round(magnitude / spacing);
            segment.scale(spacing / magnitude);
            morePoints.add(orig[i]);
            for (int j = 1; j <= numPointsBetween; j++) {
                morePoints.add(Functions.Positions.add(orig[i], Functions.Positions.scale2d(j, segment)));
            }
        }

        morePoints.add(orig[orig.length - 1]);

        return morePoints.toArray(new Position[morePoints.size()]);
    }

    private static Position[] smoothPath(Position[] path, double weightSmooth) {
        Position[] smoothedPath = path.clone();

        double weightData = 1 - weightSmooth;
        double tolerance = 0.001;

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++) {
                double aux = smoothedPath[i].getX();
                smoothedPath[i].addX(weightData * (path[i].getX() - smoothedPath[i].getX()) + weightSmooth * (smoothedPath[i - 1].getX() + smoothedPath[i + 1].getX() - (2.0 * smoothedPath[i].getX())));
                change += Math.abs(aux - smoothedPath[i].getX());

                smoothedPath[i].addY(weightData * (path[i].getY() - smoothedPath[i].getY()) + weightSmooth * (smoothedPath[i - 1].getY() + smoothedPath[i + 1].getY() - (2.0 * smoothedPath[i].getY())));
                change += Math.abs(aux - smoothedPath[i].getY());
            }
        }
        return smoothedPath;
    }

}
