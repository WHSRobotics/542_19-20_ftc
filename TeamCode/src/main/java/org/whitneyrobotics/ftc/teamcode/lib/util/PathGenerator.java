package org.whitneyrobotics.ftc.teamcode.lib.util;

import java.util.ArrayList;

public class PathGenerator {

    public static Position[] generatePath(Position[] targetPositions, double spacing, double weightSmooth) {
        Position[] injectedPath = injectPoints(targetPositions, spacing);
        Position[] smoothedPath = smoothPath(injectedPath, weightSmooth);

        return smoothedPath;
    }


    private static Position[] injectPoints(Position[] orig, double spacing) {
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

    private static Position[] smoothPath(Position[] orig, double weightSmooth) {
        Position[] smoothed = new Position[orig.length];
        for (int i = 0; i < orig.length; i++) {
            smoothed[i] = new Position(orig[i].getX(), orig[i].getY());
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
}
