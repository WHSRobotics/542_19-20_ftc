package org.whitneyrobotics.ftc.teamcode.lib.geometry;

/**
 * Class for storing positions on the field
 *
 * @see Coordinate - Alternative class, with heading
 */

public class Position {
    private double xPos;
    private double yPos;

    public Position(double x, double y) {
        xPos = x;
        yPos = y;
    }

    public void scale(double scaleFactor) {
        xPos = xPos* scaleFactor;
        yPos = yPos*scaleFactor;
    }

    public void addX(double x) {
        xPos += x;
    }

    public void addY(double y) {
        yPos += y;
    }

    public double getMagnitude() {
        return Math.hypot(xPos, yPos);
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }
/*
    public double getHeading() {
        return Double.NaN;
    }*/

    public void setX(double x) {
        xPos = x;
    }

    public void setY(double y) {
        yPos = y;
    }
}
