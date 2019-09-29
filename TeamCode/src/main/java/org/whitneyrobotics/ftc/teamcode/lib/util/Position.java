package org.whitneyrobotics.ftc.teamcode.lib.util;

/**
 * Class for storing positions on the field
 *
 * @see Coordinate - Alternative class, with heading
 */

public class Position {
    protected double xPos;
    protected double yPos;
    protected double zPos;

    public Position(double x, double y, double z) {
        xPos = x;
        yPos = y;
        zPos = z;
    }

    public void scale(double scaleFactor) {
        xPos *= scaleFactor;
        yPos *= scaleFactor;
        zPos *= scaleFactor;
    }

    public void addX(double x) {
        xPos += x;
    }

    public void addY(double y) {
        yPos += y;
    }

    public double get2dMagnitude() {
        return Math.hypot(xPos, yPos);
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public double getZ() {
        return zPos;
    }

    public void setX(double x) {
        xPos = x;
    }

    public void setY(double y) {
        yPos = y;
    }

    public void setZ(double z) {
        zPos = z;
    }

}
