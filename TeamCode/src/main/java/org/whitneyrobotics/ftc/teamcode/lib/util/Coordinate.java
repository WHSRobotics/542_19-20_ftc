package org.whitneyrobotics.ftc.teamcode.lib.util;

/**
 * Class for carrying coordinate values
 *
 * @see Position - Alternative class, without heading
 */
public class Coordinate extends Position {

    private double orientation;

    public Coordinate(double xPosition, double yPosition, double zPosition, double orientationInput) {
        super(xPosition, yPosition, zPosition);
        orientation = orientationInput;
    }

    public Coordinate(Position pos, double heading) {
        super(pos.getX(), pos.getY(), pos.getZ());
        orientation = heading;
    }

    public Coordinate returnCoord() {
        return this;
    }

    public double getHeading() {
        return orientation;
    }

    public Position getPos() {
        Position pos = new Position(xPos, yPos, zPos);
        return pos;
    }

    public void setHeading(double heading) {
        orientation = heading;
    }

    public void setPos(Position pos) {
        xPos = pos.getX();
        yPos = pos.getY();
        zPos = pos.getZ();
    }
}