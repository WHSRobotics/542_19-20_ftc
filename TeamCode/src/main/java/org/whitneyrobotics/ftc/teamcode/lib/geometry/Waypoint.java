package org.whitneyrobotics.ftc.teamcode.lib.geometry;

public class Waypoint {

    Position position;
    double angularVelocity;
    double tangentialVelocity;

    public Waypoint(Position position, double angularVelocity, double tangentialVelocity){
        this.position = position;
        this.angularVelocity = angularVelocity;
        this.tangentialVelocity = tangentialVelocity;
    }


    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    public double getTangentialVelocity() {
        return tangentialVelocity;
    }

    public void setTangentialVelocity(double tangentialVelocity) {
        this.tangentialVelocity = tangentialVelocity;
    }


}
