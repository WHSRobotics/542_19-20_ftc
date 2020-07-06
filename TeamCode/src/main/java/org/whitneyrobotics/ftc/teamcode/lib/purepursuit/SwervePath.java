package org.whitneyrobotics.ftc.teamcode.lib.purepursuit;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.SwerveWaypoint;

import java.util.ArrayList;

public class SwervePath {

    private ArrayList<SwerveWaypoint> waypoints = new ArrayList<>();

    public SwervePath(ArrayList<SwerveWaypoint> waypoints){
        this.waypoints = waypoints;
    }

    public ArrayList<SwerveWaypoint> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(ArrayList<SwerveWaypoint> waypoints) {
        this.waypoints = waypoints;
    }


}
