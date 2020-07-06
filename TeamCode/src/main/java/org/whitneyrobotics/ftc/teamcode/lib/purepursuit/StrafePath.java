package org.whitneyrobotics.ftc.teamcode.lib.purepursuit;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.StrafeWaypoint;

import java.util.ArrayList;

public class StrafePath {

    private ArrayList<StrafeWaypoint> waypoints = new ArrayList<>();

    public StrafePath(ArrayList<StrafeWaypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public ArrayList<StrafeWaypoint> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(ArrayList<StrafeWaypoint> waypoints) {
        this.waypoints = waypoints;
    }


}
