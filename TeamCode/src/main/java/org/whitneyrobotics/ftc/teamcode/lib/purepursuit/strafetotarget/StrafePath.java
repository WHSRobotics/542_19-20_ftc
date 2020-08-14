package org.whitneyrobotics.ftc.teamcode.lib.purepursuit.strafetotarget;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.StrafeWaypoint;
import org.whitneyrobotics.ftc.teamcode.lib.purepursuit.FollowerConstants;

import java.util.ArrayList;

public class StrafePath {

    private ArrayList<StrafeWaypoint> waypoints = new ArrayList<>();

    FollowerConstants lookaheadDistance;

    public StrafePath(ArrayList<StrafeWaypoint> waypoints, FollowerConstants lookaheadDistance) {
        this.waypoints = waypoints;
        this.lookaheadDistance = lookaheadDistance;
    }

    public ArrayList<StrafeWaypoint> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(ArrayList<StrafeWaypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public double getLookaheadDistance() {
        return lookaheadDistance.getLookaheadDistance();
    }

}
