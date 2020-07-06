package org.whitneyrobotics.ftc.teamcode.lib.purepursuit;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Waypoint;

import java.util.ArrayList;

public class Path {

    private ArrayList<Waypoint> waypoints = new ArrayList<>();

    public Path(ArrayList<Position> positions){
    }

    public ArrayList<Waypoint> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(ArrayList<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }


}
