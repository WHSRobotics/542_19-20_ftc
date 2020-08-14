package org.whitneyrobotics.ftc.teamcode.lib.purepursuit.swervetotarget;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.SwerveWaypoint;
import org.whitneyrobotics.ftc.teamcode.lib.purepursuit.FollowerConstants;

import java.util.ArrayList;

public class SwervePath {
    private ArrayList<SwerveWaypoint> waypoints = new ArrayList<>();
    private FollowerConstants followerConstants;

    public SwervePath(ArrayList<SwerveWaypoint> waypoints, FollowerConstants followerConstants){
        this.waypoints = waypoints;
        this.followerConstants = followerConstants;
    }

    public ArrayList<SwerveWaypoint> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(ArrayList<SwerveWaypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public double getFollowerConstants() {
        return followerConstants.getLookaheadDistance();
    }

    public Position getPositionAtIndex(int index){
        return waypoints.get(index).getPosition();
    }

    public double getTargetVelocityAtIndex(int index){
        return waypoints.get(index).getTangentialVelocity();
    }

    public int size() {
        return waypoints.size();
    }

    public boolean backwards(){
        return followerConstants.backwards();
    }

}
