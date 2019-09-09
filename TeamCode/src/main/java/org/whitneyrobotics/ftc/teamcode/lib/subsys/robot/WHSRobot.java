package org.whitneyrobotics.ftc.teamcode.lib.subsys.robot;


import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;

/**
 * WHSRobot interface, containing methods for autonomous navigation.
 */

public interface WHSRobot {

    /**
     * The main method for autonomous navigation
     * <p>
     *     driveToTarget provides a easy way of achieving coordinate based navigation, that is
     *     navigation based on absolute field coordinates rather then set distances and angles.
     *     This method provides numerous advantages over the latter, including being able to easily
     *     change individual waypoints without affecting others.
     * </p>
     *
     * <p>driveToTarget can be used in a switch statement like structure in a loop as follows:
     *     <pre>
     *          robot.driveToTarget(new Position(x, y , 150), true/false);
                if(!robot.driveToTargetInProgress() || !robot.rotateToTargetInProgress()){
                    // advance to the next state
                }
     *     </pre>
     * </p>
     *
     * @param targetPos The {@link Position} to navigate to (in absolute field-frame coordinates)
     * @param backwards The option of enabling backwards navigation. If enabled, then for drives
     *                  which require a turn greater than 90 deg, the robot will act as though the
     *                  front of the robot is the back (ie it will drive backwards).
     *
     * @see #driveToTargetInProgress()
     * @see #rotateToTarget(double, boolean)
     * @see #estimatePosition()
     * @see #estimateHeading()
     */
    public void driveToTarget(Position targetPos, boolean backwards);

    /**
     * Rotates to a specified heading.
     *
     * @param targetHeading Field-frame absolute target heading, in degrees.
     * @param backwards The option of backwards turning. For turns of greater than 90 degrees,
     *                  rather then facing the front of the robot towards the targetHeading, it
     *                  faces the back of the robot.
     *
     * @see #rotateToTargetInProgress()
     * @see #driveToTarget(Position, boolean)
     * @see #estimateHeading()
     */
    public void rotateToTarget(double targetHeading, boolean backwards); //-180 to 180 degrees

    /**
     * Status boolean for {@link #driveToTarget} , used in its control.
     *
     * @return True if driveToTarget is in progress, false if not in progress
     *
     * @see #driveToTarget(Position, boolean)
     */
    public boolean driveToTargetInProgress();

    /**
     * Status boolean for {@link #rotateToTarget}, used in its control.
     *
     * @return True if rotateToTarget is in progress, false if not in progress
     *
     * @see #rotateToTarget(double, boolean)
     */
    public boolean rotateToTargetInProgress();

    /**
     * Position estimation method.
     * <p>estimatePosition is a method for estimating the robot's position by the use of the
     * drivetrain encoders and IMU. By using the encoders (to measure how far the robot has
     * traveled) and the IMU (to measure in which direction the robot has traveled) in conjunction
     * with trigonometry it is possible to figure out the robot's absolute x and y position on the
     * field. </p>
     * <p>The method then updates the internal {@link Coordinate} object in the WHSRobot.</p>
     * <p>Note: the z value is always set to 150.</p>
     * <p>This method should be called every loop, along with {@link #estimateHeading()}.</p>
     *
     * @see #estimateHeading()
     * @see #getCoordinate()
     */
    public void estimatePosition();

    /**
     * Heading estimation method.
     * <p>estimateHeading is a method for estimating the robot's heading by the use of the
     * IMU. As of right now, this is just the IMU value + IMU bias (set during
     * {@link #setInitialCoordinate(Coordinate)} run through
     * {@link lib.util.Functions#normalizeAngle(double)}.</p>
     * <p>The method then updates the internal {@link Coordinate} object in the WHSRobot.</p>
     * <p>This method should be called every loop cycle, (probably) along with {@link #estimatePosition()}.</p>
     *
     * @see #estimatePosition()
     * @see #getCoordinate()
     */
    public void estimateHeading();

    /**
     * Sets the inital coordinate of the robot.  Further navigation will be relative to this initial coordinate.
     *
     * @param initCoord An {@link Coordinate} object to set as the inital coordinate.
     */
    public void setInitialCoordinate(Coordinate initCoord);

    /**
     * Hard sets the robot's coordinate.
     *
     * @param coord The {@link Coordinate} to set the robot to.
     */
    public void setCoordinate(Coordinate coord);

    /**
     * Returns what the robot's current coordinate is.
     *
     * @return The current {@link Coordinate} of the robot.
     *
     * @see #estimatePosition()
     */
    public Coordinate getCoordinate();


}
