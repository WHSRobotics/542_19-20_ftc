package org.whitneyrobotics.ftc.teamcode.lib.purepursuit;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveConstants {

    // Path Generation
    private double spacing, weightSmooth, turnSpeed, pathMaxVelocity, maxAcceleration;
    // Follower
    private double lookaheadDistance;
    // Based on robot physical properties
    private double kV, kA;

    public SwerveConstants(double spacing, double weightSmooth, double turnSpeed, double pathMaxVelocity, double maxAcceleration, double lookaheadDistance, double kV, double kA) {
        this.spacing = spacing;
        this.weightSmooth = weightSmooth;
        this.turnSpeed = turnSpeed;
        this.pathMaxVelocity = pathMaxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.lookaheadDistance = lookaheadDistance;
        this.kV = kV;
        this.kA = kA;
    }


    public double getSpacing() {
        return spacing;
    }

    public double getWeightSmooth() {
        return weightSmooth;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }

    public double getPathMaxVelocity() {
        return pathMaxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getLookaheadDistance() {
        return lookaheadDistance;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public void setSpacing(double spacing) {
        this.spacing = spacing;
    }

    public void setWeightSmooth(double weightSmooth) {
        this.weightSmooth = weightSmooth;
    }

    public void setTurnSpeed(double turnSpeed) {
        this.turnSpeed = turnSpeed;
    }

    public void setPathMaxVelocity(double pathMaxVelocity) {
        this.pathMaxVelocity = pathMaxVelocity;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }
    /*//real constants
    public static double MAX_ACCELERATION = 800.0; // mm/s^2
    public static double MAX_VELOCITY = 1400.0;
    public static double kP = 0.0001;
    public static double kV = 0.0006338;
    public static double kA = 0.0012;

    public static double lookaheadDistance = 350;//220;
    public static double velocityConstant = 2.0;

    public static double MAX_ANGULAR_ACCELERATION = 5.2;
    public static double hKP = 0.0005;
    public static double hKI = 0.0;
    public static double hKD = 0.0;

    public static class StartToFoundationSwerveConstants {
        public static double kP = SwerveConstants.kP/2.54;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class FoundationToWallSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class WallToSkystoneSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class StartToSkystoneSwerveConstants {
        public static double kP = SwerveConstants.kP/2;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant*1.1;
    }

    public static class SkystoneToMovedFoundationSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class SkystoneToUnmovedFoundationSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance + 50;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class MovedFoundationToParkingSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance + 200;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class WallToParkingSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }
    public static class secondSkystoneToMovedFoundationSwerveConstants{
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }



*/


}
