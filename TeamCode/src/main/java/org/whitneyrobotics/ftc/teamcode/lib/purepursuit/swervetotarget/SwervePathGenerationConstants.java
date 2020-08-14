package org.whitneyrobotics.ftc.teamcode.lib.purepursuit.swervetotarget;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SwervePathGenerationConstants {

    // Path Generation
    private double spacing, weightSmooth, turnSpeed, pathMaxVelocity;

    public SwervePathGenerationConstants(double spacing, double weightSmooth, double turnSpeed, double pathMaxVelocity) {
        this.spacing = spacing;
        this.weightSmooth = weightSmooth;
        this.turnSpeed = turnSpeed;
        this.pathMaxVelocity = pathMaxVelocity;
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
