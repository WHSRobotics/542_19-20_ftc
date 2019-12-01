package org.whitneyrobotics.ftc.teamcode.lib.util;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveConstants {

    public static double MAX_ACCELERATION = 1500.0; // mm/s^2
    public static double MAX_VELOCITY = 1400.0;
    public static double kP = 0.0;
    public static double kV = 0.0006338;
    public static double kA = 0.0012;
    public static double lookaheadDistance = 220;
    public static double velocityConstant = 2.5;

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
        public static double kP = SwerveConstants.kP/2.0;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class MovedFoundationToParkingSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }

    public static class WallToParkingSwerveConstants {
        public static double kP = SwerveConstants.kP;
        public static double kV = SwerveConstants.kV;
        public static double kA = SwerveConstants.kA;

        public static double lookaheadDistance = SwerveConstants.lookaheadDistance;
        public static double velocityConstant = SwerveConstants.velocityConstant;
    }





}
