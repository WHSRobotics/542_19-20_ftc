package org.whitneyrobotics.ftc.teamcode.lib.util;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SwerveConstants {

    public static double MAX_ACCELERATION = 2000.0; // mm/s^2

    public static class StartToFoundationSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class FoundationToWallSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class WallToSkystoneSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class StartToSkystoneSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class SkystoneToMovedFoundationSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class SkystoneToUnmovedFoundationSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class MovedFoundationToParkingSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }

    public static class WallToParkingSwerveConstants {
        public static double kP = 0.001;
        public static double kV = 0;
        public static double kA = 0;

        public static double lookaheadDistance = 220;
        public static double velocityConstant = 2.5;
    }





}
