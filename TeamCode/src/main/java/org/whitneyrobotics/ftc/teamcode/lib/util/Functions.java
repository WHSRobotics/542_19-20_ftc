package org.whitneyrobotics.ftc.teamcode.lib.util;

/**
 * General purpose functions class
 */
public class Functions {
    public static double lastKnownRate = 0;
    public static double limitedRate;

    public static double calculateDistance(Position current, Position target) {
        double distance;
        distance = Math.sqrt(Math.pow(target.getX() - current.getX(), 2) +
                Math.pow(target.getY() - current.getY(), 2));
        return distance;
    }

    public static double[][] doubleArrayCopy(double[][] arr) {
        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for (int j = 0; j < arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }
        return temp;
    }

    public static double[][] posArrayTo2dDoubleArray(Position[] positions) {
        double[][] doublePositions = new double[positions.length][2];
        for (int i = 0; i < positions.length; i++) {
            doublePositions[i][0] = positions[i].getX();
            doublePositions[i][1] = positions[i].getY();
        }
        return doublePositions;
    }

    public static int calculateIndexOfSmallestValue(double[] array) {
        double smallest = array[0];
        int posInArray = 0;
        for (int i = 1; i < array.length; i++) {
            if (array[i] < smallest) {
                smallest = array[i];
                posInArray = i;
            }
        }
        return posInArray;
    }

    public static double distanceFormula(double x1, double y1, double x2, double y2) {
        double distance = Math.sqrt(Math.pow(Math.abs(x1 - x2), 2) + Math.pow(Math.abs(y2 - y1), 2));
        return distance;
    }

    /**
     * Converts angles from 0-360 to -180-180
     */
    public static double normalizeAngle(double angle) {

        if (angle > 180) {
            angle = angle - 360;
        } else if (angle < -180) {
            angle = angle + 360;
        }
        return angle;
    }

    public static Position transformCoordinates(double[][] dcm, Position vector) {
        Position transformedVector;

        double x = dcm[0][0] * vector.getX() + dcm[0][1] * vector.getY() + dcm[0][2] * vector.getZ();
        double y = dcm[1][0] * vector.getX() + dcm[1][1] * vector.getY() + dcm[1][2] * vector.getZ();
        double z = dcm[2][0] * vector.getX() + dcm[2][1] * vector.getY() + dcm[2][2] * vector.getZ();

        transformedVector = new Position(x, y, z);
        return transformedVector;
    }

    /**
     * limits how fast the input can change
     *
     * @param input                   the thing you want to limit
     * @param maxRateOfChange         the max speed at which it should change
     * @param time                    the time at which you are calling the method
     * @param lastRateLimiterCallTime the time at which you last called this method
     * @return Returns the new limited rate
     */
    public static double rateLimiter(double input, double maxRateOfChange, double time, double lastRateLimiterCallTime) {
        double maxChange = (time - lastRateLimiterCallTime) * maxRateOfChange;
        limitedRate += constrain(input - lastKnownRate, -maxChange, maxChange);
        lastKnownRate = limitedRate;
        return limitedRate;
    }

    public static double constrain(double input, double min, double max) {
        if (max >= input && input >= min) {
            return input;
        } else if (input > max) {
            return max;
        } else {
            return min;
        }
    }

    public static double cosd(double degree) {
        double rad = degree * Math.PI / 180;
        return Math.cos(rad);
    }

    public static double sind(double degree) {
        double rad = degree * Math.PI / 180;
        return Math.sin(rad);
    }

    public static double tand(double degree) {
        double rad = degree * Math.PI / 180;
        return Math.tan(rad);
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        if (x <= in_min) {
            return out_min;
        }
        if (x >= in_max) {
            return out_max;
        }
        if (in_min >= in_max) {
            throw new IllegalArgumentException("in_min greater than in_max");
        }
        if (out_min >= out_max) {
            throw new IllegalArgumentException("out_min greater than out_max");
        }
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static Position body2field(Position bodyVector, Coordinate currentCoord) {
        Position fieldVector;
        double heading = currentCoord.getHeading();

        double[][] C_b2f = {{Functions.cosd(heading), -Functions.sind(heading), 0},
                {Functions.sind(heading), Functions.cosd(heading), 0},
                {0, 0, 1}};

        fieldVector = Functions.transformCoordinates(C_b2f, bodyVector);
        return fieldVector;

    }

    public static Position field2body(Position fieldVector, Coordinate currentCoord) {
        Position bodyVector;
        double heading = currentCoord.getHeading();

        double[][] C_f2b = {{Functions.cosd(heading), Functions.sind(heading), 0},
                {-Functions.sind(heading), Functions.cosd(heading), 0},
                {0, 0, 1}};

        bodyVector = Functions.transformCoordinates(C_f2b, fieldVector);
        return bodyVector;

    }

    public static Position front2back(Position frontVector) {
        Position backVector;
        double heading = 180;

        double[][] C_f2b = {{-1, 0, 0},
                {0, -1, 0},
                {0, 0, 1}};

        backVector = Functions.transformCoordinates(C_f2b, frontVector);
        return backVector;
    }

    public static class Positions {
        public static Position add(Position pos1, Position pos2) {
            Position sum;

            double x = pos1.getX() + pos2.getX();
            double y = pos1.getY() + pos2.getY();
            double z = pos1.getZ() + pos2.getZ();

            sum = new Position(x, y, z);
            return sum;
        }

        public static Position subtract(Position pos1, Position pos2) {
            Position difference;

            double x = pos1.getX() - pos2.getX();
            double y = pos1.getY() - pos2.getY();
            double z = pos1.getZ() - pos2.getZ();

            difference = new Position(x, y, z);
            return difference;
        }

        public static double dot2d(Position pos1, Position pos2) {
            double dotProduct = pos1.getX() * pos2.getX() + pos1.getY() * pos2.getY();
            return dotProduct;
        }

        public static Position cross2d(Position pos1, Position pos2) {
            Position crossProduct2d;

            double x = 0.0;
            double y = 0.0;
            double z = pos1.getX() * pos2.getY() - pos1.getY() * pos2.getX();

            crossProduct2d = new Position(x, y, z);
            return crossProduct2d;
        }

        public static Position scale2d(double scaleFactor, Position pos) {
            Position scaledPos;

            double x = scaleFactor * pos.getX();
            double y = scaleFactor * pos.getY();
            double z = pos.getZ();

            scaledPos = new Position(x, y, z);
            return scaledPos;
        }
    }
}