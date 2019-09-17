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

    public static double[][] positionArrayToDoubleArray(Position[] positions) {
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
        /*
        else {
            angle=angle;
        }
        */
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

    public static Position addPositions(Position pos1, Position pos2) {
        Position sum;

        double x = pos1.getX() + pos2.getX();
        double y = pos1.getY() + pos2.getY();
        double z = pos1.getZ() + pos2.getZ();

        sum = new Position(x, y, z);
        return sum;
    }

    public static Position subtractPositions(Position pos1, Position pos2) {
        Position difference;

        double x = pos1.getX() - pos2.getX();
        double y = pos1.getY() - pos2.getY();
        double z = pos1.getZ() - pos2.getZ();

        difference = new Position(x, y, z);
        return difference;
    }

    public static double calculateMagnitude(Position pos) {
        double magnitude = Math.pow(pos.getX(), 2) + Math.pow(pos.getY(), 2);
        magnitude = Math.sqrt(magnitude);
        return magnitude;
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


    public static class Vectors {
        public static double[] add(double[] vector1, double[] vector2) {
            if (vector1.length != vector2.length) {
                throw new IllegalArgumentException("input vector lengths not equal");
            }
            double[] sumVector = new double[vector1.length];
            for (int i = 0; i < vector1.length; i++) {
                sumVector[i] = vector1[i] + vector2[i];
            }
            return sumVector;
        }

        public static double[] subtract(double[] vector1, double[] vector2) {
            if (vector1.length != vector2.length) {
                throw new IllegalArgumentException("input vector lengths not equal");
            }
            double[] differenceVector = new double[vector1.length];
            for (int i = 0; i < vector1.length; i++) {
                differenceVector[i] = vector1[i] - vector2[i];
            }
            return differenceVector;
        }

        public static double dot(double[] vector1, double[] vector2) {
            if (vector1.length != vector2.length) {
                throw new IllegalArgumentException("input vector lengths not equal");
            }
            double dotProduct = 0;
            for (int i = 0; i < vector1.length; i++) {
                dotProduct += vector1[i] * vector2[i];
            }
            return dotProduct;
        }

        public static double cross2D(double[] vector1, double[] vector2) {
            return vector1[0] * vector2[1] - vector1[1] * vector2[0];
        }

        public static double[] scale(double scalar, double[] vector) {
            double[] scaledVector = new double[vector.length];
            for (int i = 0; i < vector.length; i++) {
                scaledVector[i] = scalar * vector[i];
            }
            return scaledVector;
        }

        public static double magnitude(double[] vector) {
            if (vector.length != 2) {
                throw new IllegalArgumentException("input vector lengths not equal");
            }
            return Math.sqrt(Math.pow(vector[0], 2) + Math.pow(vector[1], 2));
        }
    }
}