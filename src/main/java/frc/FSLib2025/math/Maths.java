package frc.FSLib2025.math;

public class Maths {
    
    public static boolean isWithin (double value, double min, double max) {
        return Math.max(min, value) == Math.min(value, max);
    }
    
    public static double clamp (double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public static int clamp (int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }

    public static double constrainAngleDegrees(double angle) {
        return Math.atan2(Math.sin(angle / 180.0 * Math.PI), Math.cos(angle / 180.0 * Math.PI)) * 180 / Math.PI;
    }

}
