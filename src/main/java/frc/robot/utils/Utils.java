package frc.robot.utils;

public class Utils {
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double convertTo180(double angle) {
        angle = angle % 360;
    
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
    
        return angle;
    }
}
