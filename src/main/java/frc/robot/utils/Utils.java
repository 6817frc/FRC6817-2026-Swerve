package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

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

    public static Pose2d redToAllianceSpecific(Pose2d redPose) {
        var alliance = DriverStation.getAlliance();

        if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Red) {
            // Default red
            return redPose;
        }

        return redPose.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.fromDegrees(180));
    }
}
