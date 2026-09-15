package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;

public class HelperFunctions {

    public static double mirror(double xVal) {
        return Math.abs(xVal - 72) + xVal;
    }

    /**
     * @param pose Pose3d to convert to Pose2d
     * @return Pose2d
     */
    public static Pose2D to2D(Pose3D pose) {
        return new Pose2D(DistanceUnit.CM, pose.getPosition().x * 100, pose.getPosition().y * 100, AngleUnit.DEGREES,pose.getOrientation().getYaw(AngleUnit.DEGREES));
    }

    public static Pose mirror(Pose point) {
        return new Pose(Math.abs(point.x() - 144), point.y(), Math.PI - point.heading());
    }

    public static double mirrorAngle90(double angle, boolean degrees){
        return degrees ? 180 - angle : Math.PI - angle;
    }

    public static double mirrorAngle0(double angle){
        return -angle;
    }

    public static Pose2D mirror90(Pose2D pos, DistanceUnit distUnit, AngleUnit angUnit) {
        return angUnit == AngleUnit.DEGREES ? new Pose2D (distUnit, -pos.getX(distUnit), pos.getY(distUnit), angUnit, 180 - pos.getHeading(AngleUnit.DEGREES)) : new Pose2D (distUnit, -pos.getX(distUnit), pos.getY(distUnit), angUnit, Math.PI - pos.getHeading(AngleUnit.RADIANS));
    }
    public static Pose2D mirror0(Pose2D pos, DistanceUnit distUnit, AngleUnit angUnit) {
        return angUnit == AngleUnit.DEGREES ? new Pose2D (distUnit, -pos.getX(distUnit), pos.getY(distUnit), angUnit, 180 - pos.getHeading(AngleUnit.DEGREES)) : new Pose2D (distUnit, -pos.getX(distUnit), pos.getY(distUnit), angUnit, Math.PI - pos.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Normalizes an angle to the range [-180, 180).
     * <p>
     * (This is duplicated from {@link Drivetrain} because I wanted the constants class to be
     * isolated from the rest of the codebase)
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    public static double normalizeAngle(double angle) {
        // Check if the angle is already in the desired range.
        if (angle >= -180 && angle < 180) {
            return angle;
        }

        // Normalize the angle to the range [-360, 360).
        double normalizedAngle = angle % 360;

        // If the result was negative, shift it to the range [0, 360).
        if (normalizedAngle < 0) {
            normalizedAngle += 360;
        }

        // If the angle is in the range [180, 360), shift it to [-180, 0).
        if (normalizedAngle >= 180) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }

    public static double averageAngles(double a, double b) {
        double x = Math.cos(Math.toRadians(a)) + Math.cos(Math.toRadians(b));
        double y = Math.sin(Math.toRadians(a)) + Math.sin(Math.toRadians(b));
        return Math.toDegrees(Math.atan2(y, x));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double disToGoal() {
        if (Status.alliance == Constants.Game.ALLIANCE.BLUE){
            double xDiff = 72 - Status.currentPose.getX(DistanceUnit.INCH);
            double yDiff = 72 - Status.currentPose.getY(DistanceUnit.INCH);
            return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        } else {
            double xDiff = 72 - Status.currentPose.getX(DistanceUnit.INCH);
            double yDiff = -72 - Status.currentPose.getY(DistanceUnit.INCH);
            return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
            }
    }

    // Assuming point 1 is less than point 2
    public static double interpolate(double point1, double point2, double percentageSplit) {
        return point1 + ((point2 - point1) * percentageSplit);
    }

    public static int getRandomWithin(int num, double percent) {
        double adjustment = num * percent;
        return (int) (num + Math.random() > percent ? adjustment : -adjustment);
    }
}
