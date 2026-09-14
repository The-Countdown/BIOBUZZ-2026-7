package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;


public class PositionProvider {
    private RobotContainer robotContainer;
    private GoBildaPinpointDriver pinpoint;
    private Pose2D lastODPose;
    private Pose2D startODPose;

    public PositionProvider(RobotContainer robotContainer, GoBildaPinpointDriver pinpoint) {
        this.robotContainer = robotContainer;
        this.pinpoint = pinpoint;
        this.startODPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    }

    public Pose2D getRobotPose() {
        // Rotate the odPose by the vision offset
        if (Constants.System.USE_BETTER_IMU) {
            double rHeading;
            rHeading = Status.currentHeading;
            Pose2D odPose = new Pose2D(DistanceUnit.CM, pinpoint.getPosX(DistanceUnit.CM), pinpoint.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES));
            double rx = (odPose.getX(DistanceUnit.CM)) /* * 0.9914129 */;
            double ry = (odPose.getY(DistanceUnit.CM)) /* * 0.998610856289 */;
            double rh = Math.toRadians(rHeading - odPose.getHeading(AngleUnit.DEGREES)); // First angle is the yaw
            double newX = rx * Math.cos(-rh) - ry * Math.sin(-rh);
            double newY = ry * Math.cos(-rh) + rx * Math.sin(-rh);
            return new Pose2D(DistanceUnit.CM, newX, newY, AngleUnit.DEGREES, rHeading);
        } else {
            double rHeading;
            rHeading = RobotContainer.HardwareDevices.pinpoint.getHeading(AngleUnit.DEGREES);
            Pose2D odPose = new Pose2D(DistanceUnit.CM, pinpoint.getPosX(DistanceUnit.CM), pinpoint.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES));
            double rx = (odPose.getX(DistanceUnit.CM)) /* * 0.9914129 */;
            double ry = (odPose.getY(DistanceUnit.CM)) /* * 0.998610856289 */;
            double rh = Math.toRadians(rHeading - odPose.getHeading(AngleUnit.DEGREES)); // First angle is the yaw
            double newX = rx * Math.cos(-rh) - ry * Math.sin(-rh);
            double newY = ry * Math.cos(-rh) + rx * Math.sin(-rh);
            return new Pose2D(DistanceUnit.CM, rx, ry, AngleUnit.DEGREES, rHeading);
        }
    }

    public void update() {
            Pose2D odPose = RobotContainer.HardwareDevices.pinpoint.getPosition();
            Status.currentPose = getRobotPose();
            lastODPose = odPose;
        }
    }
