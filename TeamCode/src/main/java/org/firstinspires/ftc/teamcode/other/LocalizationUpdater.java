package org.firstinspires.ftc.teamcode.other;

import static org.firstinspires.ftc.teamcode.main.RobotContainer.HardwareDevices.pinpoint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class LocalizationUpdater extends Thread {
    private final RobotContainer robotContainer;
    public double CURRENT_LOOP_TIME_MS;
    public double CURRENT_LOOP_TIME_AVG_MS;
    public boolean enabled;
    public boolean teleop;

    /**
     * The pinpoint takes more time than a normal device, however, I don't know how much that is,
     * so I put it into a thread to be safe.
     */
    public LocalizationUpdater(RobotContainer robotContainer, boolean teleop) {
        this.robotContainer = robotContainer;
        this.CURRENT_LOOP_TIME_AVG_MS = 0;
        this.CURRENT_LOOP_TIME_MS = 0;
        this.enabled = true;
        this.teleop = teleop;
        // Set the thread to be a daemon thread so that it will not prevent the program from exiting.
        setDaemon(true);
        setName("PinpointUpdater");
    }

    @Override
    public void run() {
        if (!enabled) {
            return;
        }
        while (Status.opModeIsActive) {
            pinpoint.update();
//            Status.currentPose = new Pose2D(DistanceUnit.CM, pinpoint.getPosX(DistanceUnit.CM), pinpoint.getPosY(DistanceUnit.CM), AngleUnit.DEGREES, pinpoint.getHeading(AngleUnit.DEGREES));
            CURRENT_LOOP_TIME_MS = robotContainer.updateLoopTime("pinpointUpdater");
            CURRENT_LOOP_TIME_AVG_MS = robotContainer.getRollingAverageLoopTime("pinpointUpdater");
            if (Status.isDrivingActive) {
                try {
                    Thread.sleep(Constants.System.PINPOINT_UPDATE_DELAY_MS);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            } else {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        Thread.yield();
    }

    public void stopThread() {
        enabled = false;
    }
}
