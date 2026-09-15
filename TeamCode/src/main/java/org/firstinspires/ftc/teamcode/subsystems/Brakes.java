package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.hardware.LinkedServos;

public class Brakes extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedServos brakeServos;
    public double targetPosition;

    public Brakes(RobotContainer robotContainer, LinkedServos brakeServos) {
        this.robotContainer = robotContainer;
        this.targetPosition = Constants.Brakes.MIN;
        this.brakeServos = brakeServos;
    }

    public void update(boolean teleop) {
        if (teleop) {

            // Actions to happen when parkingBrake boolean was just changed
            if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
                Status.parkingBrake = !Status.parkingBrake;
                if (Status.parkingBrake) {
                    Status.turretFaceBack = true;
                    robotContainer.delayedActionManager.schedule(() -> targetPosition = Constants.Brakes.EXTENDED, 250);
                    robotContainer.delayedActionManager.schedule(() -> robotContainer.brakes.setPosition(targetPosition), 250);
                }

            // Actions to run in a loop
            } else if (Status.parking && !Status.parkingBrake) {
                targetPosition = Constants.Brakes.PARK;
                robotContainer.brakes.setPosition(Constants.Brakes.PARK);
            } else if (!Status.parkingBrake) {
                targetPosition = Constants.Brakes.MIN;
                robotContainer.brakes.setPosition(Constants.Brakes.MIN);
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        brakeServos.setPosition(HelperFunctions.clamp(position, Constants.Brakes.MIN, Constants.Brakes.MAX));
    }

    public double getPosition() {
        return brakeServos.getPosition();
    }

    public double getPositionDegrees() {
        return brakeServos.getPositionDegrees();
    }
}
