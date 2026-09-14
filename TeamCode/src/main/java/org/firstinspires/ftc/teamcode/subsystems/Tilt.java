package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.hardware.LinkedServos;

public class Tilt extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedServos tiltServos;
    public double targetPosition;

    public Tilt(RobotContainer robotContainer, LinkedServos tiltServos) {
        this.robotContainer = robotContainer;
        this.targetPosition = Constants.Tilt.MIN;
        this.tiltServos = tiltServos;
    }

    public void update(boolean teleop) {
        if (teleop) {

            // Actions to happen when robotTilted boolean was just changed
            if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
                Status.robotTilted = !Status.robotTilted;
                if (Status.robotTilted) {
                    Status.turretFaceBack = true;
                    robotContainer.delayedActionManager.schedule(() -> targetPosition = Constants.Tilt.EXTENDED, 250);
                    robotContainer.delayedActionManager.schedule(() -> robotContainer.tilt.setPosition(targetPosition), 250);
                }

            // Actions to run in a loop
            } else if (Status.parking && !Status.robotTilted) {
                Status.turretFaceBack = false;
                targetPosition = Constants.Tilt.PARK;
                robotContainer.tilt.setPosition(Constants.Tilt.PARK);
            } else if (!Status.robotTilted) {
                Status.turretFaceBack = false;
                robotContainer.turret.setFlywheelPowerModifier(0);
                targetPosition = Constants.Tilt.MIN;
                robotContainer.tilt.setPosition(Constants.Tilt.MIN);
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        tiltServos.setPosition(HelperFunctions.clamp(position, Constants.Tilt.MIN, Constants.Tilt.MAX));
    }

    public double getPosition() {
        return tiltServos.getPosition();
    }

    public double getPositionDegrees() {
        return tiltServos.getPositionDegrees();
    }
}
