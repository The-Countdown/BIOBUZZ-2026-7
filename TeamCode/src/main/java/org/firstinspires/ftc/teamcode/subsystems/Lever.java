package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class Lever extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;

    public Lever(RobotContainer robotContainer, BetterServo leverServo) {
        this.robotContainer = robotContainer;
    }

    public void update(boolean teleop) {
        if (teleop) {
            if (robotContainer.gamepadEx1.rightBumper.isHeld() || robotContainer.gamepadEx2.rightBumper.isHeld()) {
                robotContainer.lever.setPosition(Constants.Lever.OPEN);
                Status.leverOpen = true;
            } else {
                robotContainer.lever.setPosition(Constants.Lever.CLOSED);
                Status.leverOpen = false;
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        leverServo.updateSetPosition(HelperFunctions.clamp(position, Constants.Lever.OPEN, Constants.Lever.CLOSED));
    }

    public void open(){
        robotContainer.lever.setPosition(Constants.Lever.OPEN);
    }

    public void close() { robotContainer.lever.setPosition(Constants.Lever.CLOSED); }

}
