package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class Door extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;

    public Door(RobotContainer robotContainer, BetterServo doorServo) {
        this.robotContainer = robotContainer;
    }

    public void update(boolean teleop) {
        if (teleop) {
            if (robotContainer.gamepadEx1.rightBumper.isHeld() || robotContainer.gamepadEx2.rightBumper.isHeld()) {
                robotContainer.door.setPosition(Constants.Door.OPEN);
                Status.doorOpen = true;
            } else {
                robotContainer.door.setPosition(Constants.Door.CLOSED);
                Status.doorOpen = false;
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        //TODO: Maybe switch these
        doorServo.updateSetPosition(HelperFunctions.clamp(position, Constants.Door.OPEN, Constants.Door.CLOSED));
    }

    public void open(){
        robotContainer.door.setPosition(Constants.Door.OPEN);
    }

    public void close() { robotContainer.door.setPosition(Constants.Door.CLOSED); }


//    public double getPosition() {
//        return doorServo.getPosition();
//    }
//
//    public double getPositionDegrees() {
//        return doorServo.getPositionDegrees();
//    }
}
