package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorTest", group = "TeleOp")
//@Disabled
public class MotorTest extends OpMode {
    private RobotContainer robotContainer;
    private int motorIndex = -1;
    private double motorPower = 0.5;


    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        robotContainer.start(this, true);
        RobotContainer.HardwareDevices.flywheelMotorLeader.start();
        RobotContainer.HardwareDevices.flywheelMotorFollower.start();
        RobotContainer.HardwareDevices.intakeMotorLeader.start();
        RobotContainer.HardwareDevices.intakeMotorFollower.start();
    }

    @Override
    public void loop() {
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();
        robotContainer.controlHubVoltage = robotContainer.getVoltage(Constants.Robot.CONTROL_HUB_INDEX);

        switch(motorIndex) {
            case -1:
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("stopped");
                break;
            case 0:
                RobotContainer.HardwareDevices.rightFront.setPower(motorPower);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("rightFront");
                break;
            case 1:
                RobotContainer.HardwareDevices.leftFront.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("leftFront");
                break;
            case 2:
                RobotContainer.HardwareDevices.rightBack.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("rightBack");
                break;
            case 3:
                RobotContainer.HardwareDevices.leftBack.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("leftBack");
                break;
            case 4:
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("intakeMotorLeader");
                break;
            case 5:
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("intakeMotorFollower");
                break;
            case 6:
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                telemetry.addLine("flyWheelMotorLeader");
                break;
            case 7:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                telemetry.addLine("flyWheelMotorFollower");
                break;
            case 8:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(motorPower);
                telemetry.addLine("Flywheel motors");
                break;
            case 9:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(motorPower);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
                telemetry.addLine("Intake motors");
                break;
            case 10:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.rightFront.setPower(0);
                RobotContainer.HardwareDevices.leftFront.setPower(0);
                RobotContainer.HardwareDevices.rightBack.setPower(0);
                RobotContainer.HardwareDevices.leftBack.setPower(0);
                RobotContainer.HardwareDevices.intakeMotorLeader.setPower(1);
                RobotContainer.HardwareDevices.intakeMotorFollower.setPower(1);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(motorPower);
                telemetry.addLine("Intake and Flywheel");
                break;
        }

        if (robotContainer.gamepadEx1.dpadRight.wasJustPressed()) {
            motorIndex++;
        } else if (robotContainer.gamepadEx1.dpadLeft.wasJustPressed()) {
            motorIndex--;
        }

        if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
            motorPower += 0.1;
        } else if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
            motorPower -= 0.1;
        }

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            motorPower = 0;
        }

        telemetry.addData("motorPower", motorPower);

        telemetry.update();
        Thread.yield();
    }
}