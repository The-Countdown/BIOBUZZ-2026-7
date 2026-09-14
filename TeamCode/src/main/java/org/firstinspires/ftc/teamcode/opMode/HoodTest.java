package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "HoodTest", group = "TeleOp")
//@Disabled
public class HoodTest extends OpMode {
    private RobotContainer robotContainer;
    private int currentPosIndex = 0;
    private double[] hoodPresets = {0.9, 0.9, 0.9};

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
    }

    @Override
    public void loop() {
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();
        robotContainer.controlHubVoltage = robotContainer.getVoltage(Constants.Robot.CONTROL_HUB_INDEX);

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            currentPosIndex = 0;
        } else if (robotContainer.gamepadEx1.circle.wasJustPressed()) {
            currentPosIndex = 1;
        } else if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
            currentPosIndex = 2;
        }

        if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
            hoodPresets[currentPosIndex] += 0.01;
        } else if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
            hoodPresets[currentPosIndex] -= 0.01;
        }

        robotContainer.turret.hood.setPos(hoodPresets[currentPosIndex]);

        if (currentPosIndex == 0) {
            telemetry.addLine("Current position: DOWN");
        } else if (currentPosIndex == 1) {
            telemetry.addLine("Current position: MIDDLE");
        } else if (currentPosIndex == 2) {
            telemetry.addLine("Current position: UP");
        }
        telemetry.addData("Current position value", hoodPresets[currentPosIndex]);

        telemetry.update();
        Thread.yield();
    }
}