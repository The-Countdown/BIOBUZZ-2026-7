package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class Drivetrain {
    private final RobotContainer robotContainer;
    private final BetterDcMotor leftFront;
    private final BetterDcMotor rightFront;
    private final BetterDcMotor leftBack;
    private final BetterDcMotor rightBack;
    private double lastTime;
    private double[] outputs = {0,0};
    private double[] targets = {0,0};
    private double[] lastOutputs = {0,0};


    public Drivetrain(RobotContainer robotContainer, BetterDcMotor leftFront, BetterDcMotor rightFront, BetterDcMotor leftBack, BetterDcMotor rightBack) {
        this.robotContainer = robotContainer;
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;

        leftFront.start();
        rightFront.start();
        leftBack.start();
        rightBack.start();
    }

    public void update(){
        if (robotContainer.gamepadEx1.circle.wasJustPressed()) {
            RobotContainer.HardwareDevices.pinpoint.setPosition(Constants.Game.ORIGIN);
            RobotContainer.HardwareDevices.betterIMU.resetAngle();
        } else if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
             Pose2D resetPose = Status.cornerResetPose;
            RobotContainer.HardwareDevices.pinpoint.setPosition(resetPose);
            RobotContainer.HardwareDevices.betterIMU.setAngle(resetPose.getHeading(AngleUnit.DEGREES));
        }

    }

    public void driveJoystickUpdate() {

        if (Status.parkingBrake){return;}

        double current = getCurrent();
        double currentMultiplier = current > 15 ? 14/current : 1;
        double y = joystickScaler(robotContainer.gamepadEx1.leftStickY()) * currentMultiplier;
        double x = joystickScaler(robotContainer.gamepadEx1.leftStickX()) * currentMultiplier;

        double currentTime = System.currentTimeMillis()/1000.0;
        double dt = currentTime - lastTime;
        double maxAcceleration = dt * Constants.Control.MAX_DRIVE_ACCELERATION;

        for (int i = 0; i < 2; i++){
            targets[i] = i == 0 ? y : x;
            double powerError = targets[i] - lastOutputs[i];
            if (Math.signum(powerError) == Math.signum(targets[i]) && Math.signum(targets[i]) != 0) {
                double delta = Math.copySign(Math.min(Math.abs(powerError), maxAcceleration), powerError);
                outputs[i] += delta;
            } else {
                outputs[i] = targets[i];
            }
        }

        y = outputs[0];
        x = outputs[1];

        lastTime = currentTime;
        lastOutputs = outputs;

        double rx = joystickRotationScaler(robotContainer.gamepadEx1.rightStickX()) * currentMultiplier;

        double denominator;
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        if (robotContainer.gamepadEx1.cross.wasJustPressed()){
            Status.fieldOriented = !Status.fieldOriented;
        }

        if (y == 0 && x == 0 && rx == 0) {
            Status.parking = true;
        } else {
            Status.parking = false;
        }

        if (Status.fieldOriented) {
            double rotX;
            double rotY;
            if (Status.alliance == Constants.Game.ALLIANCE.RED) {
                rotX = x * Math.cos(-Math.toRadians(Status.currentHeading + 90)) - y * Math.sin(-Math.toRadians(Status.currentHeading + 90));
                rotY = x * Math.sin(-Math.toRadians(Status.currentHeading + 90)) + y * Math.cos(-Math.toRadians(Status.currentHeading + 90));
            } else {
                rotX = x * Math.cos(-Math.toRadians(Status.currentHeading - 90)) - y * Math.sin(-Math.toRadians(Status.currentHeading - 90));
                rotY = x * Math.sin(-Math.toRadians(Status.currentHeading - 90)) + y * Math.cos(-Math.toRadians(Status.currentHeading - 90));
            }

            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontPower = (rotY + rotX + rx) / denominator;
            rightFrontPower = (rotY - rotX - rx) / denominator;
            leftBackPower = (rotY - rotX + rx) / denominator;
            rightBackPower = (rotY + rotX - rx) / denominator;
        } else {
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFrontPower = (y + x + rx) / denominator;
            rightFrontPower = (y - x - rx) / denominator;
            leftBackPower = (y - x + rx) / denominator;
            rightBackPower = (y + x - rx) / denominator;
        }

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

        if (!Status.competitionMode){
            robotContainer.panelsTelemetry.addData("Y Targets", targets[0]);
            robotContainer.panelsTelemetry.addData("Y Output", outputs[0]);
            robotContainer.panelsTelemetry.addData("X Target", targets[1]);
            robotContainer.panelsTelemetry.addData("X Output", outputs[1]);
        }
    }

    /**
     * This function scales the power of the joystick to follow a curve, so that it allows for finer adjustments.
     * It allows for changes to the curve with the constant JOYSTICK_SCALER_EXPONENT.
     * @param input the input from the joystick
     * @return the scaled power
     */
    public double joystickScaler(double input) {
        return Math.pow(Math.abs(input), Constants.Control.JOYSTICK_SCALER_EXPONENT) * input;
    }

    public double joystickRotationScaler(double input) {
        return Math.pow(Math.abs(input), Constants.Control.JOYSTICK_ROTATION_SCALER_EXPONENT) * input;
    }

    public double getPower(){
        return leftBack.getPower() + leftFront.getPower() + leftBack.getPower() + rightBack.getPower();
    }

    public double getCurrent(){
        return leftBack.getCurrent(CurrentUnit.AMPS) + leftFront.getCurrent(CurrentUnit.AMPS) + rightFront.getCurrent(CurrentUnit.AMPS) + rightBack.getCurrent(CurrentUnit.AMPS);
    }
}
