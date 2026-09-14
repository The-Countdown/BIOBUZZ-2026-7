package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.hardware.LinkedMotors;
import org.firstinspires.ftc.teamcode.main.Status;

public class Intake {
    private RobotContainer robotContainer;
    private LinkedMotors intakeMotors;
    public double power = 0;
    public double currentAmps = 0;
    public double newPower;

    public Intake(RobotContainer robotContainer, LinkedMotors intakeMotors) {
        this.robotContainer = robotContainer;
        this.intakeMotors = intakeMotors;
        this.newPower = 0;
    }

    public void update(boolean teleop) {
        if (teleop) {
            if (Math.abs(robotContainer.turret.flywheelError) < Constants.Intake.FLYWHEEL_TOLERANCE || !Status.doorOpen) {
                if (Status.doorOpen) {
                    setPower((robotContainer.gamepadEx1.rightTriggerRaw() + robotContainer.gamepadEx2.rightTriggerRaw()) - (robotContainer.gamepadEx1.leftTriggerRaw() + robotContainer.gamepadEx2.leftTriggerRaw()));
                } else {
                    setPower(Math.min((robotContainer.gamepadEx1.rightTriggerRaw() + robotContainer.gamepadEx2.rightTriggerRaw()) - (robotContainer.gamepadEx1.leftTriggerRaw() + robotContainer.gamepadEx2.leftTriggerRaw()), 0.75));
                }
            } else {
                setPower(0);
            }
        } else {
            if (Status.doorOpen){
                setPower(power);
            } else {
                setPower(Math.min(power, 0.75));
            }
        }
    }

    public void setPower(double power) {
        this.power = power;
        double current = getCurrent();
        if (current > 5 && !Status.doorOpen) {
            this.newPower = Math.max(power - (power * ((current - 5) * 0.1)), 0);
            intakeMotors.setPower(newPower);
        } else {
            intakeMotors.setPower(power);
        }
    }

    public double getVelocity() {
        return intakeMotors.getVelocity();
    }

    public double getPower() {
        return intakeMotors.getPower();
    }

    public double getCurrent() {
        return intakeMotors.getCurrent(CurrentUnit.AMPS);
    }

    public double getNewPower(){
        return newPower;
    }
}
