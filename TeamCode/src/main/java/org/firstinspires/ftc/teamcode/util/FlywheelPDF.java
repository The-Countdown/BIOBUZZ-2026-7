package org.firstinspires.ftc.teamcode.util;

import androidx.appcompat.widget.ThemedSpinnerAdapter;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.hardware.LinkedMotors;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class FlywheelPDF {
    private final RobotContainer robotContainer;
    private LinkedMotors flywheelMotors;

    private double error;
    private double lastError;
    private double p;
    private double i;
    private double s;
    private double ff;
    private double d;
    private double pid;
    private double dt;
    private double errorDiff;
    private double lastTime;
    private double lastTargetPower;
    private double dropTime;
    public TelemetryManager panelsTelemetry;

    public FlywheelPDF(RobotContainer robotContainer, LinkedMotors flywheelMotors) {
        this.robotContainer = robotContainer;
        this.flywheelMotors = flywheelMotors;
        this.lastTargetPower = 0;
        this.lastTime = 0;
        this.dropTime = 0;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double targetSpeed) {
        if (targetSpeed == 0) {
            lastTargetPower = 0;
            return 0;
        }
        double error = targetSpeed - flywheelMotors.getVelocity();
        dt = System.currentTimeMillis() - lastTime;
        errorDiff = (lastError - error) / dt; // differance between the errors scaled by time

        if (errorDiff < -4 & System.currentTimeMillis() - dropTime > 250 ){
            dropTime = System.currentTimeMillis();
        }

//            s = Math.signum(error)* Math.pow(error, 2) * Constants.Turret.FLYWHEEL_KS; // This is tuned just like p

            p = Constants.Turret.FLYWHEEL_KP * error; // proportional to error

            i += error * Constants.Turret.FLYWHEEL_KI;

            d = Constants.Turret.FLYWHEEL_KD  * -errorDiff; // this slows the acceleration down

            ff = (targetSpeed * Constants.Turret.FLYWHEEL_KF) / Constants.Turret.FLYWHEEL_MAX_VELOCITY; // flat addition to the power


        pid = (p + i + d) * dt + ff;

        lastTargetPower = pid;

        lastTargetPower = HelperFunctions.clamp(lastTargetPower, 0, 1);

//        if (!Status.competitionMode){
            panelsTelemetry.addData("Power", lastTargetPower);
            panelsTelemetry.addData("P", p);
            panelsTelemetry.addData("I", i);
            panelsTelemetry.addData("D", d);
    //        panelsTelemetry.addData("S", s);
            panelsTelemetry.addData("FF", ff);
            panelsTelemetry.addData("PID", pid);
            panelsTelemetry.addData("DT", dt);
            panelsTelemetry.addData("Drop Time", dropTime);
            panelsTelemetry.addData("Target", targetSpeed);
            panelsTelemetry.addData("Velocity", flywheelMotors.getVelocity());
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("error Diff", errorDiff);
//        }

        lastError = error;
        lastTime += dt;
        return lastTargetPower;
    }
}
