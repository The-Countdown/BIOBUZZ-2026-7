package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.CalibrationData;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

// enum SensorMode {
//     IMU
// }

public class BetterIMU {
    // private SensorMode currentMode;
    private AdafruitBNO055IMU imu;
    private double angleOffset = 0;
    private CalibrationData calibration;

    public BetterIMU(AdafruitBNO055IMU imu) {
        this.imu = imu;
        this.angleOffset = 0;

        this.calibration = new CalibrationData();

        // super(deviceClient, true); // IDK to use true or false
        // setMode(SensorMode.IMU);
    }

    // Return the yaw in degrees
    public double getAngle() {
        return imu.getAngularOrientation().firstAngle - angleOffset + ((RobotContainer.HardwareDevices.pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES) / 360) * Constants.System.IMU_PER_ROTATION_OFFSET);
    }

    public void resetAngle() {
        this.angleOffset = imu.getAngularOrientation().firstAngle;
    }

    public void setAngle(double angle){
        this.angleOffset = imu.getAngularOrientation().firstAngle - angle;
    }

    public void setAngleOffset(double offsetAngle) {
        this.angleOffset = imu.getAngularOrientation().firstAngle - offsetAngle;
    }

    public void initialize(Parameters params) {
        imu.initialize(params);
    }

    public CalibrationData readCalibrationData() {
        return imu.readCalibrationData();
    }

    // public synchronized void setMode(SensorMode mode) {
    //     this.currentMode = mode;

    //     switch (mode) {
    //         case IMU:
    //             write8(BNO055IMU.Register.OPR_MODE, 0x07);
    //     }
    // }
}
