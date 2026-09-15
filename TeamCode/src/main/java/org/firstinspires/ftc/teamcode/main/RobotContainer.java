package org.firstinspires.ftc.teamcode.main;

import android.os.DropBoxManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.hardware.BetterIMU;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.other.PositionProvider;
import org.firstinspires.ftc.teamcode.subsystems.Lever;
import org.firstinspires.ftc.teamcode.subsystems.IndicatorLighting;
import org.firstinspires.ftc.teamcode.other.LocalizationUpdater;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Brakes;
import org.firstinspires.ftc.teamcode.util.DelayedActionManager;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.hardware.LinkedMotors;
import org.firstinspires.ftc.teamcode.hardware.LinkedServos;
import org.firstinspires.ftc.teamcode.util.TelemetryLogger;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * The Robot class serves as the central manager for all robot hardware and high-level operations.
 * It initializes and provides access to key components such as the IMU, Limelight, swerve drive modules,
 * and associated control mechanisms. This class also manages the interaction with the FTC SDK, handles
 * asynchronous tasks through a Handler, and provides utility methods for common robot operations.
 * The {@link HardwareDevices} nested class contains static members that describe the different types of hardware the robot uses.
 * This class acts as the main interface for controlling the robot, offering a structured and organized
 * approach to managing complex robotic systems.
 */
public class RobotContainer {
    public double startTimeMs = System.currentTimeMillis();
    public OpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public TelemetryManager panelsTelemetry;
    public boolean isRunning = false;
    public GamepadWrapper gamepadEx1;
    public GamepadWrapper gamepadEx2;
    public final Map<String, LinkedList<Double>> loopTimesMap = new HashMap<>();
    public final Map<String, ElapsedTime> loopTimers = new HashMap<>();
    private final ElapsedTime telemetryLoopTimer = new ElapsedTime();
    private final ArrayList<String> eventTelemetry = new ArrayList<>();
    private final ArrayList<String> eventTelemetryCaptions = new ArrayList<>();
    private final ArrayList<Object> eventTelemetryValues = new ArrayList<>();
    private Map<String, Object> currentLoopData = new HashMap<>();
    public TelemetryLogger telemetryLogger;
    public LocalizationUpdater localizationUpdater;
    public DelayedActionManager delayedActionManager = new DelayedActionManager(this);
    public PositionProvider positionProvider;
    public Drivetrain drivetrain;
    public IndicatorLighting.Light indicatorLightLeft;
    public IndicatorLighting.Light indicatorLightRight;
    public IndicatorLighting.Group allIndicatorLights;
    public Turret turret;
    public Intake intake;
    public Brakes brakes;
    public Lever lever;
    public double controlHubVoltage;
    public double expansionHubVoltage;
    public double controlHubCurrent;
    public double expansionHubCurrent;
    public double switchCurrent;

    public ArrayList<String> telemetryHeaderList;
    public Map<String, ArrayList<String>> telemetryCache;

    public double CURRENT_LOOP_TIME_MS;
    public double PREV_LOOP_TIME_MS;

    public static BNO055IMU.Parameters betterIMUP = new BNO055IMU.Parameters();

    public static class HardwareDevices {
        public static List<LynxModule> allHubs;
        public static LynxModule controlHub;
        public static LynxModule expansionHub;
        public static IMU imu;
        public static BetterIMU betterIMU;

        public static GoBildaPinpointDriver pinpoint;
        public static AnalogInput switchCurrentSensor;

        public static BetterDcMotor leftFront;
        public static BetterDcMotor rightFront;
        public static BetterDcMotor leftBack;
        public static BetterDcMotor rightBack;

        // Gobilda RGB indicator light
        public static ServoImplEx indicatorLightLeft;
        public static ServoImplEx indicatorLightRight;

//        // Turret
        public static BetterDcMotor flywheelMotorLeader;
        public static BetterDcMotor flywheelMotorFollower;
        public static BetterServo turretServoLeader;
        public static BetterServo turretServoFollower;
        public static BetterServo hoodServo;
        public static BetterServo leverServo;

        // Intake
        public static BetterDcMotor intakeMotorLeader;
        public static BetterDcMotor intakeMotorFollower;

        // Brakes
        public static BetterServo brakeServoLeader;
        public static BetterServo brakeServoFollower;

    }

    public RobotContainer(OpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.telemetryCache = new HashMap<>();
        this.telemetryHeaderList = new ArrayList<>();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        HardwareDevices.indicatorLightLeft = getHardwareDevice(ServoImplEx.class, "indicatorLightLeft");
        HardwareDevices.indicatorLightRight = getHardwareDevice(ServoImplEx.class, "indicatorLightRight");
        indicatorLightLeft = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightLeft);
        indicatorLightRight = new IndicatorLighting.Light(this, HardwareDevices.indicatorLightRight);
        allIndicatorLights = new IndicatorLighting.Group(this);
        allIndicatorLights.addLight(indicatorLightLeft);
        allIndicatorLights.addLight(indicatorLightRight);

        allIndicatorLights.setColor(Constants.LED.COLOR.RED);

        HardwareDevices.imu = getHardwareDevice(IMU.class, "imu");
        HardwareDevices.imu.initialize(Constants.Robot.IMU_PARAMETERS);

        HardwareDevices.betterIMU = new BetterIMU(getHardwareDevice(AdafruitBNO055IMU.class, "betterIMU"));
        betterIMUP.mode = BNO055IMU.SensorMode.IMU;
        betterIMUP.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        betterIMUP.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        betterIMUP.calibrationDataFile = "AdafruitIMUCalibration.json";

        HardwareDevices.betterIMU.initialize(betterIMUP);
        panelsTelemetry.debug("Calibrating External IMU...");
        panelsTelemetry.update(telemetry);
        Thread.sleep(800); // Ensure that the IMU has some still time so that it will auto calibrate at zero.

        HardwareDevices.pinpoint = getHardwareDevice(GoBildaPinpointDriver.class, "pinpoint");
        HardwareDevices.pinpoint.setOffsets(Constants.System.PINPOINT_X_OFFSET_MM, Constants.System.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        HardwareDevices.pinpoint.setEncoderResolution(Constants.System.PINPOINT_ODOM_POD);
        HardwareDevices.pinpoint.setEncoderDirections(Constants.System.PINPOINT_X_ENCODER_DIRECTION, Constants.System.PINPOINT_Y_ENCODER_DIRECTION);

        HardwareDevices.switchCurrentSensor = getHardwareDevice(AnalogInput.class, "switchCurrentSensor");

        HardwareDevices.leftFront = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "leftFront"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.rightFront = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "rightFront"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.leftBack = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "leftBack"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.rightBack = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "rightBack"), Constants.Robot.MOTOR_UPDATE_TIME);

        HardwareDevices.leftFront.setDirection(DcMotorImplEx.Direction.REVERSE);
        HardwareDevices.rightFront.setDirection(DcMotorImplEx.Direction.FORWARD);
        HardwareDevices.leftBack.setDirection(DcMotorImplEx.Direction.REVERSE);
        HardwareDevices.rightBack.setDirection(DcMotorImplEx.Direction.FORWARD);

        HardwareDevices.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HardwareDevices.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HardwareDevices.turretServoLeader = new BetterServo(getHardwareDevice(ServoImplEx.class, "turretServoLeader"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.turretServoFollower = new BetterServo(getHardwareDevice(ServoImplEx.class, "turretServoFollower"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.hoodServo = new BetterServo(getHardwareDevice(ServoImplEx.class, "hoodServo"), Constants.Robot.SERVO_UPDATE_TIME);

        HardwareDevices.intakeMotorLeader = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "intakeMotorLeader"), Constants.Robot.MOTOR_UPDATE_TIME);
        HardwareDevices.intakeMotorFollower = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "intakeMotorFollower"), Constants.Robot.MOTOR_UPDATE_TIME);
        LinkedMotors intakeMotors = new LinkedMotors(HardwareDevices.intakeMotorLeader, HardwareDevices.intakeMotorFollower);

        HardwareDevices.flywheelMotorLeader = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "flywheelMotorLeader"), 0);
        HardwareDevices.flywheelMotorFollower = new BetterDcMotor(hardwareMap.get(DcMotorImplEx.class, "flywheelMotorFollower"), 0);

        HardwareDevices.brakeServoLeader = new BetterServo(getHardwareDevice(ServoImplEx.class, "brakesServoLeader"), Constants.Robot.SERVO_UPDATE_TIME);
        HardwareDevices.brakeServoFollower = new BetterServo(getHardwareDevice(ServoImplEx.class, "brakesServoFollower"), Constants.Robot.SERVO_UPDATE_TIME);
        LinkedServos brakeServos = new LinkedServos(HardwareDevices.brakeServoLeader, HardwareDevices.brakeServoFollower);

        HardwareDevices.leverServo = new BetterServo(getHardwareDevice(ServoImplEx.class, "leverServo"), Constants.Robot.SERVO_UPDATE_TIME);

        LinkedMotors flywheelMotors = new LinkedMotors(HardwareDevices.flywheelMotorLeader, HardwareDevices.flywheelMotorFollower);
        HardwareDevices.flywheelMotorFollower.setDirection(DcMotorImplEx.Direction.REVERSE);
        flywheelMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinkedServos turretServos = new LinkedServos(HardwareDevices.turretServoLeader, HardwareDevices.turretServoFollower);
        positionProvider = new PositionProvider(this, HardwareDevices.pinpoint);

        turret = new Turret(this, flywheelMotors, HardwareDevices.hoodServo, turretServos);
        HardwareDevices.intakeMotorLeader.setDirection(DcMotor.Direction.REVERSE);
        intake = new Intake(this, intakeMotors);
        brakes = new Brakes(this, brakeServos);
        lever = new Lever(this, HardwareDevices.leverServo);

        drivetrain = new Drivetrain(this, HardwareDevices.leftFront, HardwareDevices.rightFront, HardwareDevices.leftBack, HardwareDevices.rightBack);

        registerLoopTimer("teleOp");
        registerLoopTimer("pinpointUpdater");
        registerLoopTimer("telemetryLogger");
    }

    public void init() {
        this.isRunning = true;
        RobotContainer.HardwareDevices.imu.resetYaw();
        HardwareDevices.allHubs = hardwareMap.getAll(LynxModule.class);
        HardwareDevices.controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        HardwareDevices.expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Reset the IMU angle
        HardwareDevices.betterIMU.resetAngle();

        if (Status.competitionMode) {
            telemetry.setMsTransmissionInterval(Constants.System.TELEMETRY_COMP_UPDATE_INTERVAL_MS);
        } else {
            telemetry.setMsTransmissionInterval(Constants.System.TELEMETRY_UPDATE_INTERVAL_MS);
        }

        panelsTelemetry.debug("Calibrating Pinpoint...");
        panelsTelemetry.update(telemetry);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void initLoop() {
        panelsTelemetry.debug("Status: Initialized");
        panelsTelemetry.debug("Alliance: " + Status.alliance.toString());
        panelsTelemetry.update(telemetry);
        allIndicatorLights.rainbow();
    }

    public void start(OpMode opmode, boolean teleop) {
        gamepadEx1 = new GamepadWrapper(opmode.gamepad1);
        gamepadEx2 = new GamepadWrapper(opmode.gamepad2);
        Status.isDrivingActive = false;
        if (teleop) {
            Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.RED ?
                    new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45) :
                    Status.alliance == Constants.Game.ALLIANCE.BLUE ?
                            new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, 45) :
                            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        } else {
            Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.INCH, 70, -70, AngleUnit.DEGREES, 45) : new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45);
        }
        Status.goalsideStartingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.INCH, Constants.Robot.GOALSIDE_STARTING_X, Constants.Robot.GOALSIDE_STARTING_Y, AngleUnit.DEGREES, Constants.Robot.GOALSIDE_STARTING_HEADING) :
                                      Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Robot.GOALSIDE_STARTING_X, -Constants.Robot.GOALSIDE_STARTING_Y, AngleUnit.DEGREES, Constants.Robot.GOALSIDE_STARTING_HEADING) :
                                      new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

        Status.startingPose = Status.alliance == Constants.Game.ALLIANCE.RED ? new Pose2D(DistanceUnit.CM, Constants.Robot.STARTING_X, Constants.Robot.STARTING_Y, AngleUnit.DEGREES, Constants.Robot.STARTING_HEADING) :
                              Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.CM, Constants.Robot.STARTING_X, -Constants.Robot.STARTING_Y, AngleUnit.DEGREES, -Constants.Robot.STARTING_HEADING) :
                              new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);



        // Start the required threads
        telemetryLogger = new TelemetryLogger(this);
        telemetryLogger.start();
        localizationUpdater = new LocalizationUpdater(this, teleop);
        localizationUpdater.start();
        telemetryLoopTimer.reset();
    }

    public void stop() {
        Status.opModeIsActive = false;
        if (this.localizationUpdater != null) {
            this.localizationUpdater.stopThread();
            try {
                this.localizationUpdater.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        if (this.telemetryLogger != null) {
            this.telemetryLogger.stopThread();
            try {
                this.telemetryLogger.join();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void update(boolean teleop) {
        CURRENT_LOOP_TIME_MS = updateLoopTime("teleOp");
        refreshData();
        if (teleop){
            gamepadEx1.update();
            gamepadEx2.update();
            drivetrain.driveJoystickUpdate();
            drivetrain.update();
            positionProvider.update();
        }
        delayedActionManager.update();
        allIndicatorLights.lightsUpdate();
        lever.update(teleop);
        turret.update(teleop);
        brakes.update(teleop);
        intake.update(teleop);

        controlHubVoltage = getVoltage(Constants.Robot.CONTROL_HUB_INDEX);
        switchCurrent = (HardwareDevices.switchCurrentSensor.getVoltage() / 3.3) * 80;

        PREV_LOOP_TIME_MS = CURRENT_LOOP_TIME_MS;

        if (teleop) {
            telemetry("teleOp");
        }

        if (Constants.System.USE_BETTER_IMU) {
            Status.currentHeading = RobotContainer.HardwareDevices.betterIMU.getAngle();
        } else {
            Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
        }
    }

    /**
     * Retrieves a hardware device from the hardware map.
     * <p>
     * This method attempts to retrieve a specific hardware device from the hardware map,
     * based on the provided class type and device name. If the device is found, it is
     * returned; otherwise, an error message is added to the telemetry, and null is returned.
     *
     * @param <T>           The type of the hardware device being requested.
     * @param hardwareClass The class of the hardware device (e.g., DcMotor.class, Servo.class).
     * @param name          The name of the hardware device as configured in the Robot Controller app.
     * @return The requested hardware device if found; null otherwise.
     */
    public <T> T getHardwareDevice(Class<T> hardwareClass, String name) {
        try {
            return hardwareMap.get(hardwareClass, name);
        } catch (Exception e) {
            telemetry.addLine("Could not load hardware class: '" + name + "' and got error: '" + e + "'");
            return null; // Or throw the exception if you prefer
        }
    }

    /**
     * Add or update a retained line of telemetry.
     */
    public void addEventTelemetry(String caption, Object value) {
        eventTelemetryCaptions.add(caption);
        eventTelemetryValues.add(value);
        eventTelemetry.add("TIME OF EVENT" + ": " + (System.currentTimeMillis() - startTimeMs)+ "\n" + caption + ": " + value.toString());
    }

    public void displayEventTelemetry() {
        for (int i = 0; i < eventTelemetryCaptions.size(); i++) {
            telemetry.addData(eventTelemetryCaptions.get(i), eventTelemetryValues.get(i));
        }
    }

    /** This is for hardware error that are critical and code execution should stop to tell the user of the error. */
    public void testCriticalHardwareDevice(Object hardwareClass) {
        if (hardwareClass == null) {
            telemetry.log().clear();
            telemetry.addLine("Failed to load hardware class, class is null");
            telemetry.addLine("This message will show for 10 seconds.");
            telemetry.update();
            try {
                Thread.sleep(10000); // 10 seconds = 10000 milliseconds.
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Do something here, prob something else.
            }
        }
    }

    /**
     * Refreshes the data from all Lynx Modules (hubs) by clearing their bulk data cache.
     * <p>
     * This method is crucial for ensuring that the robot is operating with the most up-to-date
     * sensor and motor data. The Lynx Modules use a bulk data cache to optimize data transfer.
     * However, if data in this cache becomes stale, the robot's actions might be based on
     * outdated information.
     * <p>
     * This method should be called periodically or whenever you suspect that the data in the
     * bulk cache might be outdated. Common scenarios include:
     * - At the start of a new control loop iteration in teleop or autonomous.
     * - After a significant delay or pause in the robot's operation.
     * - Before reading critical sensor values that need to be absolutely current.
     * - If there is a change in the bulk caching mode.
     * <p>
     * Calling this method ensures that the next time you read data from the hubs, the latest
     * information will be fetched, rather than possibly outdated cached data.
     */
    public void refreshData() {
        for (LynxModule hub : HardwareDevices.allHubs) {
            hub.clearBulkCache();
        }
    }

    public double getVoltage(int hubIndex) {
        LynxModule selectedHub;

        // Determine which hub to use based on hubIndex
        switch (hubIndex) {
            case 0:
                selectedHub = HardwareDevices.controlHub;
                break;
            case 1:
                selectedHub = HardwareDevices.expansionHub;
                break;
            default:
                // Invalid index
                opMode.telemetry.addLine("ERROR: Invalid hub index");
                opMode.telemetry.update();
                return -1; // Or throw an exception
        }

        if (selectedHub == null) {
            opMode.telemetry.addLine("ERROR: Hub not found");
            opMode.telemetry.update();
            return -1;
        }

        return selectedHub.getInputVoltage(VoltageUnit.VOLTS);
    }

    public double getCurrent(int hubIndex) {
        LynxModule selectedHub;

        // Determine which hub to use based on hubIndex
        switch (hubIndex) {
            case 0:
                selectedHub = HardwareDevices.controlHub;
                break;
            case 1:
                selectedHub = HardwareDevices.expansionHub;
                break;
            default:
                // Invalid index
                addEventTelemetry("ERROR", "Invalid hub index");
                return -1; // Or throw an exception
        }

        if (selectedHub == null) {
            addEventTelemetry("ERROR", "Hub not found");
            return -1;
        }

        return selectedHub.getCurrent(CurrentUnit.AMPS);
    }

    public synchronized void registerLoopTimer(String name) {
        loopTimesMap.put(name, new LinkedList<>());
        loopTimers.put(name, new ElapsedTime());
    }

    /**
     * Call at the top of your named loop.
     * @return The elapsed time (ms) since last call for this name.
     */
    public synchronized double updateLoopTime(String name) {
        ElapsedTime timer = loopTimers.get(name);
        LinkedList<Double> times = loopTimesMap.get(name);
        if (timer == null || times == null) {
            throw new IllegalArgumentException("LoopTimer not registered: " + name);
        }
        double dt = timer.milliseconds();
        timer.reset();
        times.add(dt);
        if (times.size() > Constants.System.LOOP_AVERAGE_WINDOW_SIZE) {
            times.removeFirst();
        }
        return dt;
    }

    /** Call anywhere to get the rolling average for that loop name. */
    public synchronized double getRollingAverageLoopTime(String name) {
        LinkedList<Double> times = loopTimesMap.get(name);
        if (times == null || times.isEmpty()) return 0;
        double sum = 0;
        for (double t : times) sum += t;
        return sum / times.size();
    }

    public synchronized double getLoopTime(String name) {
        LinkedList<Double> times = loopTimesMap.get(name);
        if (times == null || times.isEmpty()) return 0;
        return times.getLast();
    }

    public void writeToFile (String fileName, String data) {
        File myFileName = AppUtil.getInstance().getSettingsFile(fileName);
        ReadWriteFile.writeFile(myFileName, data);
    }

    public void addDataLog(String caption, Object data, boolean driveStation) {
        if (driveStation) {
            telemetry.addData(caption, data);
        }

        if (!Status.loggingToFile) {
            return;
        }

        if (data == null) data = "null";

        String dataString = data.toString();

        dataString = dataString.replaceAll(",", "|");


        // Add new headers if needed
        if (!telemetryHeaderList.contains(caption)) {
            telemetryHeaderList.add(caption);
            telemetryCache.put(caption, new ArrayList<>());
        }

        // Put this loop’s value in the buffer
        currentLoopData.put(caption, dataString);
    }

    public void commitLoopData() {
        if (!Status.loggingToFile) {
            return;
        }
        // Determine the current row number
        int row = telemetryCache.get(telemetryHeaderList.get(0)).size();

        // For every column, add value from buffer or empty string
        for (String header : telemetryHeaderList) {
            ArrayList<String> column = telemetryCache.get(header);
            String value = currentLoopData.getOrDefault(header, "").toString();
            column.add(value);
        }

        // Clear the buffer for the next loop
        currentLoopData.clear();
    }

    public void writeDataLog() {
        StringBuilder csvLog = new StringBuilder();

        for (int i = 0; i < telemetryHeaderList.size(); i++) {
            csvLog.append(telemetryHeaderList.get(i));
            if (i < telemetryHeaderList.size() - 1) csvLog.append(',');
        }
        csvLog.append('\n');

        int rowCount = 0;
        if (!telemetryHeaderList.isEmpty()) {
            rowCount = telemetryCache.get(telemetryHeaderList.get(0)).size();
        }

        for (int row = 0; row < rowCount; row++) {
            for (int col = 0; col < telemetryHeaderList.size(); col++) {
                String header = telemetryHeaderList.get(col);
                ArrayList<String> columnData = telemetryCache.get(header);
                csvLog.append(columnData.get(row));
                if (col < telemetryHeaderList.size() - 1) csvLog.append(',');
            }
            csvLog.append('\n');
        }

        writeToFile("TelemetryLog.txt", csvLog.toString());
    }

    public void writeEventLog() {
        StringBuilder log = new StringBuilder();

        for (int i = 0; i < eventTelemetry.size(); i++) {
            log.append(eventTelemetry.get(i));
            log.append('\n');
            log.append('\n');
        }

        writeToFile("EventLog.txt", log.toString());
    }

    public void telemetry(String opMode) {
        if (telemetryLoopTimer.milliseconds() < Constants.System.TELEMETRY_UPDATE_INTERVAL_MS && !Status.competitionMode) {
            return;
        } else if (telemetryLoopTimer.milliseconds() < Constants.System.TELEMETRY_COMP_UPDATE_INTERVAL_MS && Status.competitionMode) {
            return;
        }
        // This line is required for the logViwer to work correctly
        // It also need to be at column zero and spelled exactly "Time Stamp"
        addDataLog("Time Stamp", System.currentTimeMillis() - startTimeMs, true);

        // Stuff also in competition mode
        addDataLog("Alliance", Status.alliance, true);
        telemetry.addLine();
        addDataLog("OpMode Avg Loop Time", (int) getRollingAverageLoopTime("teleOp") + " ms", true);
        addDataLog("Pinpoint Avg Loop Time", (int) getRollingAverageLoopTime("pinpointUpdater") + " ms", true);
        telemetry.addLine();
        addDataLog("Switch Amps", switchCurrent + " A", true);
        telemetry.addLine();
        addDataLog("Pinpoint X", Status.currentPose.getX(DistanceUnit.CM) + " cm", true);
        addDataLog("Pinpoint Y", Status.currentPose.getY(DistanceUnit.CM) + " cm", true);
        addDataLog("Robot Heading", Status.currentHeading + "°", true);
        addDataLog("Logging to file", Status.loggingToFile, true);

        addDataLog("BetterIMU Yaw", Status.currentHeading, true); // First angle is the yaw
        addDataLog("Pinpoint Yaw", HardwareDevices.pinpoint.getHeading(AngleUnit.DEGREES), true); // First angle is the yaw
        addDataLog("Use Better IMU", Constants.System.USE_BETTER_IMU, true);

        if (!Status.competitionMode) {

            // Stuff not in competition mode.

            // Get current and voltage for telemetry
//                controlHubVoltage = getVoltage(Constants.Robot.CONTROL_HUB_INDEX);
//                expansionHubVoltage = getVoltage(Constants.Robot.EXPANSION_HUB_INDEX);
//                controlHubCurrent = getCurrent(Constants.Robot.CONTROL_HUB_INDEX);
//                expansionHubCurrent = getCurrent(Constants.Robot.EXPANSION_HUB_INDEX);
//                addDataLog("Control Hub Voltage", controlHubVoltage + " V", true);
//                addDataLog("Expansion Hub Voltage", expansionHubVoltage + " V", true);
//                addDataLog("Control Hub Current", controlHubCurrent + " A", true);
//                addDataLog("Expansion Hub Current", expansionHubCurrent + " A", true);
//                addDataLog("Switch Amps", switchCurrent + " A", true);
//                telemetry.addLine();
//                addDataLog("Flywheel Target Velocity", turret.flywheel.targetVelocity, true);
//                addDataLog("Flywheel Current Velocity", HardwareDevices.flywheelMotorLeader.getVelocity(), true);
//                addDataLog("Flywheel Main Motor Current mA", HardwareDevices.flywheelMotorLeader.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Flywheel Secondary Motor Current mA", HardwareDevices.flywheelMotorFollower.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Flywheel2 Current Velocity", HardwareDevices.flywheelMotorFollower.getVelocity(), true);
//                telemetry.addLine();
//                addDataLog("Turret Current Angle", turret.getPositionDegrees(), true);
//                addDataLog("Hood Position", HardwareDevices.hoodServo.getPosition(), true);
//                telemetry.addLine();
//                addDataLog("Brakes position", brakes.getPosition(), true);
//                addDataLog("Brakes angle", brakes.getPositionDegrees(), true);
//                telemetry.addLine();
//                addDataLog("Robot Heading", Status.currentHeading + "°", true);
//                addDataLog("Pinpoint Heading", RobotContainer.HardwareDevices.pinpoint.getPosition().getHeading(AngleUnit.DEGREES), true);
//                addDataLog("Pinpoint Status", RobotContainer.HardwareDevices.pinpoint.getDeviceStatus(), true);
//                addDataLog("Distance to Goal", HelperFunctions.disToGoal(), true);
//                telemetry.addLine();
//                addDataLog("OpMode Loop Time", getLoopTime("teleOp") + " ms", true);
//                addDataLog("Pinpoint Loop Time", (int) getLoopTime("pinpointUpdater") + " ms", true);
//                telemetry.addLine();
//                addDataLog("Goal Position", Status.goalPose, true);
//                addDataLog("Start Position", Status.startingPose, true);
//                telemetry.addLine();
//                addDataLog("Field Oriented", Status.fieldOriented, true);
//                addDataLog("Intake Toggle", Status.intakeGamepadable, true);
//
//                // Test Power draw
//                addDataLog("Motor Intake 1", HardwareDevices.intakeMotorLeader.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Intake 2", HardwareDevices.intakeMotorFollower.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Flywheel 1", HardwareDevices.flywheelMotorLeader.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Flywheel 2", HardwareDevices.flywheelMotorFollower.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Drive LF", HardwareDevices.leftFront.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Drive RF", HardwareDevices.rightFront.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Drive LB", HardwareDevices.leftBack.getCurrent(CurrentUnit.MILLIAMPS), true);
//                addDataLog("Motor Drive RB", HardwareDevices.rightBack.getCurrent(CurrentUnit.MILLIAMPS), true);
//
//                addDataLog("Servo Hood", HardwareDevices.hoodServo.getPosition(), true);
//                addDataLog("Servo Turret Leader", HardwareDevices.turretServoLeader.getPosition(), true);
//                addDataLog("Servo Turret Follower", HardwareDevices.turretServoFollower.getPosition(), true);
//                addDataLog("Brakes Servo Leader", HardwareDevices.brakesServoLeader.getPosition(), true);
//                addDataLog("Brakes Servo Follower", HardwareDevices.brakesServoFollower.getPosition(), true);
//
                telemetry.addLine();
                displayEventTelemetry();
                commitLoopData();
//
//                panelsTelemetry.addData("Flywheel Error", turret.flywheelError);
//                panelsTelemetry.addData("Flywheel Target", turret.flywheel.targetVelocity);
//                panelsTelemetry.addData("Flywheel Velocity", turret.flywheel.getFlywheelVelocity());
//                panelsTelemetry.addData("Lever Open", Status.leverOpen);
//                panelsTelemetry.addData("New Intake Power", intake.getNewPower());
//
//                panelsTelemetry.addData("Current", switchCurrent + " A");
//                panelsTelemetry.addData("Intake Current", intake.getCurrent() + "A");
//                panelsTelemetry.addData("Drive Current", drivetrain.getCurrent() + "A");
//                panelsTelemetry.addData("Flywheel Current", turret.flywheel.getCurrent() + "A");
        }

        panelsTelemetry.update(telemetry);
        telemetry.update();
    }
}
