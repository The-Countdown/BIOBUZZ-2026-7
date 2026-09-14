package org.firstinspires.ftc.teamcode.main;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import java.util.HashMap;

/**
 * The `Constants` class provides a centralized location for all the fixed values and
 * Configurableurations used throughout the robot's code.
 */
public class Constants {

    @Configurable
    public static class Game {
        public enum ALLIANCE {
            BLUE,
            RED
        }
        // Red
        public static double goalFarXRed = 80.0;
        public static double goalFarYRed = 70.0;
        public static double goalOpposingXRed = 80.0;
        public static double goalOpposingYRed = 75.0;
        public static double goalXRed = 80.0;
        public static double goalYRed = 60.0;
        public static double goalWallXRed = 75.0;
        public static double goalWallYRed = 75.0;
        public static double closeGoalXRed = 70.0;
        public static double closeGoalYRed = 80.0;
        // Blue
        public static double goalFarXBlue = -80.0;
        public static double goalFarYBlue = 80.0;
        public static double goalOpposingXBlue = -50.0;
        public static double goalOpposingYBlue = 60.0;
        public static double goalXBlue = -60.0;
        public static double goalYBlue = 60.0;
        public static double goalWallXBlue = -60.0;
        public static double goalWallYBlue = 80.0;
        public static double closeGoalXBlue = -70.0;
        public static double closeGoalYBlue = 80.0;

        public static Pose2D ORIGIN = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
        // Starting positions of the sets of 3 artifacts
        public static Pose2D POSPPG = new Pose2D(DistanceUnit.CM, -91.44, -109.22, AngleUnit.DEGREES, 0);
        public static Pose2D POSPGP = new Pose2D(DistanceUnit.CM, -30.48, -109.22, AngleUnit.DEGREES, 0);
        public static Pose2D POSGPP = new Pose2D(DistanceUnit.CM, 30.48, -109.22, AngleUnit.DEGREES, 0);

        public enum MOTIF {
            GPP,
            PGP,
            PPG,
            UNKNOWN
        }

        public enum ARTIFACT_COLOR{
            PURPLE,
            GREEN,
            NONE,
            UNKNOWN
        }
        public static double GRAVITY = 9.811;
        public static double GOAL_HEIGHT = 1.016;
    }

    @Configurable
    public static class Robot {
        public static int MOTOR_UPDATE_TIME = 50;
        public static int SERVO_UPDATE_TIME = 100;

        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );

        public static double STARTING_X = 0, STARTING_Y = 0, STARTING_HEADING = 0;
        public static double GOALSIDE_STARTING_X = 0, GOALSIDE_STARTING_Y = 0, GOALSIDE_STARTING_HEADING = 0;

        public static Pose2D redCornerResetPose = new Pose2D (DistanceUnit.CM, -159, -160, AngleUnit.DEGREES, 88);
        public static Pose2D blueCornerResetPose = new Pose2D (DistanceUnit.CM, -159, 160, AngleUnit.DEGREES, -88);
        public static Pose2D middleResetPose = new Pose2D(DistanceUnit.CM, 165.0, 0.0, AngleUnit.DEGREES, 180);

        public static final int
                CONTROL_HUB_INDEX = 0,
                EXPANSION_HUB_INDEX = 1;

        public static double CORNER_X = -162.5;
        public static double CORNER_Y = 161.5;
        public static double CORNER_ANGLE = -90;
        public static int MOTOR_TICKS_PER_REVOLUTION = 28;
    }

    @Configurable
    public static class System {
        public static final int LOOP_AVERAGE_WINDOW_SIZE = 30;
        public static int TELEMETRY_UPDATE_INTERVAL_MS = 750;
        public static int TELEMETRY_COMP_UPDATE_INTERVAL_MS = 750;
        public static final double ANALOG_MAX_VOLTAGE = 3.3;
        public static int PINPOINT_UPDATE_DELAY_MS = 50;
        public static final double
                PINPOINT_X_OFFSET_MM = -119.6;
        public static final double PINPOINT_Y_OFFSET_MM = -2.86;
        public static final float PINPOINT_RESOLUTION = 19.68619876f;
        public static final GoBildaPinpointDriver.GoBildaOdometryPods
                PINPOINT_ODOM_POD = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD; //TODO: change later
        public static final GoBildaPinpointDriver.EncoderDirection
                PINPOINT_X_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public static double IMU_PER_ROTATION_OFFSET = 3.1;
        public static boolean USE_BETTER_IMU = false;
    }

    @Configurable
    public static class Control {
        public static double JOYSTICK_SCALER_EXPONENT = 1.0;
        public static double JOYSTICK_ROTATION_SCALER_EXPONENT = 0.5;

        //power per second
        public static double MAX_DRIVE_ACCELERATION = 3;
        public static double ZERO_POWER_TOLERANCE = 0.03;
    }

    @Configurable
    public static class LED {
        public static final HashMap<COLOR, LED_COLOR_VALUES> COLOR_MAP = new HashMap<COLOR, LED_COLOR_VALUES>() {{
            put(COLOR.OFF, new LED_COLOR_VALUES(500, 0.0));
            put(COLOR.RED, new LED_COLOR_VALUES(1100, 0.279));
            put(COLOR.ORANGE, new LED_COLOR_VALUES(1200, 0.333));
            put(COLOR.YELLOW, new LED_COLOR_VALUES(1300, 0.388));
            put(COLOR.SAGE, new LED_COLOR_VALUES(1400, 0.444));
            put(COLOR.GREEN, new LED_COLOR_VALUES(1500, 0.500));
            put(COLOR.AZURE, new LED_COLOR_VALUES(1600, 0.555));
            put(COLOR.BLUE, new LED_COLOR_VALUES(1700, 0.611));
            put(COLOR.INDIGO, new LED_COLOR_VALUES(1800, 0.666));
            put(COLOR.VIOLET, new LED_COLOR_VALUES(1900, 0.722));
            put(COLOR.WHITE, new LED_COLOR_VALUES(2500, 1.0));
        }};

        public enum COLOR {
            OFF,
            RED,
            ORANGE,
            YELLOW,
            SAGE,
            GREEN,
            AZURE,
            BLUE,
            INDIGO,
            VIOLET,
            WHITE
        }

        public static class LED_COLOR_VALUES {
            public final int MICROSECONDS; public final double ANALOG;
            public LED_COLOR_VALUES(int MICROSECONDS, double ANALOG) {
                this.MICROSECONDS = MICROSECONDS; this.ANALOG = ANALOG;
            }
        }

        public static double FLYWHEEL_GRADIANT_PERIOD = 100;
        public static double RAINBOW_SPEED = 0.003;
    }

    @Configurable
    public static class Tilt {
        public static double MAX = 1;
        public static double EXTENDED = 0.4;
        public static double MIN = 0;
        public static double PARK = 0.0;
    }

    @Configurable
    public static class Door {
        public static double OPEN = 0.45;
        public static double CLOSED = 0.56;
    }

    @Configurable
    public static class Turret {
        public static boolean ADJUST_TURRET_BY_MULTIPLIER = true;
        public static double FLYWHEEL_HEIGHT = 0.3048;
        public static double DESIRED_MAX_HEIGHT = 1.25;
        public static double MAX_SERVO = 1;
        public static double NEUTRAL_SERVO = 0.5;
        public static double MIN_SERVO = 0;
        public static double MAX_ANGLE = 180;
        public static double NEUTRAL_ANGLE = 0;
        public static double MIN_ANGLE = -180;
        public static double[] TURRET_INTERPOLATION_ANGLES = {-180,-90, 0, 90, 180};
        public static double[] TURRET_INTERPOLATION_POSITIONS = {0.9517, 0.7226, 0.4985, 0.2755, 0.0559};
        public static double SPEED_FACTOR = 0.0003;
        public static int FLYWHEEL_MAX_VELOCITY = 2700;
        public static int FLYWHEEL_MAX_POWER_ERROR = 100;
        public static double FLYWHEEL_CURVE = 4;
        public static double[] FLYWHEEL_SPEED_TABLE = {0.260, 0.270, 0.320, 0.330, 0.355, 0.440, 0.465};
        public static double[] FLYWHEEL_SPEED_TABLE_DISTANCES = {40, 56, 90, 95, 103, 127, 152};
        public static int FLYWHEEL_SPINUP_MS = 500;
        public static double[] HOOD_PRESETS = {0.58, 0.7, 0.95};
        public static double[] HOOD_TABLE_DISTANCES = {50, 90, 130};
        public static double FLYWHEEL_KP = 0.001;
        public static double FLYWHEEL_FAR_KP = 0.001;
        public static double FLYWHEEL_KI = 0.0;
        public static double FLYWHEEL_FAR_KI = 0.0;
        public static double FLYWHEEL_KD = 0.008;
        public static double FLYWHEEL_FAR_KD = 0.008;
        public static double FLYWHEEL_KF = 1.2;
        public static double FLYWHEEL_FAR_KF = 1.2;
        public static double FLYWHEEL_KS = 0.0;
        public static double FLYWHEEL_FAR_KS = 0.0;
        public static double FLYWHEEL_DROP_KP = 0.00;
        public static double FLYWHEEL_FAR_DROP_KP = 0.00;
        public static double PID_TOLERANCE_VELOCITY = 20;
        public static double TURRET_TOLERANCE_DEGREES = 1;
        public static double PRESSURE_OFFSET = 0.004;
        public static double GOAL_POSITION_VELOCITY_MULTIPLIER = 3;
        public static double TURRET_ANGLE_VELOCITY_MULTIPLIER = -0.32;
        public static double TURRET_ANGLE_ANGULAR_VELOCITY_MULTIPLIER = -0.3;
        public static double FLYWHEEL_POWER_VELOCITY_MULTIPLIER = 0.00085;

    }

    @Configurable
    public static class Intake {
        public static double TOP_SPEED = 1;
        public static double BEST_INTAKE_SPEED = 1;
        public static double REVERSE_TOP_SPEED = 0.5;
        public static double FLYWHEEL_TOLERANCE = 5000;
    }
}
