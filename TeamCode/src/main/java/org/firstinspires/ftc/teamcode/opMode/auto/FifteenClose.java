
package org.firstinspires.ftc.teamcode.opMode.auto;

import androidx.appcompat.widget.ThemedSpinnerAdapter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

@Autonomous(name = "15Close", group = "Autonomous")
@Configurable // Panels
public class FifteenClose extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private RobotContainer robotContainer;
    private int pathState; // Current autonomous path state (state machine)
    private FifteenClose.Paths paths; // Paths defined in the Paths class
    private boolean holding;
    private boolean actionRun;
    private ElapsedTime pathTimer = new ElapsedTime(); // Path timer
    Runnable start;
    Runnable shoot;
    Runnable goToEnd;
    Runnable intake;
    Runnable endOfIntake;
    double startingX = 124.5;
    double startingY = 124.5;
    double startingHeading = 218;
    double startingPoseHeading = startingHeading - 90;
    double SHOOT_WAIT_TIME = 500;
    double SHOOTING_TIME = 1000;
    double GATE_WAIT_TIME = 750;
    double POSE_OFFSET = 24;

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotContainer.init();

        follower = PedroConstants.createFollower(hardwareMap);
        if (Status.alliance == org.firstinspires.ftc.teamcode.main.Constants.Game.ALLIANCE.RED) {
            follower.setStartingPose(new Pose(startingX, startingY, Math.toRadians(startingHeading)));
            RobotContainer.HardwareDevices.betterIMU.setAngle(HelperFunctions.normalizeAngle(startingPoseHeading));
        } else {
            follower.setStartingPose(HelperFunctions.mirror(new Pose(startingX, startingY, Math.toRadians(startingHeading))));
            RobotContainer.HardwareDevices.betterIMU.setAngle(HelperFunctions.normalizeAngle(HelperFunctions.mirrorAngle0(startingPoseHeading)));
        }

        paths = new FifteenClose.Paths(follower); // Build paths
        holding = false;
        actionRun = false;

        POSE_OFFSET = Status.alliance == Constants.Game.ALLIANCE.RED ? POSE_OFFSET : -POSE_OFFSET;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", Status.alliance);
        panelsTelemetry.update(telemetry);

        //Actions
        start = new Runnable() {
            @Override
            public void run() {
                Status.flywheelToggle = true;
                robotContainer.door.close();
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);
                robotContainer.intake.setPower(1.0);
            }
        };

        shoot = new Runnable() {
            @Override
            public void run(){
                robotContainer.door.open();
                robotContainer.intake.setPower(1.0);

                robotContainer.delayedActionManager.schedule(() -> robotContainer.door.close(), (int) (SHOOTING_TIME * 2.0 / 3.0));
                robotContainer.delayedActionManager.schedule(() -> robotContainer.door.open(), (int) (SHOOTING_TIME * 5.0 / 6.0));

                robotContainer.delayedActionManager.schedule(() -> robotContainer.door.close(), (int) (SHOOTING_TIME * 23.0/24.0));

                robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(0.7), (int) (SHOOTING_TIME));
            }
        };

        intake = new Runnable (){
            @Override
            public void run() {

            }
        };

        endOfIntake = new Runnable (){
            @Override
            public void run() {

            }
        };

        goToEnd = new Runnable (){
            @Override
            public void run() {
                Status.flywheelToggle = false;
                robotContainer.intake.setPower(0.0);
            }
        };
    }

    @Override
    public void start(){
        robotContainer.start(this, false);
        start.run();
        pathTimer.reset();
        setPathState(0);
        Status.currentPose = new Pose2D(DistanceUnit.INCH, 124-72, 124-72, AngleUnit.DEGREES, HelperFunctions.normalizeAngle(218));
    }

    @Override
    public void init_loop() {
        robotContainer.initLoop();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Update pose for turret
        Status.currentPose = new Pose2D(DistanceUnit.INCH, follower.getPose().getY() - 72, -(follower.getPose().getX() - 72), AngleUnit.DEGREES, HelperFunctions.normalizeAngle(RobotContainer.HardwareDevices.betterIMU.getAngle()));

        robotContainer.update(false);

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Turret Target", robotContainer.turret.getTargetPosition());
        panelsTelemetry.debug("Turret Target Degrees", robotContainer.turret.getTargetPositionDegrees());
        panelsTelemetry.debug("Path Timer", pathTimer.milliseconds());
        panelsTelemetry.debug("Holding", holding);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Current Pose", Status.currentPose);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        blackboard.put("Robot Pose", new Pose2D(DistanceUnit.INCH, Status.currentPose.getX(DistanceUnit.INCH), (Status.currentPose.getY(DistanceUnit.INCH) + POSE_OFFSET), AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
    }

    private static class Paths {
        public PathChain
                BC_SC,
                SC_TM,
                TM_G,
                G_SC,
                TM_SC,
                SC_G,
                G_GC,
                SC_GC,
                GC_SC,
                SC_TL,
                TL_SC,
                SC_TH,
                TH_SC,
                SC_EC;
        public Pose
                // Red
                RED_BEGINNING_CLOSE = new Pose(124.500, 124.500, Math.toRadians(218)),
                RED_TAPE_MID = new Pose(126.000, 60.000, Math.toRadians(0)),
                RED_SHOOTING_CLOSE_65 = new Pose(84.000, 84.000, Math.toRadians(-65)),
                RED_SHOOTING_CLOSE_45 = new Pose(84, 84, Math.toRadians(-45)),
                RED_SHOOTING_CLOSE_0 = new Pose(84, 84, Math.toRadians(0)),
                RED_GATE = new Pose(122.000, 68.000, Math.toRadians(0)),
                RED_GATE_COLLECT = new Pose(115.000, 61, Math.toRadians(25)),
                RED_TAPE_LOW = new Pose(126.000, 35.000, Math.toRadians(0)),
                RED_TAPE_HIGH = new Pose(122.000, 84.000, Math.toRadians(0)),
                RED_END_CLOSE = new Pose(103.000, 84.000, Math.toRadians(0)),

                // Blue
                BLUE_BEGINNING_CLOSE = HelperFunctions.mirror(RED_BEGINNING_CLOSE),
                BLUE_TAPE_MID = HelperFunctions.mirror(RED_TAPE_MID),
                BLUE_SHOOTING_CLOSE_65 = HelperFunctions.mirror(RED_SHOOTING_CLOSE_65),
                BLUE_SHOOTING_CLOSE_45 = HelperFunctions.mirror(RED_SHOOTING_CLOSE_45),
                BLUE_SHOOTING_CLOSE_0 = HelperFunctions.mirror(RED_SHOOTING_CLOSE_0),
                BLUE_GATE = HelperFunctions.mirror(RED_GATE),
                BLUE_GATE_COLLECT = HelperFunctions.mirror(RED_GATE_COLLECT),
                BLUE_TAPE_LOW = HelperFunctions.mirror(RED_TAPE_LOW),
                BLUE_TAPE_HIGH = HelperFunctions.mirror(RED_TAPE_HIGH),
                BLUE_END_CLOSE = HelperFunctions.mirror(RED_END_CLOSE);

        public Paths(Follower follower) {
            if (Status.alliance == org.firstinspires.ftc.teamcode.main.Constants.Game.ALLIANCE.RED) {
                BC_SC = follower.pathBuilder().addPath(
                                new BezierLine(
                                        RED_BEGINNING_CLOSE,
                                        RED_SHOOTING_CLOSE_65
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                SC_TM = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_SHOOTING_CLOSE_65,
                                        new Pose(80.000, 60.000),
                                        RED_TAPE_MID
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TM_G = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_TAPE_MID,
                                        new Pose(115.000, 64.000),
                                        RED_GATE
                                )
                        ).setConstantHeadingInterpolation(0)
                        .build();

                G_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_GATE,
                                        new Pose(80.000, 60.000),
                                        RED_SHOOTING_CLOSE_65
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                TM_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_TAPE_MID,
                                        new Pose(80.000, 60.000),
                                        RED_SHOOTING_CLOSE_65
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_G = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_SHOOTING_CLOSE_65,
                                        new Pose(95.000, 62.000),
                                        RED_GATE
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                G_GC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_GATE,
                                        new Pose(115.000, 64.000),
                                        RED_GATE_COLLECT
                                )
                        ).setLinearHeadingInterpolation(RED_GATE.getHeading(), RED_GATE_COLLECT.getHeading())
                        .build();

                SC_GC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_SHOOTING_CLOSE_65,
                                        new Pose(95.000, 62.000),
                                        RED_GATE_COLLECT
                                )
                        ).setLinearHeadingInterpolation(RED_SHOOTING_CLOSE_65.getHeading(), RED_GATE_COLLECT.getHeading())
                        .build();

                GC_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_GATE,
                                        new Pose(95.000, 62.000),
                                        RED_SHOOTING_CLOSE_45
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_TL = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_SHOOTING_CLOSE_45,
                                        new Pose(88.500, 38.000),
                                        RED_TAPE_LOW
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TL_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_TAPE_LOW,
                                        new Pose(88.500, 38.000),
                                        RED_SHOOTING_CLOSE_45
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_TH = follower.pathBuilder().addPath(
                                new BezierLine(
                                        RED_SHOOTING_CLOSE_45,
                                        RED_TAPE_HIGH
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TH_SC = follower.pathBuilder().addPath(
                                new BezierLine(
                                        RED_TAPE_HIGH,
                                        RED_SHOOTING_CLOSE_0
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_EC = follower.pathBuilder().addPath(
                                new BezierLine(
                                        RED_SHOOTING_CLOSE_0,
                                        RED_END_CLOSE
                                )
                        ).setTangentHeadingInterpolation()
                        .build();
            } else {
                BC_SC = follower.pathBuilder().addPath(
                                new BezierLine(
                                        BLUE_BEGINNING_CLOSE,
                                        BLUE_SHOOTING_CLOSE_65
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                SC_TM = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_CLOSE_65,
                                        HelperFunctions.mirror(new Pose(80.000, 60.000)),
                                        BLUE_TAPE_MID
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TM_G = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_TAPE_MID,
                                        HelperFunctions.mirror(new Pose(115.000, 64.000)),
                                        BLUE_GATE
                                )
                        ).setConstantHeadingInterpolation(180)
                        .build();

                G_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_GATE,
                                        HelperFunctions.mirror(new Pose(80.000, 60.000)),
                                        BLUE_SHOOTING_CLOSE_65
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                TM_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_TAPE_MID,
                                        HelperFunctions.mirror(new Pose(80.000, 60.000)),
                                        BLUE_SHOOTING_CLOSE_65
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_G = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_CLOSE_65,
                                        HelperFunctions.mirror(new Pose(95.000, 62.000)),
                                        BLUE_GATE
                                )
                        ).setTangentHeadingInterpolation()

                        .build();

                G_GC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_GATE,
                                        HelperFunctions.mirror(new Pose(115.000, 64.000)),
                                        BLUE_GATE_COLLECT
                                )
                        ).setLinearHeadingInterpolation(BLUE_GATE.getHeading(), BLUE_GATE_COLLECT.getHeading())
                        .build();

                SC_GC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_CLOSE_65,
                                        HelperFunctions.mirror(new Pose(95.000, 62.000)),
                                        BLUE_GATE_COLLECT
                                )
                        ).setLinearHeadingInterpolation(BLUE_SHOOTING_CLOSE_65.getHeading(), BLUE_GATE_COLLECT.getHeading())
                        .build();

                GC_SC = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_GATE,
                                        HelperFunctions.mirror(new Pose(95.000, 62.000)),
                                        BLUE_SHOOTING_CLOSE_45
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_TL = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_CLOSE_45,
                                        HelperFunctions.mirror(new Pose(88.500, 38.000)),
                                        BLUE_TAPE_LOW
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_TL = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_TAPE_LOW,
                                        HelperFunctions.mirror(new Pose(88.500, 38.000)),
                                        BLUE_SHOOTING_CLOSE_45
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_TH = follower.pathBuilder().addPath(
                                new BezierLine(
                                        BLUE_SHOOTING_CLOSE_45,
                                        BLUE_TAPE_HIGH
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TH_SC = follower.pathBuilder().addPath(
                                new BezierLine(
                                        BLUE_TAPE_HIGH,
                                        BLUE_SHOOTING_CLOSE_0
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SC_EC = follower.pathBuilder().addPath(
                                new BezierLine(
                                        BLUE_SHOOTING_CLOSE_0,
                                        BLUE_END_CLOSE
                                )
                        ).setTangentHeadingInterpolation()
                        .build();
            }
        }
    }


    public void autonomousPathUpdate() {

        if (!follower.isBusy() && !holding) {
            pathTimer.reset();
            holding = true;
            actionRun = false;
        }

        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(paths.BC_SC, true);
                    incrementPathState();
                }
                break;
            case 1:
                if (holding) {
                    if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                        shoot.run();
                        actionRun = true;
                    }

                    if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                        incrementPathState();
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SC_TM, false);
                    incrementPathState();
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.TM_G, true);
                    incrementPathState();
                }
            case 4:
                if (holding && pathTimer.milliseconds() > GATE_WAIT_TIME) {
                    follower.followPath(paths.G_SC, true);
                    incrementPathState();
                }
                break;
            case 5:
                if (holding) {
                    if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                        shoot.run();
                        actionRun = true;
                    }

                    if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                        incrementPathState();
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SC_G, true);
                    incrementPathState();
                }
                break;
            case 7:
                incrementPathState();
            case 8:
                if (pathTimer.milliseconds() > GATE_WAIT_TIME && holding) {
                    incrementPathState();
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    endOfIntake.run();
                    follower.followPath(paths.G_SC, true);
                    incrementPathState();
                }
                break;
            case 10:
                if (holding) {
                    if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                        shoot.run();
                        actionRun = true;
                    }

                    if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                        incrementPathState();
                    }
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SC_GC, true);
                    incrementPathState();
                }
                break;
            case 12:
                incrementPathState();
            case 13:
                if (pathTimer.milliseconds() > GATE_WAIT_TIME && holding) {
                    incrementPathState();
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    endOfIntake.run();
                    follower.followPath(paths.GC_SC, true);
                    incrementPathState();
                }
                break;
            case 15:
                if (holding) {
                    if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                        shoot.run();
                        actionRun = true;
                    }

                    if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                        incrementPathState();
                    }
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(paths.SC_TH, false);
                    intake.run();
                    incrementPathState();
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    endOfIntake.run();
                    follower.followPath(paths.TH_SC, true);
                    incrementPathState();
                }
                break;
            case 18:
                if (holding) {
                    if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                        shoot.run();
                        actionRun = true;
                    }

                    if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                        incrementPathState();
                    }
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    goToEnd.run();
                    follower.followPath(paths.SC_EC, false);
                    incrementPathState();
                }
                break;
            default:
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pathNum) {
        pathState = pathNum;
        pathTimer.reset();
        holding = false;
        actionRun = false;
    }

    public void incrementPathState() {
        pathState++;
        pathTimer.reset();
        holding = false;
        actionRun = false;
    }
}
