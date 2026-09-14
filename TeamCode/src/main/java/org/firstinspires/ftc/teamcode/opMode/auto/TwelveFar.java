
package org.firstinspires.ftc.teamcode.opMode.auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "12Far", group = "Autonomous")
@Configurable // Panels
public class TwelveFar extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private RobotContainer robotContainer;
    private int pathState; // Current autonomous path state (state machine)
    private TwelveFar.Paths paths; // Paths defined in the Paths class
    private boolean holding;
    private boolean actionRun;
    private ElapsedTime pathTimer = new ElapsedTime(); // Path timer
    Runnable start;
    Runnable shoot;
    Runnable goToEnd;
    Runnable intake;
    Runnable endOfIntake;

    double startingX = 55;
    double startingY = 9;
    double startingHeading = 90;
    double startingPoseHeading = startingHeading -90;
    double SHOOT_WAIT_TIME = 750;
    double SHOOTING_TIME = 1000;

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
        if (Status.alliance == org.firstinspires.ftc.teamcode.main.Constants.Game.ALLIANCE.BLUE) {
            follower.setStartingPose(new Pose(startingX, startingY, Math.toRadians(startingHeading)));
            RobotContainer.HardwareDevices.betterIMU.setAngle(HelperFunctions.normalizeAngle(startingPoseHeading));
        } else {
            follower.setStartingPose(HelperFunctions.mirror(new Pose(startingX, startingY, Math.toRadians(HelperFunctions.mirrorAngle0(startingHeading)))));
            RobotContainer.HardwareDevices.betterIMU.setAngle(HelperFunctions.normalizeAngle(HelperFunctions.mirrorAngle0(startingPoseHeading)));
        }

        paths = new TwelveFar.Paths(follower); // Build paths
        holding = false;
        actionRun = false;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", Status.alliance);
        panelsTelemetry.update(telemetry);

        //Actions
        start = new Runnable() {
            @Override
            public void run() {
                Status.flywheelToggle = true;
                robotContainer.door.close();
                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
                robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(1), 200);
            }
        };

        shoot = new Runnable() {
            @Override
            public void run(){
                robotContainer.door.open();

                robotContainer.delayedActionManager.schedule(() -> robotContainer.door.close(), (int) (SHOOTING_TIME * 2 / 3));
                robotContainer.delayedActionManager.schedule(() -> robotContainer.door.open(), (int) (SHOOTING_TIME * 2 / 3 + 250));

                robotContainer.delayedActionManager.schedule(() -> robotContainer.door.close(), (int) (SHOOTING_TIME));
            }
        };

        intake = new Runnable (){
            @Override
            public void run() {
                robotContainer.door.close();
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
        blackboard.put("Robot Pose", new Pose2D(DistanceUnit.INCH, Status.currentPose.getX(DistanceUnit.INCH), Status.currentPose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, Status.currentPose.getHeading(AngleUnit.DEGREES)));
    }

    private static class Paths {
        public PathChain BF_TL;
        public PathChain TL_SF;
        public PathChain SF_TM;
        public PathChain TM_SF;
        public PathChain SF_TH;
        public PathChain TH_SF;
        public PathChain SF_EF;
        public static Pose
            // Blue
            BLUE_BEGINNING_FAR = new Pose(55.000, 9.000, Math.toRadians(90)),
            BLUE_SHOOTING_FAR = new Pose(60.000, 12.000, Math.toRadians(180)),
            BLUE_TAPE_LOW = new Pose(17.000, 36.000, Math.toRadians(180)),
            BLUE_TAPE_MID = new Pose(13.000, 70.000, Math.toRadians(180)),
            BLUE_TAPE_HIGH = new Pose(17.000, 84.000, Math.toRadians(180)),
            BLUE_END_FAR = new Pose(35.000, 12.000, Math.toRadians(180)),

            // Red
            RED_BEGINNING_FAR = HelperFunctions.mirror(BLUE_BEGINNING_FAR),
            RED_SHOOTING_FAR = HelperFunctions.mirror(BLUE_SHOOTING_FAR),
            RED_TAPE_LOW = HelperFunctions.mirror(BLUE_TAPE_LOW),
            RED_TAPE_MID = HelperFunctions.mirror(BLUE_TAPE_MID),
            RED_TAPE_HIGH = HelperFunctions.mirror(BLUE_TAPE_HIGH),
            RED_END_FAR = HelperFunctions.mirror(BLUE_END_FAR);

        public Paths(Follower follower) {
            if (Status.alliance == org.firstinspires.ftc.teamcode.main.Constants.Game.ALLIANCE.BLUE) {
                BF_TL = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_BEGINNING_FAR,
                                        new Pose(56.000, 39.500),
                                        BLUE_TAPE_LOW
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TL_SF = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_TAPE_LOW,
                                        new Pose(30.000, 11.500),
                                        BLUE_SHOOTING_FAR
                                )
                        ).setConstantHeadingInterpolation(BLUE_TAPE_LOW.getHeading())
                        .build();

                SF_TM = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_FAR,
                                        new Pose(41.500, 93.500),
                                        new Pose(10.000, 40.000),
                                        new Pose(35.000, 67.000),
                                        BLUE_TAPE_MID
                                )
                        ).setConstantHeadingInterpolation(BLUE_TAPE_MID.getHeading())
                        .build();

                TM_SF = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_TAPE_MID,
                                        new Pose(24.000, 16.500),
                                        BLUE_SHOOTING_FAR
                                )
                        ).setConstantHeadingInterpolation(BLUE_TAPE_MID.getHeading())
                        .build();

                SF_TH = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_FAR,
                                        new Pose(63.000, 92.000),
                                        BLUE_TAPE_HIGH
                                )
                        ).setConstantHeadingInterpolation(BLUE_TAPE_HIGH.getHeading())
                        .build();

                TH_SF = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        BLUE_TAPE_HIGH,
                                        new Pose(26.500, 22.000),
                                        BLUE_SHOOTING_FAR
                                )
                        ).setConstantHeadingInterpolation(BLUE_TAPE_HIGH.getHeading())
                        .build();

                SF_EF = follower.pathBuilder().addPath(
                                new BezierLine(
                                        BLUE_SHOOTING_FAR,
                                        BLUE_END_FAR
                                )
                        ).setConstantHeadingInterpolation(BLUE_SHOOTING_FAR.getHeading())
                        .build();
            } else {
                BF_TL = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_BEGINNING_FAR,
                                        HelperFunctions.mirror(new Pose(56.000, 39.500)),
                                        RED_TAPE_LOW
                                )
                        ).setTangentHeadingInterpolation()
                        .build();

                TL_SF = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_TAPE_LOW,
                                        HelperFunctions.mirror(new Pose(30.000, 11.500)),
                                        RED_SHOOTING_FAR
                                )
                        ).setConstantHeadingInterpolation(RED_TAPE_LOW.getHeading())
                        .build();

                SF_TM = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_SHOOTING_FAR,
                                        HelperFunctions.mirror(new Pose(41.500, 93.500)),
                                        HelperFunctions.mirror(new Pose(10.000, 40.000)),
                                        HelperFunctions.mirror(new Pose(35.000, 67.000)),
                                        RED_TAPE_MID
                                )
                        ).setConstantHeadingInterpolation(RED_TAPE_MID.getHeading())
                        .build();

                TM_SF = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_TAPE_MID,
                                        HelperFunctions.mirror(new Pose(24.000, 16.500)),
                                        RED_SHOOTING_FAR
                                )
                        ).setConstantHeadingInterpolation(RED_TAPE_MID.getHeading())
                        .build();

                SF_TH = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_SHOOTING_FAR,
                                        HelperFunctions.mirror(new Pose(63.000, 92.000)),
                                        RED_TAPE_HIGH
                                )
                        ).setConstantHeadingInterpolation(RED_TAPE_HIGH.getHeading())
                        .build();

                TH_SF = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        RED_TAPE_HIGH,
                                        HelperFunctions.mirror(new Pose(26.500, 22.000)),
                                        RED_SHOOTING_FAR
                                )
                        ).setConstantHeadingInterpolation(RED_TAPE_HIGH.getHeading())
                        .build();

                SF_EF = follower.pathBuilder().addPath(
                                new BezierLine(
                                        RED_SHOOTING_FAR,
                                        RED_END_FAR
                                )
                        ).setConstantHeadingInterpolation(RED_SHOOTING_FAR.getHeading())
                        .build();
            }
        }
    }


    public void autonomousPathUpdate() {

        if (!follower.isBusy() && !holding){
            pathTimer.reset();
            holding = true;
            actionRun = false;
        }

        switch (pathState) {
            case 0:
                if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME * 5){
                    shoot.run();
                    actionRun = true;
                }

                if (pathTimer.milliseconds() > SHOOT_WAIT_TIME * 5 + SHOOTING_TIME) {
                    follower.followPath(paths.BF_TL, true);
                    intake.run();
                    incrementPathState();
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    endOfIntake.run();
                    follower.followPath(paths.TL_SF, true);
                    incrementPathState();
                }
                break;
            case 2:
                if (holding) {
                        if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                            shoot.run();
                        }

                        if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                            incrementPathState();
                        }
                    }
                    break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(paths.SF_TM, true);
                    intake.run();
                    incrementPathState();
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    endOfIntake.run();
                    follower.followPath(paths.TM_SF, true);
                    incrementPathState();
                }
                break;
            case 5:
                if (holding) {
                        if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                            shoot.run();
                        }

                        if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                            incrementPathState();
                        }
                    }
                    break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(paths.SF_TH, true);
                    intake.run();
                    incrementPathState();
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    endOfIntake.run();
                    follower.followPath(paths.TH_SF, true);
                    incrementPathState();
                }
                break;
            case 8:
                if (holding) {
                        if (!actionRun && pathTimer.milliseconds() > SHOOT_WAIT_TIME) {
                            shoot.run();
                        }

                        if (pathTimer.milliseconds() > SHOOT_WAIT_TIME + SHOOTING_TIME) {
                            incrementPathState();
                        }
                    }
                    break;
            case 9:
                if(!follower.isBusy()) {
                    goToEnd.run();
                    follower.followPath(paths.SF_EF, true);
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

    public void incrementPathState(){
        pathState++;
        pathTimer.reset();
        holding = false;
        actionRun = false;

    }
}

