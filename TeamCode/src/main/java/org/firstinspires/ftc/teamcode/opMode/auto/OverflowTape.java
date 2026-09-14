
package org.firstinspires.ftc.teamcode.opMode.auto;

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


@Autonomous(name = "OverflowTape", group = "Autonomous")
@Configurable // Panels
    public class OverflowTape extends OpMode {
        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        public Follower follower; // Pedro Pathing follower instance
        private RobotContainer robotContainer;
        private int pathState; // Current autonomous path state (state machine)
        private Paths paths; // Paths defined in the Paths class
        private boolean holding;
        private boolean actionRun;
        private ElapsedTime pathTimer = new ElapsedTime(); // Path timer
        Runnable start;
        Runnable shoot;
        Runnable goToEnd;
        Runnable intake;
        Runnable endOfIntake;

        double startingX = 57;
        double startingY = 7.75;
        double startingHeading = 180;
        double startingPoseHeading = startingHeading -90;
        double START_WAIT_TIME = 750;
        double SHOOT_WAIT_TIME = 600;
        double SHOOTING_TIME = 1200;

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
            if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
                follower.setStartingPose(new Pose(startingX, startingY, Math.toRadians(startingHeading)));
                RobotContainer.HardwareDevices.betterIMU.setAngle(HelperFunctions.normalizeAngle(startingPoseHeading));
            } else {
                follower.setStartingPose(HelperFunctions.mirror(new Pose(startingX, startingY, Math.toRadians(HelperFunctions.mirrorAngle0(startingHeading)))));
                RobotContainer.HardwareDevices.betterIMU.setAngle(HelperFunctions.normalizeAngle(HelperFunctions.mirrorAngle0(startingPoseHeading)));
            }

            paths = new Paths(follower); // Build paths
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
                    robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.setPower(1.0), 200);
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

    public static class Paths {
        public PathChain BF_TL;
        public PathChain TL_SF;
        public PathChain SF_HW;
        public PathChain HW_SF;
        public PathChain SF_HS_1;
        public PathChain HS_SF_1;
        public PathChain SF_HS_2;
        public PathChain HS_SF_2;
        public PathChain SF_HS_3;
        public PathChain HS_SF_3;
        public PathChain SF_EF;

        public Pose
            // BLUE
            BLUE_BEGINNING_FAR = new Pose(56.000, 8.000),
            BLUE_TAPE_LOW = HelperFunctions.mirror(new Pose(126.000, 35.000, HelperFunctions.mirrorAngle90(Math.toRadians(0), false))),
            BLUE_HUMAN_SHIFT_1 = new Pose(18.000, 8.000),
            BLUE_HUMAN_SHIFT_2 = new Pose(14.000, 12.000),
            BLUE_HUMAN_SHIFT_3 = new Pose(14.000, 16.000),
            BLUE_HUMAN_WALL = new Pose(14.500, 8.000),
            BLUE_SHOOTING_FAR = new Pose(56.000, 12.000),
            BLUE_END_FAR = new Pose(36.000, 13.000),

            // RED
            RED_BEGINNING_FAR = HelperFunctions.mirror(BLUE_BEGINNING_FAR),
            RED_TAPE_LOW = new Pose(126.000, 35.000, Math.toRadians(0)),
            RED_HUMAN_SHIFT_1 = HelperFunctions.mirror(BLUE_HUMAN_SHIFT_1),
            RED_HUMAN_SHIFT_2 = HelperFunctions.mirror(BLUE_HUMAN_SHIFT_2),
            RED_HUMAN_SHIFT_3 = HelperFunctions.mirror(BLUE_HUMAN_SHIFT_3),
            RED_HUMAN_WALL = HelperFunctions.mirror(BLUE_HUMAN_WALL),
            RED_SHOOTING_FAR = HelperFunctions.mirror(BLUE_SHOOTING_FAR),
            RED_END_FAR = HelperFunctions.mirror(BLUE_END_FAR);

        public Paths(Follower follower) {
            if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
                BF_TL = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_BEGINNING_FAR,
                                        new Pose (42, 50),
                                        BLUE_TAPE_LOW
                                )
                        )
                        .setConstantHeadingInterpolation(HelperFunctions.mirrorAngle90(Math.toRadians(0), false))
                        .build();

                TL_SF = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_TAPE_LOW,
                                        new Pose (42, 8),
                                        BLUE_SHOOTING_FAR
                                )
                        )
                        .setConstantHeadingInterpolation(HelperFunctions.mirrorAngle90(Math.toRadians(0), false))
                        .build();

                SF_HW = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        BLUE_SHOOTING_FAR,
                                        BLUE_HUMAN_WALL
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();


                HW_SF = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_HUMAN_WALL,
                                        new Pose(36.000, 9.000),
                                        BLUE_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_HS_1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_FAR,
                                        new Pose(25.000, 9.000),
                                        BLUE_HUMAN_SHIFT_1
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HS_SF_1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_HUMAN_SHIFT_1,
                                        new Pose(35.000, 10.000),
                                        BLUE_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_HS_2 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_FAR,
                                        new Pose(25.000, 13.000),
                                        BLUE_HUMAN_SHIFT_2
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HS_SF_2 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_HUMAN_SHIFT_2,
                                        new Pose(35.000, 10.000),
                                        BLUE_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_HS_3 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_SHOOTING_FAR,
                                        new Pose(25.000, 17.000),
                                        BLUE_HUMAN_SHIFT_3
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HS_SF_3 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        BLUE_HUMAN_SHIFT_3,
                                        new Pose(35.000, 10.000),
                                        BLUE_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_EF = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        BLUE_SHOOTING_FAR,
                                        BLUE_END_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();
            } else {
                BF_TL = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_BEGINNING_FAR,
                                        HelperFunctions.mirror(new Pose (42, 50)),
                                        RED_TAPE_LOW
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                TL_SF = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_TAPE_LOW,
                                        HelperFunctions.mirror(new Pose (42, 8)),
                                        RED_SHOOTING_FAR
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build();

                SF_HW = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        RED_SHOOTING_FAR,
                                        RED_HUMAN_WALL
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HW_SF = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_HUMAN_WALL,
                                        HelperFunctions.mirror(new Pose(36.000, 9.000)),
                                        RED_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_HS_1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_SHOOTING_FAR,
                                        HelperFunctions.mirror(new Pose(25.000, 9.000)),
                                        RED_HUMAN_SHIFT_1
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HS_SF_1 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_HUMAN_SHIFT_1,
                                        HelperFunctions.mirror(new Pose(35.000, 10.000)),
                                        RED_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_HS_2 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_SHOOTING_FAR,
                                        HelperFunctions.mirror(new Pose(25.000, 13.000)),
                                        RED_HUMAN_SHIFT_2
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HS_SF_2 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_HUMAN_SHIFT_2,
                                        HelperFunctions.mirror(new Pose(35.000, 10.000)),
                                        RED_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_HS_3 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_SHOOTING_FAR,
                                        HelperFunctions.mirror(new Pose(25.000, 17.000)),
                                        RED_HUMAN_SHIFT_3
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build();

                HS_SF_3 = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        RED_HUMAN_SHIFT_3,
                                        HelperFunctions.mirror(new Pose(35.000, 10.000)),
                                        RED_SHOOTING_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

                SF_EF = follower.pathBuilder()
                        .addPath(
                                new BezierLine(
                                        RED_SHOOTING_FAR,
                                        RED_END_FAR
                                )
                        )
                        .setTangentHeadingInterpolation()
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
                    if (!actionRun && pathTimer.milliseconds() > START_WAIT_TIME){
                        shoot.run();
                        actionRun = true;
                        pathTimer.reset();
                    }

                    if (pathTimer.milliseconds() > SHOOTING_TIME && actionRun) {
                        follower.followPath(paths.BF_TL, false);
                        incrementPathState();
                    }
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.TL_SF, true);
                        endOfIntake.run();
                        incrementPathState();
                    }
                    break;
                case 2:
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
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.SF_HW, true);
                        endOfIntake.run();
                        incrementPathState();
                    }
                    break;
                case 4:
                    if (!follower.isBusy() && holding && pathTimer.milliseconds() > 300) {
                        follower.followPath(paths.HW_SF, true);
                        endOfIntake.run();
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
                    if (!follower.isBusy()){
                        intake.run();
                        follower.followPath(paths.SF_HS_1, true);
                        incrementPathState();
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.HS_SF_1, true);
                        endOfIntake.run();
                        incrementPathState();
                    }
                    break;
                case 8:
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
                case 9:
                    if (!follower.isBusy()){
                        intake.run();
                        follower.followPath(paths.SF_HS_2, true);
                        incrementPathState();
                    }
                    break;
                case 10:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.HS_SF_2, true);
                        endOfIntake.run();
                        incrementPathState();
                    }
                    break;
                case 11:
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
                case 12:
                    if (!follower.isBusy()){
                        intake.run();
                        follower.followPath(paths.SF_HS_3, true);
                        incrementPathState();
                    }
                    break;
                case 13:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.HS_SF_3, true);
                        endOfIntake.run();
                        incrementPathState();
                    }
                    break;
                case 14:
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
                case 15:
                    if (!follower.isBusy()){
                        goToEnd.run();
                        follower.followPath(paths.SF_EF);
                        incrementPathState();
                    }
                    break;
                default:
                    break;
            }
        }

        /**
         * These change the states of the paths and actions. It will also reset the timers of the individual switches
         **/
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

