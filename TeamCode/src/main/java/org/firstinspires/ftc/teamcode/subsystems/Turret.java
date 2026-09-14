package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.FlywheelPDF;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.hardware.LinkedMotors;
import org.firstinspires.ftc.teamcode.hardware.LinkedServos;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedMotors flyWheelMotors;
    private final LinkedServos turretServos;
    public final BetterServo hoodServo;
    private FlywheelPDF flywheelPDF;
    private double targetPosition;
    private double targetPositionDegrees;
    private double manualTurretPos;
    public double turretAngleOffset;
    public double turretAngleOffsetFar;
    private boolean manualHood;
    public double flywheelError;
    public double flywheelPowerModifier;
    private double goalAngle;
    public double goalOrientedVelocityX;
    public double goalOrientedVelocityY;
    private GamepadWrapper.ButtonReader backFieldButton = new GamepadWrapper.ButtonReader();

    public Turret(RobotContainer robotContainer, LinkedMotors flyWheelMotors, BetterServo hoodServo, LinkedServos turretServos) {
        this.robotContainer = robotContainer;
        this.flyWheelMotors = flyWheelMotors;
        this.hoodServo = hoodServo;
        this.turretServos = turretServos;
        this.flywheelPDF = new FlywheelPDF(robotContainer, flyWheelMotors);
        this.targetPosition = 0;
        this.manualTurretPos = 0;
        this.turretAngleOffset = 0;
        this.turretAngleOffsetFar = 0;
        this.flywheelError = 0;
        this.flywheelPowerModifier = 0;
        this.manualHood = false;
    }

    public void update(boolean teleop) {
        Status.flywheelToggleButton.update(Status.flywheelToggle);
        robotContainer.turret.hood.setPos(HelperFunctions.clamp(robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal(), false), Constants.Turret.HOOD_PRESETS[0], Constants.Turret.HOOD_PRESETS[Constants.Turret.HOOD_PRESETS.length-1]));

        if (Status.flywheelToggle) {
//            flywheel.targetVelocity = Math.min(Status.flywheelToggleButton.holdDuration() * Constants.Turret.FLYWHEEL_CURVE, robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal()));
            flywheel.targetVelocity = robotContainer.turret.flywheel.interpolateByDistance(HelperFunctions.disToGoal(), true) - (goalOrientedVelocityX * Constants.Turret.FLYWHEEL_POWER_VELOCITY_MULTIPLIER);
        } else {
            flywheel.targetVelocity = 0;
        }

        flywheel.targetVelocity = (flywheel.targetVelocity + flywheelPowerModifier) * Constants.Turret.FLYWHEEL_MAX_VELOCITY;
        if (flywheel.targetVelocity > 10) {
            flywheel.updateError();
        } else {
            flywheelError = 0;
        }
        Status.flywheelAtTargetSpeed = robotContainer.turret.flywheel.atTargetVelocity();
        double targetPower = flywheelPDF.calculate(flywheel.targetVelocity);

        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            goalAngle = 45;
        } else {
            goalAngle = -45;
        }

        goalOrientedVelocityX = (pinpoint.getVelX(DistanceUnit.INCH) * Math.cos(Math.toRadians(goalAngle))) - (pinpoint.getVelY(DistanceUnit.INCH) * Math.sin(Math.toRadians(goalAngle)));
        goalOrientedVelocityY = (pinpoint.getVelX(DistanceUnit.INCH) * Math.sin(Math.toRadians(goalAngle))) + (pinpoint.getVelY(DistanceUnit.INCH) * Math.cos(Math.toRadians(goalAngle)));

        Pose2D farGoalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Game.goalFarXBlue, Constants.Game.goalFarYBlue, AngleUnit.DEGREES, 45) : new Pose2D(DistanceUnit.INCH, Constants.Game.goalFarXRed, Constants.Game.goalFarYRed, AngleUnit.DEGREES, -45);
        Pose2D closeOpposingAllianceGoalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Game.goalOpposingXBlue, Constants.Game.goalOpposingYBlue, AngleUnit.DEGREES, 45) : new Pose2D(DistanceUnit.INCH, Constants.Game.goalOpposingXRed, Constants.Game.goalOpposingYRed, AngleUnit.DEGREES, -45);
        Pose2D normalGoalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Game.goalXBlue, Constants.Game.goalYBlue, AngleUnit.DEGREES, 45) : new Pose2D(DistanceUnit.INCH, Constants.Game.goalXRed, Constants.Game.goalYRed, AngleUnit.DEGREES, -45);
        Pose2D wallGoalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Game.goalWallXBlue, Constants.Game.goalWallYBlue, AngleUnit.DEGREES, 45) : new Pose2D(DistanceUnit.INCH, Constants.Game.goalWallXRed, Constants.Game.goalWallYRed, AngleUnit.DEGREES, -45);
        Pose2D closeGoalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? new Pose2D(DistanceUnit.INCH, Constants.Game.closeGoalXBlue, Constants.Game.closeGoalYBlue, AngleUnit.DEGREES, 45) : new Pose2D(DistanceUnit.INCH, Constants.Game.closeGoalXRed, Constants.Game.closeGoalYRed, AngleUnit.DEGREES, -45);

        // Aiming Differences based on quadrant
        Status.goalPose = normalGoalPose;


        if (Status.currentPose.getX(DistanceUnit.CM) < -50) { // Back
            Status.goalPose = farGoalPose;

            if (Status.currentPose.getY(DistanceUnit.CM) < 0) { // Back Red

            } else  if (Status.currentPose.getY(DistanceUnit.CM) > 0){ // Back Blue

            }
        } else if (Status.currentPose.getX(DistanceUnit.CM) > 50) { // Front
            if (Status.currentPose.getX(DistanceUnit.CM) > 100 ){ // Very Front
                if (Status.currentPose.getY(DistanceUnit.CM) < -50) { // Front Red
                    Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? closeOpposingAllianceGoalPose : closeGoalPose;
                } else  if (Status.currentPose.getY(DistanceUnit.CM) > 50){ // Front Blue
                    Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.RED ? closeOpposingAllianceGoalPose : closeGoalPose;
                } else {
                    Status.goalPose = wallGoalPose;
                }
            } else if (Status.currentPose.getY(DistanceUnit.CM) < -50) { // Front Red
                Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.BLUE ? closeOpposingAllianceGoalPose : normalGoalPose;
            } else  if (Status.currentPose.getY(DistanceUnit.CM) > 50){ // Front Blue
                Status.goalPose = Status.alliance == Constants.Game.ALLIANCE.RED ? closeOpposingAllianceGoalPose : normalGoalPose;
            }
        }

        if (teleop) {

            // This will either increase or decrease the flywheel power by small increments
            if (robotContainer.gamepadEx2.cross.wasJustPressed()){
                flywheelPowerModifier -= 0.02;
            }
            if (robotContainer.gamepadEx2.triangle.wasJustPressed()){
                flywheelPowerModifier += 0.02;
            }

            // Change this to change the Status.change degree whatever to rotate the robot pose, but this will need to be changed in the robot
            // So some though will be required
            if (robotContainer.gamepadEx2.dpadLeft.wasJustReleased()) {
                if (Status.currentPose.getX(DistanceUnit.CM) > -75) {
                    turretAngleOffset -= 2;
                } else {
                    turretAngleOffsetFar -= 2;
                }
            }

            if (robotContainer.gamepadEx2.dpadRight.wasJustReleased()) {
                if (Status.currentPose.getX(DistanceUnit.CM) > -75) {
                    turretAngleOffset += 1.5;
                } else {
                    turretAngleOffsetFar += 1.5;
                }
            }

            if (robotContainer.gamepadEx2.dpadDown.wasJustPressed()) {
                turretAngleOffset = 0;
                turretAngleOffsetFar = 0;
            }

            // Manual Turret Hood
            if (robotContainer.gamepadEx2.dpadUp.wasJustPressed()) {
                manualHood = true;
                double targetHoodPos = robotContainer.turret.hoodServo.getPosition() == Constants.Turret.HOOD_PRESETS[2]  ? Constants.Turret.HOOD_PRESETS[0] : Constants.Turret.HOOD_PRESETS[2];
                robotContainer.turret.hood.setPos(targetHoodPos);
            }

            if (robotContainer.gamepadEx2.dpadDown.isHeldFor(1.0)){
                manualHood = false;
            }

            if (robotContainer.gamepadEx1.leftBumper.wasJustPressed() || robotContainer.gamepadEx2.leftBumper.wasJustPressed()) {
                Status.flywheelToggle = !Status.flywheelToggle;
            }

            // Automatic turret hood
//            if (!manualHood){
//                if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
//                    robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
//                } else if ((Status.currentPose.getX(DistanceUnit.CM) < 20 || Status.currentPose.getY(DistanceUnit.CM) < 20) && Status.alliance == Constants.Game.ALLIANCE.BLUE) {
//                    robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
//                } else if ((Status.currentPose.getX(DistanceUnit.CM) < 20 || Status.currentPose.getY(DistanceUnit.CM) > -20) && Status.alliance == Constants.Game.ALLIANCE.RED) {
//                    robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
//                } else {
//                    robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);
//                }
//            }

//            backFieldButton.update(Status.currentPose.getX(DistanceUnit.CM) < -20);
//
//            if (backFieldButton.wasJustPressed()) {
//                if (Status.alliance == Constants.Game.ALLIANCE.RED) {
//                    Status.goalPose = new Pose2D(DistanceUnit.INCH, Constants.Game.goalFarX, Constants.Game.goalFarY, AngleUnit.DEGREES, -45);
//                } else {
//                    Status.goalPose = new Pose2D(DistanceUnit.INCH, -Constants.Game.goalFarX, Constants.Game.goalFarY, AngleUnit.DEGREES, 45);
//                }
//            } else if (backFieldButton.wasJustReleased()) {
//                if (Status.alliance == Constants.Game.ALLIANCE.RED) {
//                    Status.goalPose = new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45);
//                } else {
//                    Status.goalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, 45);
//                }
//            }

            // Turret turn - Right stick X
             if (Status.manualControl && robotContainer.gamepadEx2.rightStickX() != 0) {
                 // Manual turret turning
                 manualTurretPos -= robotContainer.gamepadEx2.rightStickX() != 0 ? (Constants.Turret.SPEED_FACTOR * robotContainer.CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.gamepadEx2.rightStickX(), 3) : 0;
                 manualTurretPos = HelperFunctions.clamp(manualTurretPos, Constants.Turret.MIN_SERVO, Constants.Turret.MAX_SERVO);
                 robotContainer.turret.setTargetPosition(manualTurretPos);
             } else if (Status.manualControl) {
             } else {
                 robotContainer.turret.pointAtGoal();
             }

        } else {
            robotContainer.turret.pointAtGoal();

//            if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
//                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[2]);
//            } else if ((Status.currentPose.getX(DistanceUnit.CM) < 20 || Status.currentPose.getY(DistanceUnit.CM) < 20) && Status.alliance == Constants.Game.ALLIANCE.BLUE) {
//                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
//            } else if ((Status.currentPose.getX(DistanceUnit.CM) < 20 || Status.currentPose.getY(DistanceUnit.CM) > -20) && Status.alliance == Constants.Game.ALLIANCE.RED) {
//                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[1]);
//            } else {
//                robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);
//            }
        }

        // Only run flywheel if good voltage
        if (robotContainer.controlHubVoltage > 8) {
            flyWheelMotors.setPower(targetPower);
        }

    }

    // This is for manual control
    public void setTargetPosition(double position) {
        targetPosition = position;
        turretServos.setPosition(HelperFunctions.clamp(position, Constants.Turret.MIN_SERVO, Constants.Turret.MAX_SERVO));
    }

    public void setFlywheelPowerModifier(double val){
        flywheelPowerModifier = val;
    }

    public double getTargetPosition(){ return targetPosition;}

    public double getTargetPositionDegrees(){return targetPositionDegrees;}

    public void setTargetAngle(double angleInDegrees) {
        targetPositionDegrees = angleInDegrees;
        if (Status.currentPose.getX(DistanceUnit.CM) < -75) {
            angleInDegrees = HelperFunctions.normalizeAngle(angleInDegrees + turretAngleOffsetFar);
        } else {
            angleInDegrees = HelperFunctions.normalizeAngle(angleInDegrees + turretAngleOffset);
        }

        double currentAngle = servoToAngle(turretServos.getPosition());
        double targetAngle = angleInDegrees;

        double delta = HelperFunctions.normalizeAngle(targetAngle - currentAngle);

        double proposedAngle = currentAngle + delta;

        //check mechanical limits
        if (proposedAngle < Constants.Turret.MIN_ANGLE || proposedAngle > Constants.Turret.MAX_ANGLE) {
            // take the long way
            delta = targetAngle - currentAngle;  // no normalization
            proposedAngle = currentAngle + delta;
        }

         double position = HelperFunctions.clamp(angleToServo(proposedAngle), Constants.Turret.MIN_SERVO, Constants.Turret.MAX_SERVO);
        turretServoLeader.updateSetPosition(position - Constants.Turret.PRESSURE_OFFSET);
        turretServoFollower.updateSetPosition(position + Constants.Turret.PRESSURE_OFFSET);
//        turretServos.setPosition(0.5);
    }

    /**
     * Interpolates a value from one table given a key from a parallel table.
     * Uses the same binary-search-style nearest-neighbor interpolation as getPoseToAim.
     *
     * @param keyTable   The table to search (e.g. angles or servo positions)
     * @param valueTable The parallel table to interpolate from
     * @param key        The input value to look up
     * @return The interpolated output value
     */
    private double tableInterpolate(double[] keyTable, double[] valueTable, double key) {
        int lowerIndex = 0;
        int higherIndex = keyTable.length - 1;
        double lowerKey = keyTable[lowerIndex];
        double higherKey = keyTable[higherIndex];

        for (int i = keyTable.length - 2; i > 0; i--) {
            double current = keyTable[i];
            if (current > key) {
                if (current < higherKey) {
                    higherKey = current;
                    higherIndex = i;
                }
            } else if (current < key) {
                if (current >= lowerKey) {
                    lowerKey = current;
                    lowerIndex = i;
                }
            } else {
                // Exact match
                return valueTable[i];
            }
        }

        double lowerValue = valueTable[lowerIndex];
        double higherValue = valueTable[higherIndex];

        if (higherKey == lowerKey) {
            return lowerValue;
        }

        return lowerValue + ((higherValue - lowerValue) * (key - lowerKey) / (higherKey - lowerKey));
    }

    private double angleToServo(double angle) {
        return tableInterpolate(
                Constants.Turret.TURRET_INTERPOLATION_ANGLES,
                Constants.Turret.TURRET_INTERPOLATION_POSITIONS,
                angle
        );
    }

    private double servoToAngle(double servo) {
        return tableInterpolate(
                Constants.Turret.TURRET_INTERPOLATION_POSITIONS,
                Constants.Turret.TURRET_INTERPOLATION_ANGLES,
                servo
        );
    }

    public double getPosition() {
        return turretServos.getPosition();
    }

    public double getPositionDegrees() {
        return targetPositionDegrees;
    }

    public void pointAtGoal() {
        if (Status.turretFaceBack) {
            setTargetAngle(0);
            return;
        }

        adjustByPosition(!Constants.Turret.ADJUST_TURRET_BY_MULTIPLIER);

        double xDiff;
        double yDiff;
        if (Status.alliance == Constants.Game.ALLIANCE.RED) {
            xDiff = Status.goalPose.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
            yDiff = Status.goalPose.getY(DistanceUnit.INCH) + Status.currentPose.getY(DistanceUnit.INCH);
        } else {
            xDiff = Status.goalPose.getX(DistanceUnit.INCH) + Status.currentPose.getX(DistanceUnit.INCH);
            yDiff = Status.goalPose.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
        }
        double angleToFaceGoal = Constants.Turret.ADJUST_TURRET_BY_MULTIPLIER ? adjustByMultiplier((Math.atan(yDiff/xDiff) * (180 / Math.PI)) + Status.currentHeading - 180) : (Math.atan(yDiff/xDiff) * (180 / Math.PI)) + Status.currentHeading - 180;
        setTargetAngle(HelperFunctions.normalizeAngle(angleToFaceGoal));

    }

    public double adjustByMultiplier(double angleToFaceGoal){
        return angleToFaceGoal - (goalOrientedVelocityY * (160/HelperFunctions.disToGoal()) * Constants.Turret.TURRET_ANGLE_VELOCITY_MULTIPLIER);
    }

    public void adjustByPosition(boolean doIt){
        if (doIt) Status.goalPose = new Pose2D(DistanceUnit.INCH, Status.goalPose.getX(DistanceUnit.INCH) + goalOrientedVelocityX * Constants.Turret.GOAL_POSITION_VELOCITY_MULTIPLIER, Status.goalPose.getY(DistanceUnit.INCH) + goalOrientedVelocityY * Constants.Turret.GOAL_POSITION_VELOCITY_MULTIPLIER, AngleUnit.DEGREES, Status.goalPose.getHeading(AngleUnit.DEGREES));
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - targetPosition) < 10;
    }

    public class Flywheel {
        public double targetVelocity = 0;
        public double targetMaxVelocity = 0;

        public double interpolateByDistance(double disToGoal, boolean flywheel){
            if (flywheel) {
                double lowerPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[0];
                int lowerPointIndex = 0;
                double higherPoint = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-1];
                int higherPointIndex = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-1;
                double currentDistance;
                for (int i = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES.length-2; i>0;i--){
                    currentDistance = Constants.Turret.FLYWHEEL_SPEED_TABLE_DISTANCES[i];
                    if(currentDistance > disToGoal){
                        if (currentDistance < higherPoint) {
                            higherPoint = currentDistance;
                            higherPointIndex = i;
                        }
                    }else if (currentDistance < disToGoal){
                        if (currentDistance >= lowerPoint) {
                            lowerPoint = currentDistance;
                            lowerPointIndex = i;
                        }
                    }else if (currentDistance == disToGoal){
                        return Constants.Turret.FLYWHEEL_SPEED_TABLE[i];
                    }
                }
                double lowerSpeed = Constants.Turret.FLYWHEEL_SPEED_TABLE[lowerPointIndex];
                double higherSpeed = Constants.Turret.FLYWHEEL_SPEED_TABLE[higherPointIndex];

                return HelperFunctions.interpolate(lowerSpeed, higherSpeed, (disToGoal-lowerPoint)/(higherPoint-lowerPoint));
            } else {
                double lowerPoint = Constants.Turret.HOOD_TABLE_DISTANCES[0];
                int lowerPointIndex = 0;
                double higherPoint = Constants.Turret.HOOD_TABLE_DISTANCES[Constants.Turret.HOOD_TABLE_DISTANCES.length-1];
                int higherPointIndex = Constants.Turret.HOOD_TABLE_DISTANCES.length-1;
                double currentDistance;

                if (disToGoal > higherPoint){
                    return Constants.Turret.HOOD_PRESETS[higherPointIndex];
                } else if (disToGoal < lowerPoint) {
                    return Constants.Turret.HOOD_PRESETS[lowerPointIndex];
                }

                for (int i = Constants.Turret.HOOD_TABLE_DISTANCES.length-2; i>0;i--){
                    currentDistance = Constants.Turret.HOOD_TABLE_DISTANCES[i];
                    if(currentDistance > disToGoal){
                        if (currentDistance < higherPoint) {
                            higherPoint = currentDistance;
                            higherPointIndex = i;
                        }
                    }else if (currentDistance < disToGoal){
                        if (currentDistance >= lowerPoint) {
                            lowerPoint = currentDistance;
                            lowerPointIndex = i;
                        }
                    }else if (currentDistance == disToGoal){
                        return Constants.Turret.HOOD_PRESETS[i];
                    }
                }
                double lowerPos = Constants.Turret.HOOD_PRESETS[lowerPointIndex];
                double higherPos = Constants.Turret.HOOD_PRESETS[higherPointIndex];

                return HelperFunctions.interpolate(lowerPos, higherPos, (disToGoal-lowerPoint)/(higherPoint-lowerPoint));
            }
        }

        public void updateLaunchValues(double distToGoal){
            double verticalVel = Math.sqrt(2 * Constants.Game.GRAVITY * (Constants.Turret.DESIRED_MAX_HEIGHT - Constants.Turret.FLYWHEEL_HEIGHT));
            double estimatedTime = (verticalVel + Math.sqrt(verticalVel - 2 * (Constants.Game.GRAVITY) * (Constants.Game.GOAL_HEIGHT - Constants.Turret.FLYWHEEL_HEIGHT))) / Constants.Game.GRAVITY;
            double horizonalVel = distToGoal / estimatedTime;
            double hoodAngle = Math.atan(verticalVel / horizonalVel);
            double xDiff;
            double yDiff;
            if (Status.alliance == Constants.Game.ALLIANCE.BLUE){
                 xDiff = -Status.goalPose.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
                 yDiff = Status.goalPose.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
            } else {
                 xDiff = Status.goalPose.getX(DistanceUnit.INCH) - Status.currentPose.getX(DistanceUnit.INCH);
                 yDiff = -Status.goalPose.getY(DistanceUnit.INCH) - Status.currentPose.getY(DistanceUnit.INCH);
            }
            double angleToFaceGoal = Math.atan2(yDiff, xDiff);
            double initialVel = Math.sqrt(Math.pow((verticalVel), 2) * Math.pow((horizonalVel), 2));
            double rpm = initialVel * (2000 / 5.14);
            targetVelocity = (rpm / 60) * Constants.Robot.MOTOR_TICKS_PER_REVOLUTION;
        }

        public boolean atTargetVelocity() {
            return Math.abs(flywheelError) < 60 && targetVelocity > 10;
        }

        public void updateError() {
            flywheelError = targetVelocity - flyWheelMotors.getVelocity();
        }

        public double getFlywheelVelocity() {
            return flyWheelMotors.getAverageVelocity();
        }

        public double getCurrent(){
            return flyWheelMotors.getCurrent(CurrentUnit.AMPS);
        }
    }

    public class Hood {
        public void setPos(double pos) {
            hoodServo.updateSetPosition(pos);
        }
    }

    public final Flywheel flywheel = new Flywheel();
    public final Hood hood = new Hood();
}
