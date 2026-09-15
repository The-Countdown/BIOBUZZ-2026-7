// Autonomous Package
package org.firstinspires.ftc.teamcode.opMode.auto;

// Autonomous Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Pedro Pathing Imports
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.follower.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;

// Panels Imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

// Our Imports
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@Autonomous
@Configurable // Allows you to configure in panels
public class ExampleAuto extends OpMode {

    // Objects that need initialising
        private Follower follower;
        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        private final PoseFactory poseFactory = PoseFactory.degrees(); // Makes the creation of poses easier
        private RobotContainer robotContainer; // Robot container sets up all necessary hardware and software so that it can be immediately used in any code across the codebase without having to set it up in every file

    // Poses
        private final Pose startPose = poseFactory.of(24, 24, 0);
        private final Pose startToScore = poseFactory.of(48, 48, 90);
        private final Pose park = poseFactory.of(72, 48, 90);

    // Action initialisation
        Runnable exampleAction;

    // Path methods
        private Path lineExample() {
            // Starts at one point and goes to the next in a straight line
            return line(startPose, startToScore).constant(startPose); // .constant means the heading will stay constant the whole time
        }
        private Path throughExample() {
            // Starts at one point and will pass through all the points on the path
            return through(startToScore, park, startPose).linear(startToScore, startPose); // .linear will make a linear interpolation of the two headings given to smoothly change throughout the path
        }
        private Path curveExample(){
            // Starts at one point and will travel to the final point curving towards all points in between
            return curve(startPose, startToScore, park).tangent(); // .tangent will make the robot follow the heading of the path that it is taking and ignore the headings of the points
        }

    private Command autoRoutineExample() {
        return sequential( // list of *commands* to run in a row when executed
                follow(follower, lineExample()),
                follow(follower, throughExample()),
                follow(follower, curveExample())
        );
    }

    @Override
    public void init() { // runs when the init is pressed on the driver station
        try { // this prevents it from crashing because of an issue with robot container
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry(); // creates a new instance of panels
        robotContainer.init();
        Scheduler.reset(); // resets the command scheduler for the ivy follower

        follower = Constants.create(hardwareMap); // TODO: Help idk why this is erroring ):
        follower.setPose(startPose); // Sets the starting pose for the robot
        follower.update();

        //Action Creation
            exampleAction = new Runnable() { // These are just some random ones that you could run just to show how you would put function calls in here
                @Override
                public void run() {
                    Status.flywheelToggle = true;
                    robotContainer.lever.close();
                    robotContainer.turret.hood.setPos(Constants.Turret.HOOD_PRESETS[0]);
                }
            };

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", Status.alliance);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() { // Runs when the start button is pressed on the driver station
        schedule(autoRoutineExample()); // This will schedule the auto routine that you had previously created to run in sequence
    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        Scheduler.execute(); // Checks which command should be running right now in the list of your commands that you configured in the start function

        robotContainer.update(false); // This will update all subsystems of the robot that need functions running in loop such as tracking the goal

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Turret Target", robotContainer.turret.getTargetPosition());
        panelsTelemetry.debug("Turret Target Degrees", robotContainer.turret.getTargetPositionDegrees());
        panelsTelemetry.debug("Path Timer", pathTimer.milliseconds());
        panelsTelemetry.debug("Holding", holding);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.pose().x());
        panelsTelemetry.debug("Y", follower.pose().y());
        panelsTelemetry.debug("Heading", follower.pose().heading());
        panelsTelemetry.debug("Current Pose", Status.currentPose);
        panelsTelemetry.update(telemetry);

        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}