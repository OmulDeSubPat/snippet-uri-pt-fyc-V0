package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// Assuming Constants.java is in the same package
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "DECODE Auto with 3 Intakes", group = "Pedro Pathing")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private int pathState = 0;
    private int currentPathIndex = 0;
    private List<Path> paths = new ArrayList<>();

    // Adjust these poses based on DECODE field coordinates (inches).
    // Assumes Red Alliance: Origin (0,0) at field center.
    // Start: Near red loading zone, facing +Y (towards field center).
    // Score: At red goal, facing -Y (backing into goal for scoring).
    // Intakes: Approximate spike mark positions in midfield for artifact pickup.
    // Headings: 0 = +X (right), π/2 = +Y (up), π = -X (left), 3π/2 = -Y (down).
    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));  // Starting position
    private final Pose scorePose = new Pose(0, 20, Math.toRadians(90)); // Scoring position at goal

    private final Pose intake1Pose = new Pose(-28, 24, Math.toRadians(-113));  // Intake 1 (left spike)
    private final Pose intake2Pose = new Pose(-46, 35, Math.toRadians(-118));    // Intake 2 (center spike)
    private final Pose intake3Pose = new Pose(-68, 44, Math.toRadians(-120));
    private final Pose push1Pose = new Pose(-34, 15, Math.toRadians(-113));
    private final Pose push2Pose = new Pose(-52, 25, Math.toRadians(-113));
    private final Pose push3Pose = new Pose(-75, 35, Math.toRadians(-113));
    // Intake 3 (right spike)

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize follower using Constants class (must be configured for your robot!)
        follower = Constants.createFollower(hardwareMap);

        // Build the sequence of paths
        buildPaths();

        // Set the robot's starting pose (critical for accurate pathing)
        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        // Path 0: Start -> Score (initial preload score)
        paths.add(new Path(new BezierLine(startPose, scorePose)));
        paths.get(0).setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Path 1: Score -> Intake 1
        paths.add(new Path(new BezierLine(scorePose, intake1Pose)));
        paths.get(1).setLinearHeadingInterpolation(scorePose.getHeading(), intake1Pose.getHeading());

        // Path 2: Intake 1 -> Push 1
        paths.add(new Path(new BezierLine(intake1Pose, push1Pose)));
        paths.get(2).setLinearHeadingInterpolation(intake1Pose.getHeading(), push1Pose.getHeading());

        // Path 3: Push 1 -> Score
        paths.add(new Path(new BezierLine(push1Pose, scorePose)));
        paths.get(3).setLinearHeadingInterpolation(push1Pose.getHeading(), scorePose.getHeading());

        // Path 4: Score -> Intake 2
        paths.add(new Path(new BezierLine(scorePose, intake2Pose)));
        paths.get(4).setLinearHeadingInterpolation(scorePose.getHeading(), intake2Pose.getHeading());

        // Path 5: Intake 2 -> Push 2
        paths.add(new Path(new BezierLine(intake2Pose, push2Pose)));
        paths.get(5).setLinearHeadingInterpolation(intake2Pose.getHeading(), push2Pose.getHeading());

        // Path 6: Push 2 -> Score
        paths.add(new Path(new BezierLine(push2Pose, scorePose)));
        paths.get(6).setLinearHeadingInterpolation(push2Pose.getHeading(), scorePose.getHeading());

        // Path 7: Score -> Intake 3
        paths.add(new Path(new BezierLine(scorePose, intake3Pose)));
        paths.get(7).setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading());

        // Path 8: Intake 3 -> Push 3
        paths.add(new Path(new BezierLine(intake3Pose, push3Pose)));
        paths.get(8).setLinearHeadingInterpolation(intake3Pose.getHeading(), push3Pose.getHeading());

        // Path 9: Push 3 -> Score (final score)
        paths.add(new Path(new BezierLine(push3Pose, scorePose)));
        paths.get(9).setLinearHeadingInterpolation(push3Pose.getHeading(), scorePose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:  // Follow current path
                if (currentPathIndex < paths.size()) {
                    follower.followPath(paths.get(currentPathIndex));
                    pathState = 1;  // Move to waiting state
                } else {
                    pathState = -1;  // All paths complete
                }
                break;
            case 1:  // Wait for current path to complete
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    pathState = 2;  // Move to sleep state
                }
                break;
            case 2:  // Wait 1.5 seconds after path completion
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    currentPathIndex++;  // Advance to next path
                    pathState = 0;  // Loop back to follow next
                }
                break;
        }
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathState = 0;
        currentPathIndex = 0;
    }

    @Override
    public void loop() {
        // Update follower (updates localization, applies powers to motors)
        follower.update();

        // Run the path state machine
        autonomousPathUpdate();

        // Telemetry for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("current path index", currentPathIndex);
        telemetry.addData("total paths", paths.size());
        telemetry.addData("pose x (in)", follower.getPose().getX());
        telemetry.addData("pose y (in)", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("is busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        // No explicit stop needed; follower disables automatically when opmode ends
    }
}