package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "closeblue", group = "Autonomous")
@Configurable
public class AutoBlueClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private boolean pathStarted = false;
    private Paths paths;

    // START (from your message)
    private static final Pose START_POSE = new Pose(22.151, 126.570, Math.toRadians(145));

    // TARGET (from your message)
    private static final Pose TARGET_POSE = new Pose(24.382, 125.529);

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();

        // Run the single path once
        if (!pathStarted) {
            follower.followPath(paths.Path1, true);
            pathStarted = true;
        }

        // Telemetry
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            START_POSE,
                            TARGET_POSE
                    ))
                    // Keep heading constant at your starting heading
                    // (If you want it to face somewhere else at the end, tell me the target heading.)
                    .setConstantHeadingInterpolation(START_POSE.getHeading())
                    .build();
        }
    }
}
