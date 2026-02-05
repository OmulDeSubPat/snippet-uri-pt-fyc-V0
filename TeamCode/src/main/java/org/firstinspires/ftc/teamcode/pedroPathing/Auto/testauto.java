package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Auto_Test", group = "Autonomous")
@Configurable
public class testauto extends OpMode {

    private TelemetryManager telemetryManager;
    private Follower follower;
    private Paths paths;

    private boolean pathStarted = false;

    @Override
    public void init() {
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Starting position you requested
        follower.setStartingPose(
                new Pose(23.148, 124.852, Math.toRadians(-36))
        );

        paths = new Paths(follower);
    }

    @Override
    public void loop() {
        follower.update();

        // Start the path once
        if (!pathStarted) {
            follower.followPath(paths.MainPath, 1.0, true);
            pathStarted = true;
        }

        Pose pose = follower.getPose();

        telemetryManager.debug("X", pose.getX());
        telemetryManager.debug("Y", pose.getY());
        telemetryManager.debug("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetryManager.debug("Busy", follower.isBusy());

        telemetryManager.update(telemetry);
    }

    /* ===================== PATH ===================== */
    public static class Paths {
        public PathChain MainPath;

        public Paths(Follower follower) {

            Pose start = new Pose(23.148, 124.852, Math.toRadians(-36));
            Pose end   = new Pose(41.04918032786885, 84.78688524590163, Math.toRadians(180));

            MainPath = follower.pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                    .build();
        }
    }
}
