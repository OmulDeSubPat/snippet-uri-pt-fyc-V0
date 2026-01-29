package org.firstinspires.ftc.teamcode.pedroPathing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class AutoBlueClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState = 0;
    private boolean pathStarted = false;

    private Paths paths;
    private ElapsedTime pathTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(64.262, 8.000, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if (!pathStarted) {
                    follower.followPath(paths.Path1, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathTimer.reset();
                    pathState = 1;
                }
                break;

            case 1:
                if (pathTimer.seconds() >= 2.0) {
                    pathState = 2;
                }
                break;

            case 2:
                if (!pathStarted) {
                    follower.followPath(paths.Path2, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathTimer.reset();
                    pathState = 3;
                }
                break;

            case 3:
                if (pathTimer.seconds() >= 2.0) {
                    pathState = 4;
                }
                break;

            case 4:
                if (!pathStarted) {
                    follower.followPath(paths.Path3, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathTimer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                if (pathTimer.seconds() >= 2.0) {
                    pathState = 6;
                }
                break;

            case 6:
                if (!pathStarted) {
                    follower.followPath(paths.Path4, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathTimer.reset();
                    pathState = 7;
                }
                break;

            case 7:
                if (pathTimer.seconds() >= 2.0) {
                    pathState = 8;
                }
                break;

            case 8:
                if (!pathStarted) {
                    follower.followPath(paths.Path5, true);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 9;
                }
                break;

            case 9:
                // Autonomous finished
                break;
        }
    }

    public static class Paths {

        public PathChain Path1, Path2, Path3, Path4, Path5;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(64.262, 8.000),
                            new Pose(60.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(60.000, 60.000),
                            new Pose(36.492, 60.590)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(36.492, 60.590),
                            new Pose(28.623, 60.508)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(28.623, 60.508),
                            new Pose(10.787, 60.148)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.787, 60.148),
                            new Pose(64.623, 8.443)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }
}
