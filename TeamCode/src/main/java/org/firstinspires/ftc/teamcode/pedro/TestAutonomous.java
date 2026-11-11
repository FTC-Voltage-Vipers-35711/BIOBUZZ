package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Two paths from the Pedro Pathing Visualizer:
 *   Path 1: start (56, 8)        -> middle (20.08, 43.31), heading 90 -> 90
 *   Path 2: middle (20.08, 43.31) -> end (14.08, 95.46),  heading 90 -> 90
 *
 * Drive motors and the Pinpoint ("pinpoint") are configured in Constants.java.
 */
@Autonomous(name = "Test Autonomous", group = "Autonomous")
public class TestAutonomous extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Safety timeout for each path, in seconds.
    private static final double PATH_TIMEOUT_SECONDS = 5.0;

    public enum PathState {
        DRIVE_STARTPOS_MIDPOS,
        WAIT_FOR_PATH_1,
        DRIVE_MIDPOS_ENDPOS,
        WAIT_FOR_PATH_2,
        DONE
    }

    private PathState pathState;

    // ---------- Poses (scaled to 40% of the visualizer distances) ----------
    // TEMPORARY FIX: each point = start + 0.4 * (visualizer point - start).
    // Visualizer originals: mid (20.08, 43.31), end (14.08, 95.46).
    // Once localization is fixed in Constants.java, switch back to the originals.
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose midPose = new Pose(51, 12, Math.toRadians(90));
    private final Pose endPose = new Pose(48, 18, Math.toRadians(90));

    private PathChain driveStartToMid, driveMidToEnd;

    public void buildPaths() {
        driveStartToMid = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .build();

        driveMidToEnd = follower.pathBuilder()
                .addPath(new BezierLine(midPose, endPose))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_MIDPOS:
                follower.followPath(driveStartToMid, true);
                setPathState(PathState.WAIT_FOR_PATH_1);
                break;

            case WAIT_FOR_PATH_1:
                if (!follower.isBusy()
                        || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    setPathState(PathState.DRIVE_MIDPOS_ENDPOS);
                }
                break;

            case DRIVE_MIDPOS_ENDPOS:
                follower.followPath(driveMidToEnd, true);
                setPathState(PathState.WAIT_FOR_PATH_2);
                break;

            case WAIT_FOR_PATH_2:
                if (!follower.isBusy()
                        || pathTimer.getElapsedTimeSeconds() > PATH_TIMEOUT_SECONDS) {
                    follower.breakFollowing();
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Autonomous finished; robot stays stopped.
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_MIDPOS);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("state time (s)", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("opmode time (s)", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}
