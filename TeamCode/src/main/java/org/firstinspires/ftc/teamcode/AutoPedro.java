package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

/**
 * Pedro Pathing autonomous - closed-loop straight-line test.
 *
 * Drives a 48x24 inch box while holding heading 0 the whole time. Every leg is
 * a dead-straight line, and the Follower's heading feedback loop corrects any
 * curving in real time (that feedback is what keeps it straight - if the robot
 * gets bumped or one side of the drivetrain is stronger, it compensates).
 *
 * Units: positions are INCHES, headings are RADIANS (0 = facing +x, CCW+).
 *
 * This doubles as the template for a real match autonomous: replace the four
 * line() segments with your own paths. Curves work too, e.g.:
 *     Path swoop = curve(new Pose(0, 0, 0), new Pose(24, 0), new Pose(48, 24, Math.PI / 2)).tangent();
 * or a smooth path through several waypoints:
 *     Path route = through(new Pose(0, 0, 0), new Pose(24, 12, 0), new Pose(48, 0, 0));
 *
 * BEFORE THIS WILL DRIVE ACCURATELY, run the tuners (see pedro/Tuning.java):
 * Mecanum -> Pinpoint -> Foresight, then verify with the Line Test.
 */
@Autonomous(name = "Pedro Auto - Straight Box Test", group = "Pedro")
public class AutoPedro extends LinearOpMode {

    // Where the robot starts on the field (inches / radians). For this test
    // everything is relative to the start pose, so (0,0,0) is fine. For a real
    // match auto, set this to your alliance's actual starting coordinates.
    private static final Pose START_POSE = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {
        // Build the Follower: Pinpoint V2 localizer (2 pods + IMU sensor fusion)
        // + mecanum drivetrain + Foresight feedback algorithm.
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);

        // The box: forward 48", strafe right 24", back 48", strafe left 24".
        // .constant(0) locks the heading to 0 radians for the whole leg.
        Path forward   = line(new Pose(0, 0, 0),    new Pose(48, 0, 0)).constant(0);
        Path strafeOut = line(new Pose(48, 0, 0),   new Pose(48, -24, 0)).constant(0);
        Path back      = line(new Pose(48, -24, 0), new Pose(0, -24, 0)).constant(0);
        Path strafeHome= line(new Pose(0, -24, 0),  new Pose(0, 0, 0)).constant(0);

        telemetry.addLine("Pedro Auto ready - press Init, then Start");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // Re-zero the localizer to the start pose at the moment the auto begins
        follower.setPose(START_POSE);
        follower.update();

        followUntilDone(follower, forward,   "1/4 forward 48in");
        followUntilDone(follower, strafeOut, "2/4 strafe right 24in");
        followUntilDone(follower, back,      "3/4 back 48in");
        followUntilDone(follower, strafeHome,"4/4 strafe left 24in - home");

        // Actively hold the end pose briefly so the robot doesn't roll away
        follower.hold(START_POSE);
        ElapsedTime holdTimer = new ElapsedTime();
        while (opModeIsActive() && holdTimer.seconds() < 2.0) {
            follower.update();
            sendPoseTelemetry(follower, "holding end pose");
            telemetry.update();
        }

        // Safe stop - never leave motors under power when the OpMode ends
        follower.stop();
    }

    /** Follows one path to its end, feeding the follower and telemetry. */
    private void followUntilDone(Follower follower, Path path, String label) {
        if (!opModeIsActive()) return;
        follower.follow(path);
        while (opModeIsActive() && !follower.atParametricEnd()) {
            follower.update();
            sendPoseTelemetry(follower, label);
            telemetry.update();
        }
    }

    private void sendPoseTelemetry(Follower follower, String label) {
        Pose pose = follower.pose();
        telemetry.addData("Segment", label);
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
    }
}
