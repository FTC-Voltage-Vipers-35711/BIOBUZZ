package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro Pathing AutoTune procedures. While connected to the Robot Controller,
 * open http://192.168.43.1:10158 and pick a procedure.
 *
 * Tuning order for this robot:
 *   1. Mecanum  - motor directions + drive characterization
 *   2. Pinpoint - odometry pod directions and offsets
 *   3. Foresight - path-following feedback loops
 *   4. Tests    - verify everything (Line Test = drives straight, no curving)
 */
public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
            h -> new PinpointLocalizer(h, Constants.localizerConfig),
            h -> new Mecanum(h, Constants.drivetrainConfig)
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
            h -> new Mecanum(h, Constants.drivetrainConfig),
            h -> new PinpointLocalizer(h, Constants.localizerConfig),
            () -> new Foresight(Constants.foresightConfig)
        );
    }
}
