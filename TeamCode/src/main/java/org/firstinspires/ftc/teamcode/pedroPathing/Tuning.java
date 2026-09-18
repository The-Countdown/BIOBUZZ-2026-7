package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    // Tuners go here
    @Tuner // AutoTune: http://192.168.43.1:10158
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, PedroPathingConstants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, PedroPathingConstants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, PedroPathingConstants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, PedroPathingConstants.localizerConfig)), () -> new Foresight(PedroPathingConstants.foresightConfig));
    }
}
