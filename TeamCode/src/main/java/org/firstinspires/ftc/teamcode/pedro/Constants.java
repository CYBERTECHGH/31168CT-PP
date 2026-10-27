package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.1760822769225113);
        c.yPodOffset.set(-3.1554109468234803);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4380463576284356);
                Controller secondaryTranslationalForward = Controller.proportional(0.16184642432946045);
                Controller primaryTranslationalLateral = Controller.proportional(0.6046538370877247);
                Controller secondaryTranslationalLateral = Controller.proportional(0.22340343615582603);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010947208627268003));
                c.brake.set(Controller.proportionalFeedforward(0.009305127333177803));

                c.headingFeedback.set(Controller.proportional(5.973420619643218));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04745629664139802, 0.009049160557725934));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09862288363108732, 0.10105313444749628));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001961854776052697, 0.0013175546186955));

                c.maxAchievableForwardVelocity.set(83.4087667906677);
                c.maxAchievableStrafeVelocity.set(69.14985681991661);
                c.naturalForwardDeceleration.set(42.02763965936379);
                c.naturalStrafeDeceleration.set(72.58698130152513);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}