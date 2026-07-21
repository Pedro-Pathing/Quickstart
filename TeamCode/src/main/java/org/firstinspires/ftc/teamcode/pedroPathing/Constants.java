package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Pinpoint;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    static MecanumConfig mecanumConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("lf");
                c.backLeftName.set("lb");
                c.frontRightName.set("rf");
                c.backRightName.set("rb");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );

    static PinpointConfig pinpointConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.brakeController.set(Controller.pid(.2,0,0));
                c.headingController.set(Controller.pid(2, 0, 0.01));
                c.linearBrakeCoefficients.set(Matrix.diag(0.0788, 0.0788));
                c.quadraticBrakeCoefficients.set(Matrix.diag(.00191035, .00191035));
                c.maxAchievableForwardVelocity.set(81.175);
                c.maxAchievableStrafeVelocity.set(66.8431);
                c.maxAchievableForwardDeceleration.set(30.3333);
                c.maxAchievableStrafeDeceleration.set(62.58098);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new Pinpoint(h, pinpointConfig), new Mecanum(h, mecanumConfig), new Foresight(foresightConfig));
    }
}
