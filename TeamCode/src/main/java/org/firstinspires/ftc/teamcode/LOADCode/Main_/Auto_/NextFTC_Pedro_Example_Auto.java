//package org.firstinspires.ftc.teamcode.LOADCode.Main_.Auto_;
//
//import com.acmerobotics.roadrunner.path.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.FollowPath;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name = "blueBottom")
//public class NextFTC_Pedro_Example_Auto extends NextFTCOpMode {
//    private final Pose startPose = new Pose(85.5, 8.3, Math.toRadians(90.0));
//    private final Pose depositPose = new Pose(84.3, 61.9, Math.toRadians(0.0));
//    private final Pose curvePoint = new Pose(138.2, 48.1);
//
//    private PathChain skib;
//
//    public NextFTC_Pedro_Example_Auto() {
//        addComponents(
//                new PedroComponent(Constants::createFollower)
//        );
//    }
//
//    private void buildPaths() {
//        skib = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, depositPose))
//                .setLinearHeadingInterpolation(startPose.heading, depositPose.heading)
//                .build();
//    }
//
//    public Command getSecondRoutine() {
//        return new SequentialGroup(
//                new FollowPath(skib)
//        );
//    }
//
//    @Override
//    public void onInit() {
//        follower.setMaxPower(0.7);
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        getSecondRoutine().run();
//    }
//}