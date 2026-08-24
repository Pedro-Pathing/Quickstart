package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayDeque;
import java.util.ArrayList;

/**
 * This is the StrafeVelocityIdentification autonomous follower OpMode. This runs the robot left at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with ForwardVelocityIdentification, allows  to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@TeleOp(group = "2")
public class StrafeVelocityIdentification extends OpMode {
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private boolean end;
    private Follower follower;
    private final Pose startPose = Pose.zero();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    /** This initializes the drive motors as well as the cache of velocities. */
    @Override
    public void init_loop() {
        telemetry.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the left.");
        telemetry.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetry.addLine("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
        telemetry.addData("pose", follower.pose());
        telemetry.update();
        follower.update();
    }

    /** This starts the OpMode by setting the drive motors to run left at full power. */
    @Override
    public void start() {
        follower.setPose(startPose);
        follower.update();

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.update();
        end = false;
    }

    /**
     * This runs the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run left enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        follower.update();

        if (!end) {
            if (Math.abs(follower.pose().y()) > (DISTANCE + startPose.y())) {
                end = true;
                follower.manual(0,0,0);
            } else {
                follower.manual(0,1,0);
                double currentVelocity = Math.abs(follower.twist().toVector2D().y());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        } else {
            follower.manual(0,0,0);
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetry.addData("Max Achievable Strafe Velocity: ", average);

            int k = 0;
            for (double v : velocities) {
                telemetry.addData(String.valueOf(k), v);
                k++;
            }

            telemetry.update();
        }
    }
}
