package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/**
 * This OpMode identifies the robot's forward deceleration.
 * It runs the robot forward until a specified velocity is reached, then cuts power to measure
 * the natural deceleration. The average deceleration is then displayed.
 * This helps in accurately estimating the robot's braking behavior for path following.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
@TeleOp(group = "2")
public class ForwardDecelerationIdentification extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;

    private double previousVelocity;
    private long previousTimeNano;

    private boolean stopping;
    private boolean end;
    private Follower follower;
    private final Pose startPose = new Pose(70.75,70.75);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    /** This initializes the drive motors as well as the cache of velocities. */
    @Override
    public void init_loop() {
        telemetry.addLine("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
        telemetry.addLine("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.addLine("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetry.update();
        follower.update();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        follower.manual(1,0,0);
        follower.update();
        end = false;
        stopping = false;
    }

    /**
     * This runs the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        follower.update();

        if (!end) {
            if (!stopping) {
                if (follower.velocity().toVector2D().x() > VELOCITY) {
                    previousVelocity = follower.velocity().toVector2D().x();
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.manual(0,0,0);
                }
            } else {
                double currentVelocity = follower.velocity().toVector2D().x();
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < 0.1) {
                    end = true;
                }
            }
        } else {
            follower.manual(0,0,0);
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();
            telemetry.addData("Forward Max Achievable Deceleration: ", Math.abs(average)); // abs valued to match constants input :)

            telemetry.update();
        }
    }
}