package org.firstinspires.ftc.teamcode.pedroPathing.identification;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

/**
 * This is the ForwardVelocityIdentification autonomous follower OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityIdentification, allows  to create a Vector that
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
public class ForwardVelocityIdentification extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

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
        telemetry.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetry.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetry.addLine("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
        telemetry.addData("pose", follower.pose());
        telemetry.update();
        follower.update();
    }

    /** This starts the OpMode by setting the drive motors to run forward at full power. */
    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.update();
        end = false;
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
            if (Math.abs(follower.pose().x()) > (DISTANCE + startPose.x())) {
                end = true;
                follower.manual(0,0,0);
            } else {
                follower.manual(1,0,0);
                double currentVelocity = Math.abs(follower.velocity().toVector2D().x());
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            follower.manual(0,0,0);
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetry.addData("Forward Velocity: ", average);

            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData(String.valueOf(i), velocities.get(i));
            }

            telemetry.update();
        }
    }
}

