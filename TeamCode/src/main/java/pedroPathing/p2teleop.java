package pedroPathing;// FTC TeleOp Program - Optimized for Driver Control

// Import necessary FTC libraries
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Two Player Teleop")
public class p2teleop extends LinearOpMode {

    // Declare hardware variables
    DcMotor slide_horizontalMotor;
    DcMotor winch_rightMotor;
    DcMotor winch_leftMotor;
    DcMotor slide_verticalMotor;
    Servo arm_clawServo;
    Servo arm_verticalServo;
    Servo diff_frontServo;
    Servo diff_rightServo;
    Servo diff_leftServo;
    Servo hang_rightServo;
    Servo hang_leftServo;
    DcMotor topRight;
    DcMotor topLeft;
    DcMotor bottomRight;
    DcMotor bottomLeft;

    @Override
    public void runOpMode() {
        // Initialize hardware variables
        slide_horizontalMotor = hardwareMap.dcMotor.get("slide_horizontalMotor");
        winch_rightMotor = hardwareMap.dcMotor.get("winch_rightMotor");
        winch_leftMotor = hardwareMap.dcMotor.get("winch_leftMotor");
        slide_verticalMotor = hardwareMap.dcMotor.get("slide_verticalMotor");
        arm_clawServo = hardwareMap.servo.get("arm_clawServo");
        arm_verticalServo = hardwareMap.servo.get("arm_verticalServo");
        diff_frontServo = hardwareMap.servo.get("diff_frontServo");
        diff_rightServo = hardwareMap.servo.get("diff_rightServo");
        diff_leftServo = hardwareMap.servo.get("diff_leftServo");
        hang_rightServo = hardwareMap.servo.get("hang_rightServo");
        hang_leftServo = hardwareMap.servo.get("hang_leftServo");
        topRight = hardwareMap.dcMotor.get("topRight");
        topLeft = hardwareMap.dcMotor.get("topLeft");
        bottomRight = hardwareMap.dcMotor.get("bottomRight");
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");

        // Set motors to run with no power initially
        topRight.setPower(0);
        topLeft.setPower(0);
        bottomRight.setPower(0);
        bottomLeft.setPower(0);
        slide_horizontalMotor.setPower(0);
        slide_verticalMotor.setPower(0);
        winch_rightMotor.setPower(0);
        winch_leftMotor.setPower(0);

        // Wait for the start button to be pressed
        waitForStart();

        // Run the loop while the OpMode is active
        while (opModeIsActive()) {

            // Mecanum Drive control
            double drive = -gamepad1.left_stick_y; // Forward/Backward movement
            double strafe = gamepad1.left_stick_x; // Left/Right movement
            double rotate = gamepad1.right_stick_x; // Rotation

            // Mecanum drive calculations (simplified)
            topRight.setPower(drive - strafe - rotate);
            topLeft.setPower(drive + strafe + rotate);
            bottomRight.setPower(drive + strafe - rotate);
            bottomLeft.setPower(drive - strafe + rotate);

            // Winch control
            if (gamepad2.right_trigger > 0) {
                winch_rightMotor.setPower(1); // Winch up
                winch_leftMotor.setPower(1); // Winch up
            } else if (gamepad2.left_trigger > 0) {
                winch_rightMotor.setPower(-1); // Winch down
                winch_leftMotor.setPower(-1); // Winch down
            } else {
                winch_rightMotor.setPower(0); // Stop winch
                winch_leftMotor.setPower(0); // Stop winch
            }

            // Slide control (Horizontal)
            slide_horizontalMotor.setPower(gamepad2.left_stick_x);

            // Slide control (Vertical)
            slide_verticalMotor.setPower(gamepad2.left_stick_y);

            // Arm Claw control
            if (gamepad2.a) {
                arm_clawServo.setPosition(1); // Open Claw
            } else if (gamepad2.b) {
                arm_clawServo.setPosition(0); // Close Claw
            }

            // Arm Vertical control
            if (gamepad2.dpad_up) {
                arm_verticalServo.setPosition(1); // Move Arm Up
            } else if (gamepad2.dpad_down) {
                arm_verticalServo.setPosition(0); // Move Arm Down
            }

            // Differential Claw Control (Right)
            if (gamepad2.x) {
                diff_rightServo.setPosition(1); // Right Claw Move
            } else if (gamepad2.y) {
                diff_rightServo.setPosition(0); // Right Claw Reset
            }

            // Differential Claw Control (Left)
            if (gamepad2.left_bumper) {
                diff_leftServo.setPosition(1); // Left Claw Move
            } else if (gamepad2.right_bumper) {
                diff_leftServo.setPosition(0); // Left Claw Reset
            }

            // Hang Servo control (move both simultaneously)
            if (gamepad2.left_trigger > 0.1) {
                hang_rightServo.setPosition(1); // Rotate right hang servo
                hang_leftServo.setPosition(1);  // Rotate left hang servo
            } else if (gamepad2.right_trigger > 0.1) {
                hang_rightServo.setPosition(0); // Rotate right hang servo
                hang_leftServo.setPosition(0);  // Rotate left hang servo
            } else {
                hang_rightServo.setPosition(0.5); // Default neutral position
                hang_leftServo.setPosition(0.5);  // Default neutral position
            }

            // Telemetry for debugging (optional)
            telemetry.addData("Drive Power", drive);
            telemetry.update();
        }
    }
}
