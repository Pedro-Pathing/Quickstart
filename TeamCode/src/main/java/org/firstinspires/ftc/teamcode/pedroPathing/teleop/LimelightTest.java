package org.firstinspires.ftc.teamcode.pedroPathing.teleop;


import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawRobot;
import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.sendPacket;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;




@TeleOp(name = "LimelightTest",group = "Test")
public class LimelightTest extends OpMode {


    private FieldManager fieldManager;
    private Style llPos;
    private Limelight3A ll;


    private Odometry odo;


    @Override
    public void init() {
        fieldManager = PanelsField.INSTANCE.getField();
        llPos = new Style("","#3F51B5",0.75);
        fieldManager.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(1);
        odo = hardwareMap.get(Odometry.class, "odo");

    }


    @Override
    public void start() {
        ll.start();
    }


    @Override
    public void loop() {
        double heading = Math.toDegrees(odo.getPose().getHeading());
        ll.updateRobotOrientation(heading);
        LLResult LLResult = ll.getLatestResult();
        if (LLResult != null && LLResult.isValid()) {
            Pose3D botpose = LLResult.getBotpose_MT2();
            Position position = botpose.getPosition();
            telemetry.addData("X", botpose.getPosition().x);
            telemetry.addData("Y", botpose.getPosition().y);
            telemetry.addData("Heading", botpose.getOrientation().getYaw(AngleUnit.DEGREES));


            double dx = botpose.getPosition().x - odo.getPose().getX();
            double dy = botpose.getPosition().y - odo.getPose().getY();
            double dist = Math.hypot(dx,dy);
            Position pos = botpose.getPosition();
            Pose botpose2D = new Pose(pos.x, pos.y, Math.toRadians(heading));
            drawRobot(botpose2D,llPos);
        }
        telemetry.update();
        fieldManager.update();
    }
}
