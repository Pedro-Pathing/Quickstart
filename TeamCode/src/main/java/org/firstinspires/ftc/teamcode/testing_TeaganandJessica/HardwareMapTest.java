package org.firstinspires.ftc.teamcode.testing_TeaganandJessica;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
public class HardwareMapTest {
public DcMotor shooter = null;
public HardwareMap hwm = null;
public ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap HardwareMapTest) {
      hwm = HardwareMapTest;
      shooter = hwm.get(DcMotor.class, "shooter");
      shooter.setDirection(DcMotor.Direction.FORWARD);
      shooter.setPower(0);
      shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    } public Command shooter(double power, double time) {
        return Command.build()
                .setStart(() -> timer.reset())
                .setExecute(() -> shooter.setPower(power))
                .setDone(() -> timer.time() >= time)
                .setEnd(EndCondition -> shooter.setPower(0))
                .requiring(shooter);
    }
}
