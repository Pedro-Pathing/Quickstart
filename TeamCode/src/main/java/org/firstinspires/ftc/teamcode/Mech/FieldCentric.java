package org.firstinspires.ftc.teamcode.Mech;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

// Drive field-centric pelo Follower: o stick segue a perspectiva do piloto, qualquer que seja o heading.
// O OpMode continua responsavel por chamar follower.update() todo loop.
public class FieldCentric {

    private final Follower follower;
    private double offsetRad; // heading que conta como a "frente" do piloto

    public FieldCentric(Follower follower) {
        this.follower = follower;
    }

    // A direcao atual do robo passa a ser a frente do piloto
    public void resetHeading() {
        offsetRad = follower.pose().heading();
    }

    // x = direita do piloto, y = frente do piloto, rx = giro p/ direita (gamepad cru, com y ja invertido)
    public DrivePowers powers(double x, double y, double rx) {
        // Pedro: lateral + = esquerda, giro + = anti-horario
        return ManualDrive.fieldCentric(y, x, rx, follower.pose().heading(), -offsetRad);
    }

    public void drive(double x, double y, double rx) {
        follower.manual(powers(x, y, rx));
    }

    public double getOffsetRad() {
        return offsetRad;
    }
}
