package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManagerTeleop;

import java.util.function.Supplier;

@Configurable
@TeleOp (name="TeleOp Competiton Bleu", group="Competition")
public class TeleOpDecode extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private Intake intake;
    private Indexeur indexeur;

    private double targetRPM = 0.0;

    private AfficheurLeft afficheurLeft;
    private TireurManagerTeleop tireurManager;
    private Shooter shooter;
    private SpinTurret tourelle;
    private AngleShooter ServoAngleShoot;
    private ServoTireur servoTireur;

    //private boolean automatedDrive = false;

    private AfficheurRight afficheurRight;
    private boolean pulseSent = false;
    private boolean lastrightbumper = false ;

    private boolean lastleftbumpergamepad1 = false;

    private static final double SLOW_MULT = 0.35; // vitesse par défaut (lent)
    private static final double BOOST_MULT = 1.0; // plein régime quand on appuie



    @Override
    public void init() {

        tourelle = new SpinTurret();
        tourelle.init(hardwareMap);

        ServoAngleShoot = new AngleShooter();
        ServoAngleShoot.init(hardwareMap);
        indexeur = new Indexeur();
        indexeur.init(hardwareMap);

        afficheurLeft = new AfficheurLeft();
        afficheurLeft.init(hardwareMap);

        afficheurRight = new AfficheurRight();
        afficheurRight.init(hardwareMap);

        intake = new Intake(indexeur, afficheurLeft);
        intake.init(hardwareMap);
        indexeur.setIntake(intake);

        shooter = new Shooter();
        shooter.setIndexeur(indexeur);   // ✔️ on passe l’indexeur
        shooter.init(hardwareMap);       // ✔️ on initialise le hardware


        servoTireur = new ServoTireur(indexeur);  // ✔️ constructeur correct
        servoTireur.init(hardwareMap);            // ✔️ initialisation du servo
        ;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        tireurManager = new TireurManagerTeleop(shooter, tourelle, ServoAngleShoot, servoTireur, indexeur, intake, afficheurRight);
        ;

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(60, 86))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }


    private void fireIfReady(double angle, int rpm, int shots) {
        if (!tireurManager.isBusy()) {
            if (shots == 1) {
                tireurManager.startTirManuel1tir(angle, rpm);
                afficheurRight.setJaune();
            } else {
                tireurManager.startTirManuel3Tirs(angle, rpm);
                afficheurRight.setClignoteVert();
            }
        } else {
            telemetry.addData("Tir", "IGNORÉ: tireur occupé (BUSY)");
        }
    }

        @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        double t = getRuntime();
        // au bout de 20 secondes : impulsion unique
        if (t>=20.0 && !pulseSent){
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            pulseSent = true;
            afficheurRight.setClignoteVert();
        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors


        // Lecture des sticks
            double ly = -gamepad1.left_stick_y;
            double lx = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

        // Deadband
            if (Math.abs(ly) < 0.05) ly = 0;
            if (Math.abs(lx) < 0.05) lx = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

        // BOOST tenu au bumper droit
            boolean boost = gamepad1.right_bumper;
            double mult = boost ? BOOST_MULT : SLOW_MULT;

        // Si rotation non ralentie, mettre rotMult = BOOST_MULT
            double rotMult = mult;

            follower.setTeleOpDrive(
                    ly * mult,
                    lx * mult,
                    rx * rotMult,
                    false);
        }

        //Automated PathFollowing
        if (gamepad1.yWasPressed()) {
            follower.followPath(pathChain.get());
             automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.left_bumper && !lastleftbumpergamepad1) {
            intake.setetatEJECTION();
        }
        lastleftbumpergamepad1 = gamepad1.left_bumper;


       if (gamepad1.xWasPressed()) {
            indexeur.setBalles(0);            // reset des balles
            // demarre l'intake automatiquement
        }


        double powertourelle = gamepad2.left_stick_x; // Joystick Horizontal
        tourelle.rotationtourelle(powertourelle);

        int shotsMode = (gamepad2.left_bumper ? 1 : 3);

        // RB (g2) : position fréquente de tir & en autonome
        if (gamepad2.right_bumper && !lastrightbumper) {
                fireIfReady(0.35, 4000, shotsMode);
            }
            lastrightbumper = gamepad2.right_bumper;

        // Y : très proche du goal dans zone proche
        if (gamepad2.yWasPressed()) {
                fireIfReady(0.12, 3500, shotsMode);}

        // B : intermédiaire
        if (gamepad2.bWasPressed()) {
                fireIfReady(0.28, 3800, shotsMode);
            }

        // X : longue distance 1
        if (gamepad2.xWasPressed()) {
                fireIfReady(0.38, 4100, shotsMode);
            }

        // A : longue distance 2 (RPM plus haut)
        if (gamepad2.aWasPressed()) {
                fireIfReady(0.38, 4700, shotsMode);
            }
        intake.update();
        indexeur.update();
        tireurManager.update();
        afficheurRight.update();

        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);
        //telemetry.addData("angle Tourelle actuel", tourelle.lectureangletourelle());
        //telemetry.addData("AngleShoot", positionAngleshoot);
        //telemetry.addData("RPM", intake.getRPM());
        telemetry.addData("DistanceBalle", intake.getCapteurDistance());
        telemetry.addData("Lum Indexeur", intake.getLumIndexeur());
        //telemetry.addData("Score", intake.getScore());
        telemetry.addData("État Indexeur", indexeur.getEtat());
        //telemetry.addData("Pale detectée", indexeur.detectionpale());
        //for (int i = 0; i < 3; i++) { telemetry.addData("Compartiment " + i, indexeur.getCouleurCompartiment(i)); }
        telemetry.addData("État tireur manager", tireurManager.getState());
        telemetry.addData("Shooter RPM", shooter.getShooterVelocityRPM());
        telemetry.addData("Index rotation finie", indexeur.isRotationTerminee());

        telemetry.update();

    }
}