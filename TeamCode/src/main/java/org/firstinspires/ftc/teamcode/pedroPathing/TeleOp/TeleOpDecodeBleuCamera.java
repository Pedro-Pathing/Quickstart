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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManager;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManagerTeleop;
import org.firstinspires.ftc.teamcode.pedroPathing.navigation.Camerahusky;

import java.util.function.Supplier;

@Configurable
@TeleOp (name="TeleOp Competiton Bleu Camera", group="Competition")
public class TeleOpDecodeBleuCamera extends OpMode {
    Camerahusky Camera = new Camerahusky();
    private double ajustementcamera = 1.5; //valeur d'ajustement caméra (erreur) a changer si probleme

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

    private double distanceCamerajustee = 0.0 ;
    private double distanceCameramesuree = 0.0 ;

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
        Camera.init(hardwareMap);
        if (!Camera.isConnected()){
            telemetry.addData(">>>", "probleme communication caméra");}
        else {
            telemetry.addData(">>>","HuskyLens Prêt");
        }
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose((55), 81, Math.toRadians(0)));
        follower.update();

        Pose p = follower.getPose();

        telemetry.addData("POSE Pedro", p);

        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        //follower.update();

        // 1. Injecter l'IMU
        //double imuHeading = follower.getHeading();     // ex: 0 rad
        //Pose p = follower.getPose();
        //follower.setStartingPose(new Pose(p.getX(), p.getY(), imuHeading)); // -> (x_auto, y_auto, 0 rad)
        //follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        tireurManager = new TireurManagerTeleop(shooter, tourelle, ServoAngleShoot, servoTireur, indexeur, intake, afficheurRight);
        ;


        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(55, 81))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();


        telemetry.addData("Starting pose TeleOp",
                "x=%.1f y=%.1f θ=%.1f°",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();

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

    private void firefondTerrainAuto(double angletourelle, double angleshooter, int rpm) {
        if (!tireurManager.isBusy()) {
                tireurManager.startTirAuto(angletourelle,angleshooter,rpm);
                }
            else {
            telemetry.addData("Tir", "IGNORÉ: tireur occupé (BUSY)");}
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


        // BOOST avec au bumper droit
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

       if (gamepad1.dpadRightWasPressed()){
           indexeur.reculerIndexeurbourrage();
       }
       boolean tirautoactif = tireurManager.isTirAutoActif();
        if (!tirautoactif) {
            double powertourelle = gamepad2.left_stick_x; // Joystick Horizontal
            tourelle.rotationtourelle(powertourelle);
        }

        // Deadzone pour considérer que le pilote "veut" bouger la tourelle
        //final double TURRET_DEADZONE = 0.02;
        //double powertourelle = gamepad2.left_stick_x;
        // Si le pilote bouge vraiment le stick -> priorité au manuel pour CETTE boucle
        //if (Math.abs(powertourelle) > TURRET_DEADZONE) {
        //        tourelle.rotationtourelle(powertourelle);
        //}

       int shotsMode = (gamepad2.left_bumper ? 1 : 3);

       // RB (g2) : position fréquente de tir & en autonome Position 4 tres éloigné
       //if (gamepad2.right_bumper && !lastrightbumper) {
       //        fireIfReady(0.35, 4400, shotsMode);
       //    }
       //    lastrightbumper = gamepad2.right_bumper;

       // Y : Position 1 distance 1 robot de la zone de tir
       if (gamepad2.yWasPressed()) {

           fireIfReady(0.30, 3970, shotsMode);
       }

        // B : Position 3 milieu de terrain bouton 2  droite
       if (gamepad2.bWasPressed()) {
           fireIfReady(0.42, 4200, shotsMode);
       }


       // A : Position 4 la plus loin bouton 1 le plus proche co driver
       if (gamepad2.aWasPressed()) {
                //fireIfReady(0.17, 3750, shotsMode);
                 fireIfReady(0.44, 4400, shotsMode);
       }

        // X : Bouton Gauche Boutton 0  reset de l'IMU de la tourelle
       if (gamepad2.xWasPressed()) {
                //fireIfReady(0.1, 3700, shotsMode);
           tourelle.resetImuToutelle();
            }
        // Pad tir de loin
        if (gamepad2.dpadUpWasPressed()){
            fireIfReady(0.55, 4850, shotsMode);
            }

        //tir tres eloigné.
        if (gamepad2.dpadDownWasPressed()){
                fireIfReady(0.55, 4950, shotsMode);
            }

        if (gamepad2.dpadLeftWasPressed()){
                tirautoactif = true;
                firefondTerrainAuto(-60.0,0.55,4850);
            }

        if (gamepad2.dpadRightWasPressed()){
                tireurManager.setState(TireurManagerTeleop.TirState.IDLE);
                shooter.setShooterTargetRPM(0);

            }

        if (Camera.hasTag() == true){
            afficheurRight.setBleu();
            distanceCamerajustee = Camera.getDistanceFiableCm();
            distanceCameramesuree = Camera.getEstimatedDistanceCm();
            telemetry.addData("Distancemesurée", distanceCameramesuree);
            telemetry.addData("Distance_camera_apriltag", distanceCamerajustee);
            telemetry.update();
            if (distanceCamerajustee > 0 && distanceCamerajustee < 70) {
                afficheurRight.setVert();
            }
            if (distanceCamerajustee > 68 && distanceCamerajustee < 110) {
                afficheurRight.setJaune();

            }

            if (distanceCamerajustee > 110 && distanceCamerajustee < 150) {
                    afficheurRight.setOrange();

            }
            if (distanceCamerajustee > 150 && distanceCamerajustee < 187) {
                    afficheurRight.setRouge();
            }
            if (distanceCamerajustee > 187 && distanceCamerajustee < 200) {
                    afficheurRight.setViolet();
            }
            afficheurRight.update();
            distanceCamerajustee = 0;
            distanceCameramesuree =0;
        }

        if(!Camera.hasTag() == true){
            afficheurRight.setIdle();
            afficheurRight.update();
        }

        // Tir automatique caméra
        if (gamepad2.rightBumperWasPressed()) {
            distanceCamerajustee = 0;


            if (Camera.hasTag() == true) {
                distanceCamerajustee = (Camera.getDistanceFiableCm())/100;
                double rpmDouble = 541.62 * distanceCamerajustee + 3210.34;//ajout 20
                int rpm = (int) Math.round(rpmDouble);


                telemetry.addData("RPM dynamique", rpm);
                telemetry.update();

                if (distanceCamerajustee > 0 && distanceCamerajustee < 0.68) {
                    fireIfReady(0.1,rpm, shotsMode); //3700
                    distanceCamerajustee = 0;
                    distanceCameramesuree =0;
                    rpm = 0;

                }
                if (distanceCamerajustee > 0.68 && distanceCamerajustee < 1.10) {
                    fireIfReady(0.17, rpm, shotsMode); //3750
                    distanceCamerajustee = 0;
                    distanceCameramesuree =0;
                    rpm = 0;
                }

                if (distanceCamerajustee > 1.10 && distanceCamerajustee < 1.50) {
                    fireIfReady(0.30, rpm, shotsMode); //3900
                    distanceCamerajustee = 0;
                    distanceCameramesuree =0;
                    rpm = 0;

                }

                if (distanceCamerajustee > 1.50 && distanceCamerajustee < 2.50) {
                    fireIfReady(0.44, rpm, shotsMode); //4200
                    distanceCamerajustee = 0;
                    distanceCameramesuree =0;
                    rpm =0;
                }

            }
            else {afficheurRight.setRouge();
                afficheurRight.update();}

        }


        intake.update();
        indexeur.update();
        tireurManager.update();
        afficheurRight.update();
        afficheurLeft.update();

        //telemetryM.debug("position", follower.getPose());
        //telemetryM.debug("velocity", follower.getVelocity());
        //telemetryM.debug("automatedDrive", automatedDrive);
        //telemetry.addData("angle Tourelle actuel", tourelle.lectureangletourelle());
        //telemetry.addData("AngleShoot", positionAngleshoot);
        //telemetry.addData("RPM", intake.getRPM());
        //telemetry.addData("DistanceBalle", intake.getCapteurDistance());
        //telemetry.addData("Lum Indexeur", intake.getLumIndexeur());
        //telemetry.addData("Score", intake.getScore());
        //telemetry.addData("État Indexeur", indexeur.getEtat());
        //telemetry.addData("Pale detectée", indexeur.detectionpale());
        //for (int i = 0; i < 3; i++) { telemetry.addData("Compartiment " + i, indexeur.getCouleurCompartiment(i)); }
        //telemetry.addData("État tireur manager", tireurManager.getState());
        //telemetry.addData("Shooter RPM", shooter.getShooterVelocityRPM());
        //telemetry.addData("Index rotation finie", indexeur.isRotationTerminee());

        telemetry.update();

    }
}
