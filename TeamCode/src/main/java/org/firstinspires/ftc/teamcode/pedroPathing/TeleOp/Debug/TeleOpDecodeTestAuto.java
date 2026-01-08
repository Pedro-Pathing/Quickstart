package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Debug;

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

import java.util.function.Supplier;

@Configurable
@TeleOp (name="AutoTir Test", group="debug")
public class TeleOpDecodeTestAuto extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Intake intake;
    private Indexeur indexeur;

    private double targetRPM = 0.0;

    private AfficheurLeft afficheurLeft;
    private TireurManager tireurManager;
    private Shooter shooter;
    private SpinTurret tourelle;
    private AngleShooter ServoAngleShoot;
    private ServoTireur servoTireur;

    private AfficheurRight afficheurRight;
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastrightbumper = false ;
    private boolean lastleftbumper = false;
    int positionAngleshoot =0 ;
    boolean anglePresetMode = false;

    int presetIndexShooter = 0;


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
        tireurManager = new TireurManager(shooter, tourelle, ServoAngleShoot, servoTireur, indexeur, intake, afficheurRight);
        ;




        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        tireurManager = new TireurManager(shooter, tourelle, ServoAngleShoot, servoTireur, indexeur, intake, afficheurRight);
        ;

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
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

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false// Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        //if (gamepad2.yWasPressed()) {
        //    slowModeMultiplier -= 0.25;
        //}

        // --- Déclenchement tir auto ---
        if (gamepad2.right_bumper && !lastrightbumper) {

            // Exemple : tir droit devant
            double angleTourelle = 0;      // à adapter
            double angleShooter = 0.12;      // à adapter
            double vitesseShooter = 3775;

            tireurManager.startTirAuto(angleTourelle, angleShooter, vitesseShooter);
        }

        lastrightbumper = gamepad2.right_bumper;
        // --- Mise à jour du manager ---

        if (gamepad2.left_bumper && !lastleftbumper) {

             double angleTourelle = 0;      // à adapter
             double angleShooter = 0.12;      // à adapter
             double vitesseShooter = 3750;  // à adapter

            tireurManager.startTirAuto(angleTourelle, angleShooter, vitesseShooter);

        }

        lastleftbumper = gamepad2.left_bumper;

        //1) gestion des prereglages tourelles avec le bouton X en manuel

        //tourelle.allerVersAngle(45);

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        intake.update();
        indexeur.update();
        tireurManager.update();

        telemetry.addData("angle Tourelle actuel", tourelle.lectureangletourelle());
        telemetry.addData("AngleShoot", positionAngleshoot);
        telemetry.addData("RPM", intake.getRPM());
        telemetry.addData("DistanceBalle", intake.getCapteurDistance());
        telemetry.addData("Lum Indexeur", intake.getLumIndexeur());
        telemetry.addData("Score", intake.getScore());
        telemetry.addData("État Indexeur", indexeur.getEtat());
        telemetry.addData("Pale detectée", indexeur.detectionpale());
        telemetry.addData("Nombre de balles", indexeur.getBalles());
        for (int i = 0; i < 3; i++) { telemetry.addData("Compartiment " + i, indexeur.getCouleurCompartiment(i)); }
        telemetry.addData("État tireur manager", tireurManager.getState());
        telemetry.addData("État indexeur", indexeur.getEtat());
        telemetry.addData("Etat de l'intake", intake.getEtat());
        telemetry.addData("Shooter RPM", shooter.getShooterVelocityRPM());
        telemetry.addData("Servo pos", servoTireur.getPosition());
        telemetry.addData("Index rotation finie", indexeur.isRotationTerminee());

        telemetry.update();

    }
}