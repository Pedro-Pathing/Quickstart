package org.firstinspires.ftc.teamcode.pedroPathing.Auto;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous (name="bleuFond", group="Competition")
public class DecodeBlueAutoFond extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //private ElapsedTime pathTimer = new ElapsedTime();
    //private ElapsedTime opModeTimer = new ElapsedTime();
    private Shooter shooter;
    private SpinTurret tourelle;
    private AngleShooter ServoAngleShoot;
    private ServoTireur servoTireur;
    private Indexeur indexeur;
    private Intake intake;

    private AfficheurLeft afficheurLeft;
    private AfficheurRight afficheurRight;

    private TireurManager tireurManager;

    private boolean shotsTriggered = false;

    public enum PathState {
        //Start Position -End Position
        //drive > Movement
        //Shoot >Attempts to Score the Artifact
        DRIVE_STARTPOSITIONTOSHOOT,
        align_RANGEE1Blue,
        align_rangee2blue,
        align_rangee3blue,
        intakeballeRangee1,
        intakerange2,
        intakerangee3,
        PremierTir,
        DrivedeuxiemeShoot,
        deuxiemetir,
        DriveTroisiemeTir,
        troisiemetir,
        Drive2Gate,
        atgate
    }
    PathState pathState;

    private final Pose startPose = new Pose(33.9416569,135.598599, Math.toRadians(180));
    private final Pose firstshootPose = new Pose(42.00700116,103.0011668,Math.toRadians(137));

    private final Pose drivetoligne1= new Pose (40.3920653, 84.8378063, Math.toRadians(180));

    private final Pose avalerballeRangee1 = new Pose (15.4830805, 84.1820303, Math.toRadians(180));

    private final Pose Shoot2 = new Pose (60.3220536, 84.0140023, Math.toRadians(137));

    private final Pose drivetoligne2= new Pose (40.71995332555426, 59.649941656942815, Math.toRadians(180));

    private final Pose avalerballeRangee2= new Pose (15.4830805, 59.649941656942815, Math.toRadians(180));

    private final Pose Gate= new Pose (14, 70, Math.toRadians(-90));

    private PathChain driveStartofirstShootPos, driveShoot2pickup1Pos, driveAvalerpremiereLigne, DrivedeuxiemeShoot,drivetorangee2, drivetavalerdeuxiemeligne,driveAvaler2emeLignetotroisemeShoot,DrivetoGate;

    public void buildPaths() {
        //put the coordinate from start to shooting
        driveStartofirstShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstshootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstshootPose.getHeading())
                .build();

        //du premiershoot à la rangée numéro 1
        driveShoot2pickup1Pos = follower.pathBuilder()
                .addPath(new BezierLine(firstshootPose, drivetoligne1))
                .setLinearHeadingInterpolation(firstshootPose.getHeading(), drivetoligne1.getHeading())
                .build();

        //de la l'alignement pickup1 à la derniere balle rangée 1
        driveAvalerpremiereLigne = follower.pathBuilder()
                .addPath(new BezierLine(drivetoligne1,avalerballeRangee1))
                .setLinearHeadingInterpolation(drivetoligne1.getHeading(), avalerballeRangee1.getHeading())

                .setVelocityConstraint(0.23)
                .build();
        //Aller à la zone de Tir apres avoir avaler les balles de la rangée 1
        DrivedeuxiemeShoot = follower.pathBuilder()
                .addPath(new BezierLine(avalerballeRangee1,Shoot2))
                .setLinearHeadingInterpolation(avalerballeRangee1.getHeading(), Shoot2.getHeading())
                .build();
        //Aller s'aligner à la deuxieme rangée de balle
        drivetorangee2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot2, drivetoligne2))
                .setLinearHeadingInterpolation(Shoot2.getHeading(), drivetoligne2.getHeading())
                .build();

        //Aller avaler les balles de la rangee 2
        drivetavalerdeuxiemeligne = follower.pathBuilder()
                .addPath(new BezierLine(drivetoligne2, avalerballeRangee2))
                .setLinearHeadingInterpolation(drivetoligne2.getHeading(), avalerballeRangee2.getHeading())
                .setVelocityConstraint(0.23)
                .build();

        //Aller à la zone de Tir apres avoir avaler les balles de la rangée 2
        driveAvaler2emeLignetotroisemeShoot = follower.pathBuilder()
                .addPath(new BezierLine(avalerballeRangee2,Shoot2))
                .setLinearHeadingInterpolation(avalerballeRangee2.getHeading(), Shoot2.getHeading())
                .build();


        //Aller de la zone du troisieme Tir à la troisieme rangée
        DrivetoGate= follower.pathBuilder()
                .addPath(new BezierLine(Shoot2,Gate))
                .setLinearHeadingInterpolation(Shoot2.getHeading(),Gate.getHeading())
                .build();


    }
    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOSITIONTOSHOOT:
                follower.followPath(driveStartofirstShootPos,0.8, true); //true will hold the positon
                setPathState(PathState.PremierTir); // Reset Timer + make new staet
                break;

            case PremierTir: // Premier tir en cours
                //intake.update();
                //indexeur.update();
                if (!follower.isBusy()) {
                    // avons nous deja demandé des tirs :

                    if (!shotsTriggered){
                        tireurManager.startTirAuto(// Lancer tir automatique
                                -45,   // angle tourelle (exemple)
                                0.35,  // angle shooter
                                3930   // RPM
                        );
                        shotsTriggered = true;}
                    else if (shotsTriggered && !tireurManager.isBusy()){
                            setPathState(PathState.align_RANGEE1Blue);
                            shotsTriggered = false;
                        }

                }
                break;


            case align_RANGEE1Blue: // On va s'aliner avec la rangée 1
                // check is follow done is path
                intake.update();
                indexeur.update();
                //if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5) {
                if (!follower.isBusy()) {
                    telemetry.addLine("Done with Shooting 1, deplacement vers premiere rangée");
                    // transition to next state
                    follower.followPath(driveShoot2pickup1Pos ,0.9, true); // chemin d'alignement de la premiere rangée
                    setPathState(PathState.intakeballeRangee1); // on va a l'étape suivante
                }
                break;

            case intakeballeRangee1:
                // mise à jour des systèmes
                intake.update();
                indexeur.update();

                if (!follower.isBusy()) {// attendre que le path soit fini
                    follower.followPath(driveAvalerpremiereLigne,0.37,true); // on avance doucement pour avaler les balles
                    setPathState(PathState.DrivedeuxiemeShoot);
                    }
                break;

            case DrivedeuxiemeShoot:
                ;
                if (!follower.isBusy()) { // Attendre que l'on est fini d'avoir pris toutes les balles
                    follower.followPath(DrivedeuxiemeShoot,0.9,true);
                    // Le robot est arrivé en position de tir :
                    setPathState(PathState.deuxiemetir);
                }
                break;

            case deuxiemetir:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) { // deuxieme période de tir
                        tireurManager.startTirAuto(// Lancer tir automatique
                                -45,   // angle tourelle (exemple)
                                0.32,  // angle shooter
                                3950   // RPM
                        );
                        shotsTriggered = true;
                    } else if (shotsTriggered && !tireurManager.isBusy()) {
                        setPathState(PathState.align_rangee2blue);
                        shotsTriggered = false;
                    }
                }
                break;

            case align_rangee2blue: // alignement avec la deuxieme zone de balle (centrale)
                intake.update(); // mise à jour de nos systemes (constate que toutes les balles sont parties)
                indexeur.update();
                //if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>5) {
                if (!follower.isBusy()) {
                    follower.followPath(drivetorangee2,0.9, true);
                // TO DO demarer intake , tourner indexeur des dectetion balles)
                telemetry.addLine("alignement ramassage ligne 2");
                // transition to next state
                setPathState(PathState.intakerange2);
                }
                break;

            case intakerange2:
                intake.update(); // mise à jour de nos systemes (constate que toutes les balles sont parties)
                indexeur.update();
                if (!follower.isBusy()) {
                    follower.followPath(drivetavalerdeuxiemeligne, 0.35 , true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("ramassage 2 terminé");
                    // transition to next state
                    setPathState(PathState.DriveTroisiemeTir);
                    }
                break;

            case DriveTroisiemeTir:
                intake.update(); // mise à jour de nos systemes (constate que toutes les balles sont parties)
                indexeur.update();
                if (!follower.isBusy()) {
                    follower.followPath(driveAvaler2emeLignetotroisemeShoot,1, true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("Position 3 de tir");
                    // transition to next state
                    setPathState(PathState.troisiemetir);
                }
                break;

            case troisiemetir:
                if (!follower.isBusy()) {
                    // le robot est arrivé sur la troisieme position de tir :

                    if (!shotsTriggered){
                        tireurManager.startTirAuto(// Lancer tir automatique
                                -32,   // angle tourelle (exemple)
                                0.28,  // angle shooter
                                3850   // RPM
                        );
                        shotsTriggered = true;}
                    else if (shotsTriggered && !tireurManager.isBusy()){
                            setPathState(PathState.Drive2Gate);
                            shotsTriggered = false;
                        }

                }
                break;
            case Drive2Gate:
                intake.update(); // mise à jour de nos systemes (constate les balles sont tirées )
                indexeur.update();
                // shoot logique 3eme Tir
                if (!follower.isBusy()) {
                    follower.followPath(DrivetoGate,1,true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("Auto Termine & A cote de la porte ");
                    // transition to next state
                    setPathState(PathState.atgate);

                }
                break;

            case atgate:
                telemetry.addLine("C'est fini");
                break;
        }

    }

    public void setPathState (PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;

    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOSITIONTOSHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);


        // --- Initialisation hardware ---
        shooter = new Shooter();
        shooter.init(hardwareMap);

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

        servoTireur = new ServoTireur(indexeur);
        servoTireur.init(hardwareMap);

        // --- TireurManager ---
        tireurManager = new TireurManager(shooter, tourelle, ServoAngleShoot, servoTireur, indexeur, intake, afficheurRight);

        indexeur.setBalles(3);
        indexeur.resetCompartiments();

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);

    }
    @Override
    public void loop (){
        follower.update();
        statePathUpdate();
        intake.update();
        indexeur.update();
        tireurManager.update();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("angle Tourelle actuel", tourelle.lectureangletourelle());
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
