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
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous (name="bleuFond", group="PedroPathing")
public class DecodeBlueAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //private ElapsedTime pathTimer = new ElapsedTime();
    //private ElapsedTime opModeTimer = new ElapsedTime();
    private Shooter shooter;
    private SpinTurret tourelle;
    private AngleShooter angleShooter;
    private ServoTireur servoTireur;
    private Indexeur indexeur;
    private Intake intake;

    private AfficheurLeft afficheurLeft;
    private AfficheurRight afficheurRight;


    private TireurManager tireurManager;

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

        DriveTroisiemeTir,
        Drive2Gate,
        atgate

    }

    PathState pathState;

    private final Pose startPose = new Pose(33.9416569,135.598599, Math.toRadians(180));
    private final Pose firstshootPose = new Pose(42.00700116,103.0011668,Math.toRadians(137));

    private final Pose drivetoligne1= new Pose (48.3920653, 82.8378063, Math.toRadians(180));

    private final Pose avalerballeRangee1 = new Pose (18.4830805, 84.1820303, Math.toRadians(180));

    private final Pose Shoot2 = new Pose (60.3220536, 84.0140023, Math.toRadians(137));

    private final Pose drivetoligne2= new Pose (47.71995332555426, 59.649941656942815, Math.toRadians(180));

    private final Pose avalerballeRangee2= new Pose (18.4830805, 59.649941656942815, Math.toRadians(180));

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
                .setVelocityConstraint(0.25)
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
                .setVelocityConstraint(0.25)
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
                follower.followPath(driveStartofirstShootPos, true); //true will hold the positon
                setPathState(PathState.PremierTir); // Reset Timer + make new staet
                break;

            case PremierTir:
                if (!follower.isBusy()) {

                    // Lancer tir automatique
                    tireurManager.startTirAuto(
                            0,   // angle tourelle (exemple)
                            0.35,  // angle shooter
                            4500   // RPM
                    );

                    setPathState(PathState.align_RANGEE1Blue);
                }
                break;


            case align_RANGEE1Blue: // On va dans cette etape
                // check is follow done is path
                intake.update();
                indexeur.update();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>5) {
                    telemetry.addLine("Done with Shooting 1, deplacement vers premiere rangée");
                    // transition to next state
                    follower.followPath(driveShoot2pickup1Pos , true);
                    setPathState(PathState.intakeballeRangee1);
                }
                break;

            case intakeballeRangee1:

                // Toujours mettre à jour les systèmes
                intake.update();
                indexeur.update();

                // On ne décide de rien tant que le path n'est pas fini
                if (!follower.isBusy()) {

                    // Condition normale : 3 balles validées par l'indexeur
                    if (indexeur.getBalles() >= 3) {

                        telemetry.addLine("ramassage terminé (3 balles)");

                        // 1. Couper l'intake
                        intake.setIntakeBallTargetRPM(0);

                        // 2. Couper l'indexeur
                        indexeur.setEtat(Indexeur.Indexeuretat.IDLE);

                        // 3. Lancer le path suivant
                        follower.followPath(driveAvalerpremiereLigne, true);
                        setPathState(PathState.DrivedeuxiemeShoot);
                        break;
                    }

                    // Optionnel : sécurité si pas 3 balles mais path terminé depuis trop longtemps
                    if (pathTimer.getElapsedTimeSeconds()> 3.0) {
                        telemetry.addLine(" ramassage incomplet, on continue quand même");

                        intake.setIntakeBallTargetRPM(0);
                        indexeur.setEtat(Indexeur.Indexeuretat.IDLE);

                        follower.followPath(driveAvalerpremiereLigne, true);
                        setPathState(PathState.DrivedeuxiemeShoot);
                        break;
                    }
                }

                break;

            case DrivedeuxiemeShoot:
                ;
                // check is follow done is path
                if (!follower.isBusy()) {
                    telemetry.addLine("Shooting");
                    // transition to next state
                    follower.followPath(DrivedeuxiemeShoot, true);
                    setPathState(PathState.align_rangee2blue);
                }

                break;

            case align_rangee2blue:

                // Zone de Tir , apres départ
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>5) {
                    follower.followPath(drivetorangee2, true);
                // TO DO demarer intake , tourner indexeur des dectetion balles)
                telemetry.addLine("ramassage terminé");
                // transition to next state
                setPathState(PathState.intakerange2);

            }
            break;

            case intakerange2:
                if (!follower.isBusy()) {
                    follower.followPath(drivetavalerdeuxiemeligne , true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("ramassage terminé");
                    // transition to next state
                    setPathState(PathState.DriveTroisiemeTir);

                }
                break;

            case DriveTroisiemeTir:
                if (!follower.isBusy()) {
                    follower.followPath(driveAvaler2emeLignetotroisemeShoot,true);
                    // TO DO demarer intake , tourner indexeur des dectetion balles)
                    telemetry.addLine("ramassage terminé");
                    // transition to next state
                    setPathState(PathState.Drive2Gate);

                }
                break;

            case Drive2Gate:
                // shoot logique 3eme Tir
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>5) {
                    follower.followPath(DrivetoGate,true);
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

        angleShooter = new AngleShooter();
        angleShooter.init(hardwareMap);

        indexeur = new Indexeur();
        indexeur.init(hardwareMap);

        afficheurLeft = new AfficheurLeft();
        afficheurLeft.init(hardwareMap);

        afficheurRight = new AfficheurRight();
        afficheurRight.init(hardwareMap);

        intake = new Intake(indexeur, afficheurLeft);
        intake.init(hardwareMap);

        servoTireur = new ServoTireur(indexeur);
        servoTireur.init(hardwareMap);

        // --- TireurManager ---
        tireurManager = new TireurManager(shooter, tourelle, angleShooter, servoTireur, indexeur, intake, afficheurRight);

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

    }
}
