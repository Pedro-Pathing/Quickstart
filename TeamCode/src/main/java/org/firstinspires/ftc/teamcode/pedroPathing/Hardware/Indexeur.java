package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexeur {
    private boolean rotationPourAmassage = false;

    // Timer d'erreur (temps passé avec une erreur >= tolérance)
    private final ElapsedTime erreurTimer = new ElapsedTime();
    // Durée max erreur avant fin (avancerrapide)
    private static final double ERREUR_TIMEOUT_S = 2.0; // 1 à 2 secondes
    private boolean rotationPourTir = false;
    private String capteurDetecteur = "Aucun";   // "Left", "Right", "Fusion"
    private float hueDetectee = -1;              // Hue retenue




    private DcMotorEx indexeur;
    private Intake intake;

    public void setIntake(Intake intake) { this.intake = intake; }
    private DigitalChannel magSensorAv;
    private Rev2mDistanceSensor distSensorIndexeur;
    private Rev2mDistanceSensor distSensorIndexeur2;
    private NormalizedColorSensor ColorLeft;
    private NormalizedColorSensor ColorRight;
    private String couleurDetecteeTemp = "Inconnue";
    double hue;
    private static final double TICKS_PER_REV_43 = 3895.9; // GoBilda 5203 43 tours

    private int vitessehomingindexeurRPM = 10;
    private int vitesserapideindexeurRPM = 25;
    private static final int COMPARTIMENTS = 3;
    private int positionLogique = 0;
    private int compartimentPhysique = 0;
    private static final double TICKS_PAR_COMPARTIMENT = TICKS_PER_REV_43 / COMPARTIMENTS;
    private int compartimentActuel = 0;

    int ballComptage = 0;
    int MAX_BALLS = 3;
    int MIN_BALLS = 0;


    // Variable interne pour mémoriser l'état précédent
    private boolean lastBallDetected = false;
    private boolean homingDone = false;
    private boolean homingDemarre = false;
    private boolean marcheForceeIndexeur = false;
    private int consecutiveDetections = 0;
    private static final int NB_LECTURES = 5;

    private int targetTicks = 0;
    private boolean rotationEnCours = false;
    private int erreurindexeur = 30;

    public enum Indexeuretat {
        IDLE,
        RECHERCHEPALE,

        AVANCERAPIDETIR,
        AVANCERAPIDEAMASSAGE,
        BOURRAGE,
        STABILISATION,
        PRETPOURTIR,

        HOMING
    }

    private Indexeuretat IndexeurState = Indexeuretat.IDLE;
    private ElapsedTime timeretat = new ElapsedTime();
    private ElapsedTime indexeurtimer = new ElapsedTime();

    private ElapsedTime bourragetimer = new ElapsedTime();
    private int SEUIL_MMDETECTION = 5; //seuil detection capteur distance

    private String[] couleurBalleDansCompartiment = new String[COMPARTIMENTS];

    public void init(@NonNull HardwareMap hwMap) {

        indexeur = hwMap.get(DcMotorEx.class, "Indexeur");
        indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexeur.setDirection(DcMotor.Direction.REVERSE);

        magSensorAv = hwMap.get(DigitalChannel.class, "magSensorAv");
        magSensorAv.setMode(DigitalChannel.Mode.INPUT);

        // ColorLeft = hwMap.get(ColorSensor.class, "ColorLeft");
        ColorLeft = hwMap.get(NormalizedColorSensor.class, "ColorLeft");
        ColorRight = hwMap.get(NormalizedColorSensor.class, "ColorRight");
        for (int i = 0; i < COMPARTIMENTS; i++) {
            couleurBalleDansCompartiment[i] = "Inconnue";
        }

    }

    public void update() {
        // Détection couleur déclenchée par la balle
        if (intake.getBalleDetectee()) {
            String c = detectBallColor();
            if (!"Inconnue".equals(c)) {
                couleurDetecteeTemp = c;   // capture anticipée
            }
        }


        // --- PRIORITÉ ABSOLUE : lancer le homing une seule fois ---
        if (!homingDemarre) {
            IndexeurState = Indexeuretat.HOMING;
            homingDemarre = true;
        }

        updateRotation(); // on accelere la rotation apres son enclenchement

        switch (IndexeurState) {

            case HOMING:
                homingIndexeur();
                break;
            case IDLE:
                indexeur.setPower(0);
                indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            break;

            case RECHERCHEPALE:
                if (!detectionpale() && ballComptage < MAX_BALLS) {
                    setindexeurTargetRPM(6);
                } else if (detectionpale()) {
                    setindexeurTargetRPM(0.0);
                    IndexeurState = Indexeuretat.IDLE;
                }
                if (timeretat.milliseconds() > 500) {
                }
                break;

            case AVANCERAPIDETIR:
                rotationPourAmassage = false;
                rotationPourTir = true;
                avancerIndexeurRapide();
                break;
            case AVANCERAPIDEAMASSAGE:
                rotationPourAmassage = true;
                rotationPourTir = false;
                avancerIndexeurRapide();
                break;


            case BOURRAGE:
                reculerIndexeurbourrage();
                break;

            case STABILISATION:
                break;

            case PRETPOURTIR:
                break;




        }
    }

    public double getindexeurVelocityRPM() {
        double ticksPerSec = indexeur.getVelocity();
        return (ticksPerSec * 60) / TICKS_PER_REV_43;
    }

    public void setindexeurTargetRPM(double targetRPM) {
        // Conversion RPM -> ticks/sec
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV_43) / 60.0;
        indexeur.setVelocity(targetTicksPerSec);
    }

    public String detectBallColor() {
        // Lecture capteur gauche
        NormalizedRGBA colorsLeft = ColorLeft.getNormalizedColors();
        float hueLeft = JavaUtil.colorToHue(colorsLeft.toColor());

        // Lecture capteur droit
        NormalizedRGBA colorsRight = ColorRight.getNormalizedColors();
        float hueRight = JavaUtil.colorToHue(colorsRight.toColor());

        // Détection individuelle
        String couleurLeft = detectHueColor(hueLeft);
        String couleurRight = detectHueColor(hueRight);

        // --- FUSION + identification du capteur ---
        if (couleurLeft.equals("Vert") || couleurRight.equals("Vert")) {
            if (couleurLeft.equals("Vert")) {
                capteurDetecteur = "Left";
                hueDetectee = hueLeft;
            } else {
                capteurDetecteur = "Right";
                hueDetectee = hueRight;
            }
            return "Vert";
        }

        if (couleurLeft.equals("Violet") || couleurRight.equals("Violet")) {
            if (couleurLeft.equals("Violet")) {
                capteurDetecteur = "Left";
                hueDetectee = hueLeft;
            } else {
                capteurDetecteur = "Right";
                hueDetectee = hueRight;
            }
            return "Violet";
        }

        // Aucun capteur n'a détecté de couleur
        capteurDetecteur = "Aucun";
        hueDetectee = -1;
        return "Inconnue";
    }


    // Fonction auxiliaire : convertit Hue en Vert/Violet/Inconnue
    private String detectHueColor(float hue) {
        if (hue >= 90 && hue <= 150) {
            return "Vert";
        } else if (hue >= 230 && hue <= 330) {
            return "Violet";
        } else {
            return "Inconnue";
        }
    }

    // Méthode de homing à appeler au démarrage
    public void homingIndexeur() {
        //if (homingDone) { //IndexeurState = Indexeuretat.IDLE;
        //}
        // Toujours en RUN_USING_ENCODER pour le homing
        indexeur.setPower(0.2);
        // si l’aimant est détecté (via detectionpale)
        if (detectionpale()) {
            indexeur.setPower(0.0); // arrêt
            indexeur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // mode normal
            homingDone = true;
            IndexeurState = Indexeuretat.IDLE;

            }
            // Sécurité : si ça tourne trop longtemps
            //if (timeretat.seconds() > 3.0) {
            //    indexeur.setPower(0);
            //    homingDone = true;




    }

    public int getBalles() {
        return ballComptage;
    }

    // Fast advance by one compartment using RUN_TO_POSITION

    public void avancerIndexeurRapide() {

        if (!homingDone || rotationEnCours) return;

        int current = indexeur.getCurrentPosition();
        int currentSlot = (int) Math.round(current / TICKS_PAR_COMPARTIMENT);
        int nextSlot = currentSlot + 1; // avance d’un compartiment
        targetTicks = (int) Math.round(nextSlot * TICKS_PAR_COMPARTIMENT);

        // Mets à jour la logique d’après ce nextSlot
        positionLogique = nextSlot;
        compartimentPhysique = Math.floorMod(positionLogique, COMPARTIMENTS);

        indexeur.setTargetPosition(targetTicks);
        indexeur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexeur.setPower(0.5);

        indexeurtimer.reset();
        erreurTimer.reset();
        rotationEnCours = true;

    }

    public void updateRotation() {

        if (!rotationEnCours) return;

        int posActuelle = indexeur.getCurrentPosition();
        int erreur = Math.abs(posActuelle - targetTicks);

        // --- LECTURE CONTINUE DE LA COULEUR ---
        String couleurInstant = detectBallColor();
        if (!"Inconnue".equals(couleurInstant)) {
            couleurDetecteeTemp = couleurInstant;   // meilleure couleur vue
        }

        // --- Gestion de la vitesse progressive ---
        if (erreur > 300) {
            indexeur.setPower(0.8);   // loin de la cible → rapide
        } else {
            indexeur.setPower(0.5);   // proche → lent et stable
        }

        // --- RESET DU TIMER D’ERREUR SI ON EST DANS LA TOLÉRANCE ---
        if (erreur < erreurindexeur) {
            erreurTimer.reset();
        }

        // --- CONDITIONS DE FIN ---
        boolean finNormale = !indexeur.isBusy();                // RUN_TO_POSITION estime que c'est fini
        boolean stabiliseDansTol = (erreur < erreurindexeur)    // erreur ok
                && (erreurTimer.seconds() > 0.15); // stabilité 150ms
        boolean finParTimerErreur = (erreur >= erreurindexeur)
                && (erreurTimer.seconds() > ERREUR_TIMEOUT_S);

        if (!(finNormale || stabiliseDansTol || finParTimerErreur)) {
            return;   // pas fini → on continue
        }

        // --- ROTATION FINIE ---
        indexeur.setPower(0);
        indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // === RECALAGE LOGIQUE SUR ENCODEUR (au bon moment !) ===
        int pos = indexeur.getCurrentPosition();
        int logicalSlot = (int) Math.round(pos / TICKS_PAR_COMPARTIMENT);

        positionLogique = logicalSlot;
        compartimentPhysique = Math.floorMod(positionLogique, COMPARTIMENTS);

        // --- CALCUL DES SLOTS ---
        int slotCapteurs = compartimentPhysique;
        int slotTir      = (compartimentPhysique + 1) % COMPARTIMENTS;
        int slotEntree   = (compartimentPhysique + 2) % COMPARTIMENTS;

        // --- UTILISATION DE LA MEILLEURE COULEUR ---
        String couleurSousCapteurs = couleurDetecteeTemp;
        couleurDetecteeTemp = "Inconnue"; // reset pour prochaine rotation

        couleurBalleDansCompartiment[slotCapteurs] = couleurSousCapteurs;

        // --- COMPTAGE ---
        if (rotationPourAmassage) {

            ballComptage = Math.min(ballComptage + 1, MAX_BALLS);

            if (ballComptage < 3) {
                intake.setetatramasage();
            } else {
                intake.setetatIDLE();
            }

        } else if (rotationPourTir) {

            couleurBalleDansCompartiment[slotTir] = "Inconnue";
            // (décrément optionnel selon ta logique)
        }

        // --- RESET FLAGS ---
        rotationPourAmassage = false;
        rotationPourTir = false;
        rotationEnCours = false;

        // --- SORTIE D’ÉTAT ---
        IndexeurState = finParTimerErreur ?
                Indexeuretat.BOURRAGE :
                Indexeuretat.IDLE;

        if (IndexeurState == Indexeuretat.BOURRAGE) {
            bourragetimer.reset();
        }

    }

    public void reculerIndexeurbourrage() {

        // Initialisation au moment d’entrer dans l’état BOURRAGE
        if (bourragetimer.milliseconds() == 0) {
            bourragetimer.reset();
            indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            indexeur.setPower(-0.4);  // recul court pour libérer la balle
        }

        // Recul pendant 600 ms
        if (bourragetimer.milliseconds() > 600) {

            // Stop
            indexeur.setPower(0);
            indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            bourragetimer.reset();

            // Si on a encore de la place → continuer l’amassage automatiquement
            if (ballComptage < MAX_BALLS) {
                rotationPourAmassage = true;
                rotationEnCours = false;
                IndexeurState = Indexeuretat.AVANCERAPIDEAMASSAGE;
            }
            // Sinon → rester tranquille
            else {
                IndexeurState = Indexeuretat.IDLE;
            }
        }
    }

    public boolean avanceTerminee() {
        boolean blocage = indexeur.isBusy() && Math.abs(indexeur.getVelocity()) < 10;
        boolean fini = !indexeur.isBusy() ||
                // le moteur pense avoir atteint sa cible
                Math.abs(indexeur.getCurrentPosition() - indexeur.getTargetPosition()) <= indexeur.getTargetPositionTolerance() || // proche de la cible
                indexeurtimer.seconds() > 1.0; // sécurité : timeout 1s
        if (fini) {
            indexeur.setPower(0.0);
            indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return fini;
    }

    public boolean detectionpale() {
        return !magSensorAv.getState();
    }
    public boolean isindexeurBusy() {
        return indexeur.isBusy(); }
    /** * Getter pour accéder directement au moteur si besoin */
    public DcMotor getindexeurMotor() {
        return indexeur;}
    private static final int TOLERANCE_TIR = 15;
    public boolean indexeurPretPourTir() {
        if (indexeur == null)
            return false;
        boolean fini = !indexeur.isBusy();
        int erreur = Math.abs(indexeur.getTargetPosition() - indexeur.getCurrentPosition());
        boolean dansTol = erreur < TOLERANCE_TIR;
        return fini && dansTol; }


    public String getCouleurCompartiment(int compartiment) {
        return couleurBalleDansCompartiment[compartiment];
    }


    public Indexeuretat getEtat() {
        return IndexeurState;
    }

    public void setEtat(Indexeuretat nouvelEtat) {

        // Empêche l’intake d'interrompre le homing
        if (IndexeurState == Indexeuretat.HOMING &&
                nouvelEtat == Indexeuretat.AVANCERAPIDEAMASSAGE) { //||
                        //nouvelEtat == Indexeuretat.AVANCERAPIDETIR)) {
            return;
        }

        // Empêche l’intake de spammer pendant une rotation
        if (rotationEnCours &&
                nouvelEtat == Indexeuretat.AVANCERAPIDEAMASSAGE) //||
                        //nouvelEtat == Indexeuretat.AVANCERAPIDETIR))
            {
            return;
        }

        this.IndexeurState = nouvelEtat;
    }
    // --- Appelé par le TireurManager pour avancer une balle vers le tir ---
    public void avancerPourTir() {
        rotationPourAmassage = false;
        rotationPourTir = true;
        setEtat(Indexeuretat.AVANCERAPIDETIR);
    }

    // --- Appelé par le TireurManager pour savoir si l'avance est finie ---
    public boolean isRotationTerminee() {
        // On se base sur la logique déjà existante
        return !rotationEnCours || indexeurPretPourTir();
    }
    public void forcerHomingTermine() {
        homingDone = true;
        IndexeurState = Indexeuretat.IDLE;
    }
    public void lancerHoming() {
        IndexeurState = Indexeuretat.HOMING;
        homingDemarre = true;
    }
    public boolean isHomingDone() { return homingDone; }

    public void decrementerBalle() {
        ballComptage = Math.max(ballComptage - 1, 0); }


    public void setBalles(int n) {
        ballComptage = Math.max(MIN_BALLS, Math.min(n, MAX_BALLS));
    }

    public void resetCompartiments() {
        for (int i = 0; i < COMPARTIMENTS; i++) {
            couleurBalleDansCompartiment[i] = "Inconnue";
        }
    }


}




