package com.google.blocks.ftcrobotcontroller.runtime;

import android.app.Activity;
import android.webkit.ConsoleMessage;
import android.webkit.JavascriptInterface;
import android.webkit.WebChromeClient;
import android.webkit.WebView;
import com.android.tools.r8.internal.HB$$ExternalSyntheticBackportWithForwarding0;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItem;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareItemMap;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareType;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TensorFlowAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodCurrentGameAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodCustomModelAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodRoverRuckusAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodSkyStoneAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaCurrentGameAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerParametersAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaRelicRecoveryAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaRoverRuckusAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaSkyStoneAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaTrackableAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaTrackableDefaultListenerAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaTrackablesAccess;
import com.google.blocks.ftcrobotcontroller.util.FileUtil;
import com.google.blocks.ftcrobotcontroller.util.Identifier;
import com.google.blocks.ftcrobotcontroller.util.ProjectsUtil;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.util.RobotLog;
import com.sun.tools.doclint.DocLint;
import fi.iki.elonen.NanoHTTPD;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeManager;
import org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

/* JADX INFO: loaded from: classes8.dex */
public final class BlocksOpMode extends LinearOpMode {
    private static final String BLOCK_EXECUTION_ERROR = "Error: Error calling method on NPObject.";
    private static final boolean DEBUG_BLOCKS_EXECUTION = false;
    private static final String LOG_PREFIX = "BlocksOpMode - ";
    private static Activity activity;
    private static WebView webView;
    private volatile boolean currentBlockFinished;
    private volatile String currentBlockFirstName;
    private volatile String currentBlockLastName;
    private volatile BlockType currentBlockType;
    private volatile Thread javaBridgeThread;
    private final String logPrefix;
    private final String project;
    private static final AtomicReference<RuntimeException> fatalExceptionHolder = new AtomicReference<>();
    private static final AtomicReference<String> fatalErrorMessageHolder = new AtomicReference<>();
    private static final AtomicReference<String> nameOfOpModeLoadedIntoWebView = new AtomicReference<>();
    static final Map<String, Access> javascriptInterfaces = new ConcurrentHashMap();
    private final AtomicLong interruptedTime = new AtomicLong();
    private volatile boolean forceStopped = false;
    private volatile boolean wasTerminated = false;

    BlocksOpMode(String project) {
        this.project = project;
        this.logPrefix = "BlocksOpMode - \"" + project + "\" - ";
    }

    /* JADX INFO: Access modifiers changed from: private */
    public String getLogPrefix() {
        Thread thread = Thread.currentThread();
        return this.logPrefix + thread.getThreadGroup().getName() + OnBotJavaFileSystemUtils.PATH_SEPARATOR + thread.getName() + " - ";
    }

    void startBlockExecution(BlockType blockType, String blockFirstName, String blockLastName) {
        this.currentBlockType = blockType;
        this.currentBlockFirstName = blockFirstName;
        this.currentBlockLastName = blockLastName;
        this.currentBlockFinished = false;
        checkIfStopRequested();
    }

    void endBlockExecution() {
        if (fatalExceptionHolder.get() == null) {
            this.currentBlockFinished = true;
        }
    }

    String getFullBlockLabel() {
        switch (this.currentBlockType) {
            case SPECIAL:
                return this.currentBlockFirstName + this.currentBlockLastName;
            case EVENT:
                return "to " + this.currentBlockFirstName + this.currentBlockLastName;
            case CREATE:
                return "new " + this.currentBlockFirstName;
            case SETTER:
                return "set " + this.currentBlockFirstName + this.currentBlockLastName + " to";
            case GETTER:
                return this.currentBlockFirstName + this.currentBlockLastName;
            case FUNCTION:
                return "call " + this.currentBlockFirstName + this.currentBlockLastName;
            default:
                return "to runOpmode";
        }
    }

    public void handleFatalException(Throwable e) {
        String errorMessage = e.getClass().getSimpleName() + (e.getMessage() != null ? " - " + e.getMessage() : "");
        RuntimeException re = new RuntimeException("Fatal error occurred while executing the block labeled \"" + getFullBlockLabel() + "\". " + errorMessage, e);
        fatalExceptionHolder.set(re);
        throw re;
    }

    private void checkIfStopRequested() {
        if (this.interruptedTime.get() != 0 && isStopRequested() && System.currentTimeMillis() - this.interruptedTime.get() >= 800) {
            RobotLog.i(getLogPrefix() + "checkIfStopRequested - about to stop OpMode by throwing RuntimeException");
            this.forceStopped = true;
            throw new RuntimeException("Stopping OpMode " + this.project + " by force.");
        }
    }

    void waitForStartForBlocks() {
        RobotLog.i(getLogPrefix() + "waitForStartForBlocks - start");
        while (!isStartedForBlocks()) {
            try {
                synchronized (this) {
                    try {
                        try {
                            wait(100L);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            return;
                        }
                    } finally {
                    }
                }
            } finally {
                RobotLog.i(getLogPrefix() + "waitForStartForBlocks - end");
            }
        }
    }

    void sleepForBlocks(long millis) {
        RobotLog.i(getLogPrefix() + "sleepForBlocks - start");
        try {
            long endTime = System.currentTimeMillis() + millis;
            while (!isInterrupted()) {
                long chunk = Math.min(100L, endTime - System.currentTimeMillis());
                if (chunk <= 0) {
                    break;
                } else {
                    sleep(chunk);
                }
            }
        } finally {
            RobotLog.i(getLogPrefix() + "sleepForBlocks - end");
        }
    }

    private boolean isInterrupted() {
        return this.interruptedTime.get() != 0;
    }

    boolean isStartedForBlocks() {
        return super.isStarted() || isInterrupted();
    }

    boolean isStopRequestedForBlocks() {
        return super.isStopRequested() || isInterrupted();
    }

    void terminateOpModeNowForBlocks() {
        this.wasTerminated = true;
        super.terminateOpModeNow();
    }

    @Override // com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
    public void runOpMode() {
        RobotLog.i(getLogPrefix() + "runOpMode - start");
        cleanUpPreviousBlocksOpMode();
        BlocksOpModeCompanion.opMode = this;
        BlocksOpModeCompanion.linearOpMode = this;
        BlocksOpModeCompanion.hardwareMap = this.hardwareMap;
        BlocksOpModeCompanion.telemetry = this.telemetry;
        BlocksOpModeCompanion.gamepad1 = this.gamepad1;
        BlocksOpModeCompanion.gamepad2 = this.gamepad2;
        try {
            fatalExceptionHolder.set(null);
            fatalErrorMessageHolder.set(null);
            this.currentBlockType = BlockType.EVENT;
            this.currentBlockFirstName = "";
            this.currentBlockLastName = "runOpMode";
            boolean interrupted = false;
            this.interruptedTime.set(0L);
            AtomicBoolean scriptFinished = new AtomicBoolean();
            Object scriptFinishedLock = new Object();
            BlocksOpModeAccess blocksOpModeAccess = new BlocksOpModeAccess(Identifier.BLOCKS_OP_MODE.identifierForJavaScript, scriptFinishedLock, scriptFinished);
            javascriptInterfaces.put(Identifier.BLOCKS_OP_MODE.identifierForJavaScript, blocksOpModeAccess);
            AppUtil appUtil = AppUtil.getInstance();
            synchronized (scriptFinishedLock) {
                appUtil.runOnUiThread(new Runnable() { // from class: com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode.1
                    @Override // java.lang.Runnable
                    public void run() {
                        try {
                            RobotLog.i(BlocksOpMode.this.getLogPrefix() + "run1 - before loadScript");
                            BlocksOpMode.this.loadScript();
                            RobotLog.i(BlocksOpMode.this.getLogPrefix() + "run1 - after loadScript");
                        } catch (Exception e) {
                            RobotLog.e(BlocksOpMode.this.getLogPrefix() + "run1 - caught " + e);
                            if (e.getStackTrace() != null) {
                                RobotLog.logStackTrace(e);
                            }
                        }
                    }
                });
                RobotLog.i(getLogPrefix() + "runOpMode - before while !scriptFinished loop");
                while (!scriptFinished.get()) {
                    try {
                        scriptFinishedLock.wait();
                    } catch (InterruptedException e) {
                        RobotLog.e(getLogPrefix() + "runOpMode - caught InterruptedException during scriptFinishedLock.wait");
                        interrupted = true;
                        this.interruptedTime.set(System.currentTimeMillis());
                        if (this.javaBridgeThread != null) {
                            this.javaBridgeThread.interrupt();
                        }
                    }
                }
                RobotLog.i(getLogPrefix() + "runOpMode - after while !scriptFinished loop");
            }
            appUtil.runOnUiThread(new Runnable() { // from class: com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode.2
                @Override // java.lang.Runnable
                public void run() {
                    try {
                        RobotLog.i(BlocksOpMode.this.getLogPrefix() + "run2 - before clearScript");
                        BlocksOpMode.this.clearScript();
                        RobotLog.i(BlocksOpMode.this.getLogPrefix() + "run2 - after clearScript");
                    } catch (Exception e2) {
                        RobotLog.e(BlocksOpMode.this.getLogPrefix() + "run2 - caught " + e2);
                        if (e2.getStackTrace() != null) {
                            RobotLog.logStackTrace(e2);
                        }
                    }
                }
            });
            if (interrupted) {
                Thread.currentThread().interrupt();
            }
            RuntimeException fatalException = fatalExceptionHolder.getAndSet(null);
            if (fatalException != null) {
                throw fatalException;
            }
            String fatalErrorMessage = fatalErrorMessageHolder.getAndSet(null);
            if (fatalErrorMessage != null) {
                RobotLog.setGlobalErrorMsg(fatalErrorMessage);
            }
        } finally {
            long interruptedTime = this.interruptedTime.get();
            if (interruptedTime != 0) {
                RobotLog.i(getLogPrefix() + "runOpMode - end - " + (System.currentTimeMillis() - interruptedTime) + "ms after InterruptedException");
            } else {
                RobotLog.i(getLogPrefix() + "runOpMode - end - no InterruptedException");
            }
            BlocksOpModeCompanion.opMode = null;
            BlocksOpModeCompanion.linearOpMode = null;
        }
    }

    private void cleanUpPreviousBlocksOpMode() {
        String name = nameOfOpModeLoadedIntoWebView.get();
        if (name != null) {
            RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Warning: The Blocks runtime system is still loaded with the Blocks OpMode named " + name + ".");
            RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Trying to clean up now.");
            AppUtil.getInstance().synchronousRunOnUiThread(new Runnable() { // from class: com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode.3
                @Override // java.lang.Runnable
                public void run() {
                    try {
                        RobotLog.w(BlocksOpMode.this.getLogPrefix() + "cleanUpPreviousBlocksOpMode run - before clearScript");
                        BlocksOpMode.this.clearScript();
                        RobotLog.w(BlocksOpMode.this.getLogPrefix() + "cleanUpPreviousBlocksOpMode run - after clearScript");
                    } catch (Exception e) {
                        RobotLog.e(BlocksOpMode.this.getLogPrefix() + "cleanUpPreviousBlocksOpMode run - caught " + e);
                        if (e.getStackTrace() != null) {
                            RobotLog.logStackTrace(e);
                        }
                    }
                }
            });
            if (nameOfOpModeLoadedIntoWebView.get() != null) {
                RobotLog.w(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Clean up was successful.");
            } else {
                RobotLog.e(getLogPrefix() + "cleanUpPreviousBlocksOpMode - Error: Clean up failed.");
                throw new RuntimeException("Unable to start running the Blocks OpMode named " + this.project + ". The Blocks runtime system is still loaded with the previous Blocks OpMode named " + name + ". Please restart the Robot Controller app.");
            }
        }
    }

    private void addJavascriptInterfaces(HardwareItemMap hardwareItemMap, Set<String> identifiersUsed) {
        addJavascriptInterfacesForIdentifiers();
        addObsoleteJavascriptInterfaces();
        for (Identifier identifier : Identifier.values()) {
            if (!javascriptInterfaces.containsKey(identifier.identifierForJavaScript)) {
                throw new RuntimeException("There is no javascript interface for Identifier." + identifier);
            }
        }
        addJavascriptInterfacesForHardware(hardwareItemMap, identifiersUsed);
        for (Map.Entry<String, Access> entry : javascriptInterfaces.entrySet()) {
            String identifier2 = entry.getKey();
            Access access = entry.getValue();
            webView.addJavascriptInterface(access, identifier2);
        }
    }

    private void addJavascriptInterface(String identifier, Access access) {
        if (javascriptInterfaces.containsKey(identifier)) {
            throw new RuntimeException("Duplicate identifier: " + identifier);
        }
        javascriptInterfaces.put(identifier, access);
    }

    void addJavascriptInterfacesForIdentifiers() {
        addJavascriptInterface(Identifier.ACCELERATION.identifierForJavaScript, new AccelerationAccess(this, Identifier.ACCELERATION.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANDROID_ACCELEROMETER.identifierForJavaScript, new AndroidAccelerometerAccess(this, Identifier.ANDROID_ACCELEROMETER.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANDROID_GYROSCOPE.identifierForJavaScript, new AndroidGyroscopeAccess(this, Identifier.ANDROID_GYROSCOPE.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANDROID_ORIENTATION.identifierForJavaScript, new AndroidOrientationAccess(this, Identifier.ANDROID_ORIENTATION.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANDROID_SOUND_POOL.identifierForJavaScript, new AndroidSoundPoolAccess(this, Identifier.ANDROID_SOUND_POOL.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANDROID_TEXT_TO_SPEECH.identifierForJavaScript, new AndroidTextToSpeechAccess(this, Identifier.ANDROID_TEXT_TO_SPEECH.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANDYMARK_IMU_ORIENTATION_ON_ROBOT.identifierForJavaScript, new AndyMarkIMUOrientationOnRobotAccess(this, Identifier.ANDYMARK_IMU_ORIENTATION_ON_ROBOT.identifierForJavaScript));
        addJavascriptInterface(Identifier.ANGULAR_VELOCITY.identifierForJavaScript, new AngularVelocityAccess(this, Identifier.ANGULAR_VELOCITY.identifierForJavaScript));
        addJavascriptInterface(Identifier.APRIL_TAG.identifierForJavaScript, new AprilTagAccess(this, Identifier.APRIL_TAG.identifierForJavaScript));
        addJavascriptInterface(Identifier.BLACKBOARD.identifierForJavaScript, new BlackboardAccess(this, Identifier.BLACKBOARD.identifierForJavaScript, OpMode.blackboard));
        addJavascriptInterface(Identifier.BLINKIN_PATTERN.identifierForJavaScript, new BlinkinPatternAccess(this, Identifier.BLINKIN_PATTERN.identifierForJavaScript));
        addJavascriptInterface(Identifier.BNO055IMU_PARAMETERS.identifierForJavaScript, new BNO055IMUParametersAccess(this, Identifier.BNO055IMU_PARAMETERS.identifierForJavaScript));
        addJavascriptInterface(Identifier.COLOR.identifierForJavaScript, new ColorAccess(this, Identifier.COLOR.identifierForJavaScript, activity));
        addJavascriptInterface(Identifier.COLOR_BLOB_LOCATOR.identifierForJavaScript, new ColorBlobLocatorAccess(this, Identifier.COLOR_BLOB_LOCATOR.identifierForJavaScript));
        addJavascriptInterface(Identifier.DBG_LOG.identifierForJavaScript, new DbgLogAccess(this, Identifier.DBG_LOG.identifierForJavaScript));
        addJavascriptInterface(Identifier.ELAPSED_TIME.identifierForJavaScript, new ElapsedTimeAccess(this, Identifier.ELAPSED_TIME.identifierForJavaScript));
        addJavascriptInterface(Identifier.EXPOSURE_CONTROL.identifierForJavaScript, new ExposureControlAccess(this, Identifier.EXPOSURE_CONTROL.identifierForJavaScript));
        addJavascriptInterface(Identifier.FOCUS_CONTROL.identifierForJavaScript, new FocusControlAccess(this, Identifier.FOCUS_CONTROL.identifierForJavaScript));
        addJavascriptInterface(Identifier.GAIN_CONTROL.identifierForJavaScript, new GainControlAccess(this, Identifier.GAIN_CONTROL.identifierForJavaScript));
        addJavascriptInterface(Identifier.GAMEPAD_1.identifierForJavaScript, new GamepadAccess(this, Identifier.GAMEPAD_1.identifierForJavaScript, this.gamepad1));
        addJavascriptInterface(Identifier.GAMEPAD_2.identifierForJavaScript, new GamepadAccess(this, Identifier.GAMEPAD_2.identifierForJavaScript, this.gamepad2));
        addJavascriptInterface(Identifier.IMU_PARAMETERS.identifierForJavaScript, new ImuParametersAccess(this, Identifier.IMU_PARAMETERS.identifierForJavaScript));
        addJavascriptInterface(Identifier.LED_EFFECT.identifierForJavaScript, new LedEffectAccess(this, Identifier.LED_EFFECT.identifierForJavaScript));
        addJavascriptInterface(Identifier.LL_RESULT.identifierForJavaScript, new LLResultAccess(this, Identifier.LL_RESULT.identifierForJavaScript));
        addJavascriptInterface(Identifier.LL_STATUS.identifierForJavaScript, new LLStatusAccess(this, Identifier.LL_STATUS.identifierForJavaScript));
        addJavascriptInterface(Identifier.LINEAR_OP_MODE.identifierForJavaScript, new LinearOpModeAccess(this, Identifier.LINEAR_OP_MODE.identifierForJavaScript, this.project));
        addJavascriptInterface(Identifier.MAGNETIC_FLUX.identifierForJavaScript, new MagneticFluxAccess(this, Identifier.MAGNETIC_FLUX.identifierForJavaScript));
        addJavascriptInterface(Identifier.MATRIX_F.identifierForJavaScript, new MatrixFAccess(this, Identifier.MATRIX_F.identifierForJavaScript));
        addJavascriptInterface(Identifier.MISC.identifierForJavaScript, new MiscAccess(this, Identifier.MISC.identifierForJavaScript, this.hardwareMap));
        addJavascriptInterface(Identifier.NAVIGATION.identifierForJavaScript, new NavigationAccess(this, Identifier.NAVIGATION.identifierForJavaScript, this.hardwareMap));
        addJavascriptInterface(Identifier.OPENCV.identifierForJavaScript, new OpencvAccess(this, Identifier.OPENCV.identifierForJavaScript));
        addJavascriptInterface(Identifier.OPEN_GL_MATRIX.identifierForJavaScript, new OpenGLMatrixAccess(this, Identifier.OPEN_GL_MATRIX.identifierForJavaScript));
        addJavascriptInterface(Identifier.ORIENTATION.identifierForJavaScript, new OrientationAccess(this, Identifier.ORIENTATION.identifierForJavaScript));
        addJavascriptInterface(Identifier.PIDF_COEFFICIENTS.identifierForJavaScript, new PIDFCoefficientsAccess(this, Identifier.PIDF_COEFFICIENTS.identifierForJavaScript));
        addJavascriptInterface(Identifier.POSE2D.identifierForJavaScript, new Pose2DAccess(this, Identifier.POSE2D.identifierForJavaScript));
        addJavascriptInterface(Identifier.POSITION.identifierForJavaScript, new PositionAccess(this, Identifier.POSITION.identifierForJavaScript));
        addJavascriptInterface(Identifier.PREDOMINANT_COLOR.identifierForJavaScript, new PredominantColorAccess(this, Identifier.PREDOMINANT_COLOR.identifierForJavaScript));
        addJavascriptInterface(Identifier.PTZ_CONTROL.identifierForJavaScript, new PtzControlAccess(this, Identifier.PTZ_CONTROL.identifierForJavaScript));
        addJavascriptInterface(Identifier.QUATERNION.identifierForJavaScript, new QuaternionAccess(this, Identifier.QUATERNION.identifierForJavaScript));
        addJavascriptInterface(Identifier.RANGE.identifierForJavaScript, new RangeAccess(this, Identifier.RANGE.identifierForJavaScript));
        addJavascriptInterface(Identifier.REV_HUB_ORIENTATION_ON_ROBOT.identifierForJavaScript, new RevHubOrientationOnRobotAccess(this, Identifier.REV_HUB_ORIENTATION_ON_ROBOT.identifierForJavaScript));
        addJavascriptInterface(Identifier.REV_9AXIS_IMU_ORIENTATION_ON_ROBOT.identifierForJavaScript, new Rev9AxisImuOrientationOnRobotAccess(this, Identifier.REV_9AXIS_IMU_ORIENTATION_ON_ROBOT.identifierForJavaScript));
        addJavascriptInterface(Identifier.RUMBLE_EFFECT.identifierForJavaScript, new RumbleEffectAccess(this, Identifier.RUMBLE_EFFECT.identifierForJavaScript));
        addJavascriptInterface(Identifier.SYSTEM.identifierForJavaScript, new SystemAccess(this, Identifier.SYSTEM.identifierForJavaScript));
        addJavascriptInterface(Identifier.TELEMETRY.identifierForJavaScript, new TelemetryAccess(this, Identifier.TELEMETRY.identifierForJavaScript, this.telemetry));
        addJavascriptInterface(Identifier.TEMPERATURE.identifierForJavaScript, new TemperatureAccess(this, Identifier.TEMPERATURE.identifierForJavaScript));
        addJavascriptInterface(Identifier.VECTOR_F.identifierForJavaScript, new VectorFAccess(this, Identifier.VECTOR_F.identifierForJavaScript));
        addJavascriptInterface(Identifier.VELOCITY.identifierForJavaScript, new VelocityAccess(this, Identifier.VELOCITY.identifierForJavaScript));
        addJavascriptInterface(Identifier.VISION_PORTAL.identifierForJavaScript, new VisionPortalAccess(this, Identifier.VISION_PORTAL.identifierForJavaScript, this.hardwareMap));
        addJavascriptInterface(Identifier.WHITE_BALANCE_CONTROL.identifierForJavaScript, new WhiteBalanceControlAccess(this, Identifier.WHITE_BALANCE_CONTROL.identifierForJavaScript));
        addJavascriptInterface(Identifier.YAW_PITCH_ROLL_ANGLES.identifierForJavaScript, new YawPitchRollAnglesAccess(this, Identifier.YAW_PITCH_ROLL_ANGLES.identifierForJavaScript));
    }

    void addObsoleteJavascriptInterfaces() {
        addJavascriptInterface(Identifier.OBSOLETE_TENSOR_FLOW.identifierForJavaScript, new TensorFlowAccess(this, Identifier.OBSOLETE_TENSOR_FLOW.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_TFOD.identifierForJavaScript, new TfodAccess(this, Identifier.OBSOLETE_TFOD.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_TFOD_CURRENT_GAME.identifierForJavaScript, new TfodCurrentGameAccess(this, Identifier.OBSOLETE_TFOD_CURRENT_GAME.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_TFOD_CUSTOM_MODEL.identifierForJavaScript, new TfodCustomModelAccess(this, Identifier.OBSOLETE_TFOD_CUSTOM_MODEL.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_TFOD_ROVER_RUCKUS.identifierForJavaScript, new TfodRoverRuckusAccess(this, Identifier.OBSOLETE_TFOD_ROVER_RUCKUS.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_TFOD_SKY_STONE.identifierForJavaScript, new TfodSkyStoneAccess(this, Identifier.OBSOLETE_TFOD_SKY_STONE.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_CURRENT_GAME.identifierForJavaScript, new VuforiaCurrentGameAccess(this, Identifier.OBSOLETE_VUFORIA_CURRENT_GAME.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_RELIC_RECOVERY.identifierForJavaScript, new VuforiaRelicRecoveryAccess(this, Identifier.OBSOLETE_VUFORIA_RELIC_RECOVERY.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_ROVER_RUCKUS.identifierForJavaScript, new VuforiaRoverRuckusAccess(this, Identifier.OBSOLETE_VUFORIA_ROVER_RUCKUS.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_SKY_STONE.identifierForJavaScript, new VuforiaSkyStoneAccess(this, Identifier.OBSOLETE_VUFORIA_SKY_STONE.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_LOCALIZER.identifierForJavaScript, new VuforiaLocalizerAccess(this, Identifier.OBSOLETE_VUFORIA_LOCALIZER.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_LOCALIZER_PARAMETERS.identifierForJavaScript, new VuforiaLocalizerParametersAccess(this, Identifier.OBSOLETE_VUFORIA_LOCALIZER_PARAMETERS.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_TRACKABLE.identifierForJavaScript, new VuforiaTrackableAccess(this, Identifier.OBSOLETE_VUFORIA_TRACKABLE.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_TRACKABLE_DEFAULT_LISTENER.identifierForJavaScript, new VuforiaTrackableDefaultListenerAccess(this, Identifier.OBSOLETE_VUFORIA_TRACKABLE_DEFAULT_LISTENER.identifierForJavaScript));
        addJavascriptInterface(Identifier.OBSOLETE_VUFORIA_TRACKABLES.identifierForJavaScript, new VuforiaTrackablesAccess(this, Identifier.OBSOLETE_VUFORIA_TRACKABLES.identifierForJavaScript));
    }

    private void addJavascriptInterfacesForHardware(HardwareItemMap hardwareItemMap, Set<String> identifiersUsed) {
        for (HardwareType hardwareType : HardwareType.values()) {
            if (hardwareItemMap.contains(hardwareType)) {
                for (HardwareItem hardwareItem : hardwareItemMap.getHardwareItems(hardwareType)) {
                    if (identifiersUsed != null && !identifiersUsed.contains(hardwareItem.identifier)) {
                        RobotLog.i(getLogPrefix() + "Skipping hardware device named \"" + hardwareItem.deviceName + "\". It isn't used in this Blocks OpMode.");
                    } else if (javascriptInterfaces.containsKey(hardwareItem.identifier)) {
                        RobotLog.w(getLogPrefix() + "There is already a JavascriptInterface for identifier \"" + hardwareItem.identifier + "\". Ignoring hardware type " + hardwareType + ".");
                    } else {
                        Access access = HardwareAccess.newHardwareAccess(this, hardwareType, this.hardwareMap, hardwareItem);
                        if (access != null) {
                            javascriptInterfaces.put(hardwareItem.identifier, access);
                        }
                    }
                }
            }
        }
    }

    private void removeJavascriptInterfaces() {
        Iterator<Map.Entry<String, Access>> it = javascriptInterfaces.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, Access> entry = it.next();
            String identifier = entry.getKey();
            Access access = entry.getValue();
            webView.removeJavascriptInterface(identifier);
            access.close();
            it.remove();
        }
    }

    private class BlocksOpModeAccess extends Access {
        private final AtomicBoolean scriptFinished;
        private final Object scriptFinishedLock;

        private BlocksOpModeAccess(String identifier, Object scriptFinishedLock, AtomicBoolean scriptFinished) {
            super(BlocksOpMode.this, identifier, "");
            this.scriptFinishedLock = scriptFinishedLock;
            this.scriptFinished = scriptFinished;
        }

        @JavascriptInterface
        public void scriptStarting() {
            RobotLog.i(BlocksOpMode.this.getLogPrefix() + "scriptStarting");
            Thread.interrupted();
            BlocksOpMode.this.javaBridgeThread = Thread.currentThread();
        }

        @JavascriptInterface
        public void caughtException(String message, String currentBlockLabel) {
            String wrongImuErrorMessage;
            if (BlocksOpMode.this.wasTerminated) {
                return;
            }
            if (message != null) {
                if (!message.startsWith("ReferenceError: ") || !message.endsWith(" is not defined")) {
                    if (BlocksOpMode.this.forceStopped) {
                        AppUtil.getInstance().showAlertDialog(UILocation.BOTH, "OpMode Force-Stopped", "User OpMode was stuck in stop(), but was able to be force stopped without restarting the app. Please make sure you are calling opModeInInit() or opModeIsActive() in any loops!");
                        return;
                    } else if (currentBlockLabel == null || currentBlockLabel.isEmpty()) {
                        if (BlocksOpMode.this.currentBlockFinished) {
                            HB$$ExternalSyntheticBackportWithForwarding0.m(BlocksOpMode.fatalErrorMessageHolder, null, "Fatal error occurred after executing the block labeled \"" + BlocksOpMode.this.getFullBlockLabel() + "\". " + message);
                        } else {
                            HB$$ExternalSyntheticBackportWithForwarding0.m(BlocksOpMode.fatalErrorMessageHolder, null, "Fatal error occurred while executing the block labeled \"" + BlocksOpMode.this.getFullBlockLabel() + "\". " + message);
                        }
                    } else {
                        HB$$ExternalSyntheticBackportWithForwarding0.m(BlocksOpMode.fatalErrorMessageHolder, null, "Fatal error occurred while executing the block labeled \"" + currentBlockLabel + "\". " + message);
                    }
                } else {
                    String missingIdentifier = message.substring(16, message.length() - 15);
                    String errorMessage = "Could not find identifier: " + missingIdentifier;
                    String missingHardwareDeviceName = BlocksOpMode.missingIdentifierToHardwareDeviceName(missingIdentifier);
                    if (missingHardwareDeviceName != null) {
                        errorMessage = "Could not find hardware device: " + missingHardwareDeviceName;
                        if (missingIdentifier.endsWith(HardwareType.BNO055IMU.identifierSuffixForJavaScript) && BlocksOpMode.this.hardwareMap.getAllNames(BNO055IMUImpl.class).isEmpty() && (wrongImuErrorMessage = getWrongImuErrorMessage()) != null) {
                            errorMessage = errorMessage + "\n\n" + wrongImuErrorMessage;
                        }
                    }
                    HB$$ExternalSyntheticBackportWithForwarding0.m(BlocksOpMode.fatalErrorMessageHolder, null, errorMessage);
                    return;
                }
            }
            RobotLog.e(BlocksOpMode.this.getLogPrefix() + "caughtException - message is " + message);
        }

        private String getWrongImuErrorMessage() {
            LynxModule controlHub = (LynxModule) EmbeddedControlHubModule.get();
            if (controlHub != null) {
                LynxModuleImuType controlHubImuType = controlHub.getImuType();
                if (controlHubImuType == LynxModuleImuType.BHI260) {
                    return "You attempted to use a BNO055 IMU on a Control Hub that contains a BHI260AP IMU. You need to update your OpMode to use the IMU blocks instead of the IMU-BNO055 blocks.";
                }
                return null;
            }
            return null;
        }

        @JavascriptInterface
        public void scriptFinished() {
            RobotLog.i(BlocksOpMode.this.getLogPrefix() + "scriptFinished");
            synchronized (this.scriptFinishedLock) {
                this.scriptFinished.set(true);
                this.scriptFinishedLock.notifyAll();
            }
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static String missingIdentifierToHardwareDeviceName(String identifier) {
        for (HardwareType hardwareType : HardwareType.values()) {
            if (identifier.endsWith(hardwareType.identifierSuffixForJavaScript)) {
                return identifier.substring(0, identifier.length() - hardwareType.identifierSuffixForJavaScript.length());
            }
        }
        return null;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void loadScript() throws IOException {
        RobotLog.i(getLogPrefix() + "loadScript - WebView user agent is \"" + webView.getSettings().getUserAgentString() + "\"");
        nameOfOpModeLoadedIntoWebView.set(this.project);
        HardwareItemMap hardwareItemMap = HardwareItemMap.newHardwareItemMap(this.hardwareMap);
        Set<String> identifiersUsed = null;
        String jsFileContent = ProjectsUtil.fetchJsFileContent(this.project);
        if (jsFileContent.startsWith(HardwareUtil.IDENTIFIERS_USED_PREFIX)) {
            int eol = jsFileContent.indexOf("\n");
            identifiersUsed = new HashSet<>(Arrays.asList(jsFileContent.substring(HardwareUtil.IDENTIFIERS_USED_PREFIX.length(), eol).split(DocLint.TAGS_SEPARATOR)));
            jsFileContent = jsFileContent.substring(eol);
        }
        addJavascriptInterfaces(hardwareItemMap, identifiersUsed);
        String jsContent = HardwareUtil.upgradeJs(jsFileContent, hardwareItemMap);
        StringBuilder html = new StringBuilder().append("<html><body onload='callRunOpMode()'><script type='text/javascript'>\n");
        FileUtil.readAsset(html, activity.getAssets(), "blocks/runtime.js");
        html.append("\n").append(jsContent).append("\n</script></body></html>\n");
        webView.loadDataWithBaseURL(null, html.toString(), NanoHTTPD.MIME_HTML, "UTF-8", null);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void clearScript() {
        removeJavascriptInterfaces();
        if (!javascriptInterfaces.isEmpty()) {
            RobotLog.w(getLogPrefix() + "clearScript - Warning: javascriptInterfaces is not empty.");
        }
        javascriptInterfaces.clear();
        webView.loadDataWithBaseURL(null, "", NanoHTTPD.MIME_HTML, "UTF-8", null);
        nameOfOpModeLoadedIntoWebView.set(null);
    }

    public static void setActivityAndWebView(Activity a, WebView wv) {
        if (activity == null && webView == null) {
            addOpModeRegistrar();
        }
        activity = a;
        webView = wv;
        webView.getSettings().setJavaScriptEnabled(true);
        webView.setWebChromeClient(new WebChromeClient() { // from class: com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode.4
            @Override // android.webkit.WebChromeClient
            public boolean onConsoleMessage(ConsoleMessage consoleMessage) {
                return false;
            }
        });
    }

    private static void addOpModeRegistrar() {
        RegisteredOpModes.getInstance().addInstanceOpModeRegistrar(new InstanceOpModeRegistrar() { // from class: com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode.5
            @Override // org.firstinspires.ftc.robotcore.internal.opmode.InstanceOpModeRegistrar
            public void register(InstanceOpModeManager manager) {
                try {
                    List<OpModeMeta> projects = ProjectsUtil.fetchEnabledProjectsWithJavaScript();
                    for (OpModeMeta opModeMeta : projects) {
                        manager.register(opModeMeta, new BlocksOpMode(opModeMeta.name));
                    }
                } catch (Exception e) {
                    RobotLog.logStackTrace(e);
                }
            }
        });
    }

    @Deprecated
    public static void registerAll(OpModeManager manager) {
        RobotLog.w(BlocksOpMode.class.getSimpleName(), "registerAll(OpModeManager) is deprecated and will be removed soon, as calling it is unnecessary in this and future API version");
    }
}
