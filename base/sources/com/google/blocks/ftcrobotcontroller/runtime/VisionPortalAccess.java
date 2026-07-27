package com.google.blocks.ftcrobotcontroller.runtime;

import android.util.Size;
import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

/* JADX INFO: loaded from: classes8.dex */
class VisionPortalAccess extends Access {
    private final HardwareMap hardwareMap;

    VisionPortalAccess(BlocksOpMode blocksOpMode, String identifier, HardwareMap hardwareMap) {
        super(blocksOpMode, identifier, "");
        this.hardwareMap = hardwareMap;
    }

    private VisionPortal.Builder checkVisionPortalBuilder(Object visionPortalBuilderArg) {
        return (VisionPortal.Builder) checkArg(visionPortalBuilderArg, VisionPortal.Builder.class, "visionPortalBuilder");
    }

    private VisionPortal.StreamFormat checkStreamFormat(String streamFormatString) {
        return (VisionPortal.StreamFormat) checkArg(streamFormatString, VisionPortal.StreamFormat.class, "streamFormat");
    }

    private VisionProcessor checkVisionProcessor(Object visionProcessorArg) {
        return (VisionProcessor) checkArg(visionProcessorArg, VisionProcessor.class, "visionProcessor");
    }

    private VisionPortal checkVisionPortal(Object visionPortalArg) {
        return (VisionPortal) checkArg(visionPortalArg, VisionPortal.class, "visionPortal");
    }

    private WebcamName checkWebcamName(Object webcamNameArg) {
        return (WebcamName) checkArg(webcamNameArg, WebcamName.class, "webcamName");
    }

    private VisionPortal.MultiPortalLayout checkMultiPortalLayout(String multiPortalLayoutString) {
        return (VisionPortal.MultiPortalLayout) checkArg(multiPortalLayoutString, VisionPortal.MultiPortalLayout.class, "multiPortalLayout");
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, constructor = true)
    public VisionPortal.Builder createBuilder() {
        try {
            startBlockExecution(BlockType.CREATE, "VisionPortal.Builder", "");
            return new VisionPortal.Builder();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setCamera"})
    public void setCamera(Object visionPortalBuilderArg, Object camera) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setCamera");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                if (camera instanceof BuiltinCameraDirection) {
                    visionPortalBuilder.setCamera((BuiltinCameraDirection) camera);
                } else if (camera instanceof WebcamName) {
                    visionPortalBuilder.setCamera((WebcamName) camera);
                } else if (camera instanceof SwitchableCameraName) {
                    visionPortalBuilder.setCamera((SwitchableCameraName) camera);
                } else {
                    reportInvalidArg("camera", "BuiltinCameraName, WebcamName, or SwitchableCameraName");
                }
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setStreamFormat"})
    public void setStreamFormat(Object visionPortalBuilderArg, String streamFormatString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setStreamFormat");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            VisionPortal.StreamFormat streamFormat = checkStreamFormat(streamFormatString);
            if (visionPortalBuilder != null && streamFormat != null) {
                visionPortalBuilder.setStreamFormat(streamFormat);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"enableLiveView"})
    public void enableLiveView(Object visionPortalBuilderArg, boolean enableLiveView) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".enableLiveView");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                visionPortalBuilder.enableLiveView(enableLiveView);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setAutoStopLiveView"})
    public void setAutoStopLiveView(Object visionPortalBuilderArg, boolean autoStopLiveView) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setAutoStopLiveView");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                visionPortalBuilder.setAutoStopLiveView(autoStopLiveView);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setAutoStartStreamOnBuild"})
    public void setAutoStartStreamOnBuild(Object visionPortalBuilderArg, boolean autoStartStreamOnBuild) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setAutoStartStreamOnBuild");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                visionPortalBuilder.setAutoStartStreamOnBuild(autoStartStreamOnBuild);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setShowStatsOverlay"})
    public void setShowStatsOverlay(Object visionPortalBuilderArg, boolean showStatsOverlay) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setShowStatsOverlay");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                visionPortalBuilder.setShowStatsOverlay(showStatsOverlay);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setLiveViewContainerId"})
    public void setLiveViewContainerId(Object visionPortalBuilderArg, int viewId) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setLiveViewContainerId");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                visionPortalBuilder.setLiveViewContainerId(viewId);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"setCameraResolution"})
    public void setCameraResolution(Object visionPortalBuilderArg, int width, int height) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".setCameraResolution");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                visionPortalBuilder.setCameraResolution(new Size(width, height));
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"addProcessor"})
    public void addProcessor(Object visionPortalBuilderArg, Object visionProcessorArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".addProcessor");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            VisionProcessor visionProcessor = checkVisionProcessor(visionProcessorArg);
            if (visionPortalBuilder != null && visionProcessor != null) {
                visionPortalBuilder.addProcessor(visionProcessor);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.Builder.class}, methodName = {"build"})
    public VisionPortal build(Object visionPortalBuilderArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal.Builder", ".build");
            VisionPortal.Builder visionPortalBuilder = checkVisionPortalBuilder(visionPortalBuilderArg);
            if (visionPortalBuilder != null) {
                return visionPortalBuilder.build();
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"easyCreateWithDefaults"})
    public VisionPortal easyCreateWithDefaults_oneProcessor(Object camera, Object visionProcessorArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".easyCreateWithDefaults");
            VisionProcessor visionProcessor = checkVisionProcessor(visionProcessorArg);
            if (visionProcessor != null) {
                if (camera instanceof BuiltinCameraDirection) {
                    return VisionPortal.easyCreateWithDefaults((BuiltinCameraDirection) camera, visionProcessor);
                }
                if (camera instanceof WebcamName) {
                    return VisionPortal.easyCreateWithDefaults((WebcamName) camera, visionProcessor);
                }
                if (camera instanceof SwitchableCameraName) {
                    return VisionPortal.easyCreateWithDefaults((SwitchableCameraName) camera, visionProcessor);
                }
                reportInvalidArg("camera", "BuiltinCameraName, WebcamName, or SwitchableCameraName");
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"easyCreateWithDefaults"})
    public VisionPortal easyCreateWithDefaults_twoProcessors(Object camera, Object visionProcessor1Arg, Object visionProcessor2Arg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".easyCreateWithDefaults");
            VisionProcessor visionProcessor1 = checkVisionProcessor(visionProcessor1Arg);
            VisionProcessor visionProcessor2 = checkVisionProcessor(visionProcessor2Arg);
            if (visionProcessor1 != null && visionProcessor2 != null) {
                if (camera instanceof BuiltinCameraDirection) {
                    return VisionPortal.easyCreateWithDefaults((BuiltinCameraDirection) camera, visionProcessor1, visionProcessor2);
                }
                if (camera instanceof WebcamName) {
                    return VisionPortal.easyCreateWithDefaults((WebcamName) camera, visionProcessor1, visionProcessor2);
                }
                if (camera instanceof SwitchableCameraName) {
                    return VisionPortal.easyCreateWithDefaults((SwitchableCameraName) camera, visionProcessor1, visionProcessor2);
                }
                reportInvalidArg("camera", "BuiltinCameraName, WebcamName, or SwitchableCameraName");
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"setProcessorEnabled"})
    public void setProcessorEnabled(Object visionPortalArg, Object visionProcessorArg, boolean enabled) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".setProcessorEnabled");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            VisionProcessor visionProcessor = checkVisionProcessor(visionProcessorArg);
            if (visionPortal != null && visionProcessor != null) {
                visionPortal.setProcessorEnabled(visionProcessor, enabled);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getProcessorEnabled"})
    public boolean getProcessorEnabled(Object visionPortalArg, Object visionProcessorArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getProcessorEnabled");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            VisionProcessor visionProcessor = checkVisionProcessor(visionProcessorArg);
            if (visionPortal != null && visionProcessor != null) {
                return visionPortal.getProcessorEnabled(visionProcessor);
            }
            endBlockExecution();
            return false;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getCameraState"})
    public String getCameraState(Object visionPortalArg) {
        VisionPortal.CameraState cameraState;
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getCameraState");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null && (cameraState = visionPortal.getCameraState()) != null) {
                return cameraState.toString();
            }
            return "";
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"saveNextFrameRaw"})
    public void saveNextFrameRaw(Object visionPortalArg, String filename) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".saveNextFrameRaw");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                visionPortal.saveNextFrameRaw(filename);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"stopStreaming"})
    public void stopStreaming(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".stopStreaming");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                visionPortal.stopStreaming();
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"resumeStreaming"})
    public void resumeStreaming(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".resumeStreaming");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                visionPortal.resumeStreaming();
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"stopLiveView"})
    public void stopLiveView(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".stopLiveView");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                visionPortal.stopLiveView();
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"resumeLiveView"})
    public void resumeLiveView(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".resumeLiveView");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                visionPortal.resumeLiveView();
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getFps"})
    public float getFps(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getFps");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return visionPortal.getFps();
            }
            endBlockExecution();
            return 0.0f;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getCameraControl"})
    public ExposureControl getExposureControl(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getExposureControl");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return (ExposureControl) visionPortal.getCameraControl(ExposureControl.class);
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getCameraControl"})
    public FocusControl getFocusControl(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getFocusControl");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return (FocusControl) visionPortal.getCameraControl(FocusControl.class);
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getCameraControl"})
    public GainControl getGainControl(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getGainControl");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return (GainControl) visionPortal.getCameraControl(GainControl.class);
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getCameraControl"})
    public PtzControl getPtzControl(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getPtzControl");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return (PtzControl) visionPortal.getCameraControl(PtzControl.class);
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getCameraControl"})
    public WhiteBalanceControl getWhiteBalanceControl(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getWhiteBalanceControl");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return (WhiteBalanceControl) visionPortal.getCameraControl(WhiteBalanceControl.class);
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"setActiveCamera"})
    public void setActiveCamera(Object visionPortalArg, Object webcamNameArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".setActiveCamera");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            WebcamName webcamName = checkWebcamName(webcamNameArg);
            if (visionPortal != null) {
                visionPortal.setActiveCamera(webcamName);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"getActiveCamera"})
    public WebcamName getActiveCamera(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".getActiveCamera");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                return visionPortal.getActiveCamera();
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public void cameraStreamServer_setSource(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "CameraStreamServer", ".setSource");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                CameraStreamServer.getInstance().setSource(visionPortal);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"close"})
    public void close(Object visionPortalArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".close");
            VisionPortal visionPortal = checkVisionPortal(visionPortalArg);
            if (visionPortal != null) {
                visionPortal.close();
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {VisionPortal.class}, methodName = {"makeMultiPortalView"})
    public String makeMultiPortalView(int numPortals, String multiPortalLayoutString) {
        try {
            startBlockExecution(BlockType.FUNCTION, "VisionPortal", ".makeMultiPortalView");
            VisionPortal.MultiPortalLayout multiPortalLayout = checkMultiPortalLayout(multiPortalLayoutString);
            if (multiPortalLayout != null) {
                return SimpleGson.getInstance().toJson(VisionPortal.makeMultiPortalView(numPortals, multiPortalLayout));
            }
            return "";
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }
}
