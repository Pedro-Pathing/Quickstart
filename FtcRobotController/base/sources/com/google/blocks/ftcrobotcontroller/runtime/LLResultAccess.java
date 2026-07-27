package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.sun.tools.doclint.DocLint;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/* JADX INFO: loaded from: classes8.dex */
class LLResultAccess extends Access {
    LLResultAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "LLResult");
    }

    private LLResult checkLLResult(Object llResultArg) {
        return (LLResult) checkArg(llResultArg, LLResult.class, "llResult");
    }

    @JavascriptInterface
    @Block(classes = {LLResult.class}, methodName = {"getStaleness"})
    public long getStaleness(Object llResultArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".getStaleness");
            LLResult llResult = checkLLResult(llResultArg);
            if (llResultArg != null) {
                return llResult.getStaleness();
            }
            endBlockExecution();
            return 0L;
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
    @Block(classes = {LLResult.class}, methodName = {"toString"})
    public String toText(Object llResultArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            LLResult llResult = checkLLResult(llResultArg);
            if (llResultArg != null) {
                return llResult.toString();
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
    @Block(exclusiveToBlocks = true)
    public String pose3DToText(String json) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            Pose3D pose3D = (Pose3D) fromJson(json, Pose3D.class);
            return pose3D.toString();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    static String llResultToJson(LLResult llResult) {
        StringBuilder json = new StringBuilder();
        json.append("{").append("\"PythonOutput\":").append(toJson(llResult.getPythonOutput())).append(DocLint.TAGS_SEPARATOR).append("\"FiducialResults\":").append(toJson(llResult.getFiducialResults())).append(DocLint.TAGS_SEPARATOR).append("\"ColorResults\":").append(toJson(llResult.getColorResults())).append(DocLint.TAGS_SEPARATOR).append("\"ControlHubTimeStamp\":").append(llResult.getControlHubTimeStamp()).append(DocLint.TAGS_SEPARATOR).append("\"ControlHubTimeStampNanos\":").append(llResult.getControlHubTimeStampNanos()).append(DocLint.TAGS_SEPARATOR).append("\"FocusMetric\":").append(llResult.getFocusMetric()).append(DocLint.TAGS_SEPARATOR).append("\"Botpose\":").append(toJson(llResult.getBotpose())).append(DocLint.TAGS_SEPARATOR).append("\"Botpose_MT2\":").append(toJson(llResult.getBotpose_MT2())).append(DocLint.TAGS_SEPARATOR).append("\"BotposeTagCount\":").append(llResult.getBotposeTagCount()).append(DocLint.TAGS_SEPARATOR).append("\"BotposeSpan\":").append(llResult.getBotposeSpan()).append(DocLint.TAGS_SEPARATOR).append("\"BotposeAvgDist\":").append(llResult.getBotposeAvgDist()).append(DocLint.TAGS_SEPARATOR).append("\"BotposeAvgArea\":").append(llResult.getBotposeAvgArea()).append(DocLint.TAGS_SEPARATOR).append("\"CaptureLatency\":").append(llResult.getCaptureLatency()).append(DocLint.TAGS_SEPARATOR).append("\"Tx\":").append(llResult.getTx()).append(DocLint.TAGS_SEPARATOR).append("\"Ty\":").append(llResult.getTy()).append(DocLint.TAGS_SEPARATOR).append("\"TxNC\":").append(llResult.getTxNC()).append(DocLint.TAGS_SEPARATOR).append("\"TyNC\":").append(llResult.getTyNC()).append(DocLint.TAGS_SEPARATOR).append("\"Ta\":").append(llResult.getTa()).append(DocLint.TAGS_SEPARATOR).append("\"PipelineIndex\":").append(llResult.getPipelineIndex()).append(DocLint.TAGS_SEPARATOR).append("\"TargetingLatency\":").append(llResult.getTargetingLatency()).append(DocLint.TAGS_SEPARATOR).append("\"Timestamp\":").append(llResult.getTimestamp()).append(DocLint.TAGS_SEPARATOR).append("\"PipelineType\":\"").append(llResult.getPipelineType()).append("\"").append(DocLint.TAGS_SEPARATOR).append("\"IsValid\":\"").append(llResult.isValid()).append("\"").append("}");
        return json.toString();
    }
}
