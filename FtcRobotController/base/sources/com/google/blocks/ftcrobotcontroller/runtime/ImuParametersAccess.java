package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

/* JADX INFO: loaded from: classes8.dex */
class ImuParametersAccess extends Access {
    ImuParametersAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "IMU.Parameters");
    }

    private ImuOrientationOnRobot checkImuOrientationOnRobot(Object imuOrientationOnRobotArg) {
        return (ImuOrientationOnRobot) checkArg(imuOrientationOnRobotArg, ImuOrientationOnRobot.class, "imuOrientationOnRobot");
    }

    @JavascriptInterface
    @Block(classes = {IMU.Parameters.class}, constructor = true)
    public IMU.Parameters create(Object imuOrientationOnRobotArg) {
        try {
            startBlockExecution(BlockType.CREATE, "");
            ImuOrientationOnRobot imuOrientationOnRobot = checkImuOrientationOnRobot(imuOrientationOnRobotArg);
            if (imuOrientationOnRobotArg != null) {
                return new IMU.Parameters(imuOrientationOnRobot);
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
}
