package org.firstinspires.ftc.teamcode.pedroPathing.theLab;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;

import fi.iki.elonen.NanoHTTPD;

/* JADX INFO: loaded from: classes7.dex */
@DeviceProperties(description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)", name = "goBILDA® Pinpoint Odometry Computer", xmlTag = "goBILDAPinpoint")
@I2cDeviceType
public class GoBildaPinpointDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    public static final byte DEFAULT_ADDRESS = 49;
    private static final float goBILDA_4_BAR_POD = 19.894367f;
    private static final float goBILDA_SWINGARM_POD = 13.262912f;
    private int deviceStatus;
    private float hOrientation;
    private float hVelocity;
    private int loopTime;
    private int xEncoderValue;
    private float xPosition;
    private float xVelocity;
    private int yEncoderValue;
    private float yPosition;
    private float yVelocity;

    public enum EncoderDirection {
        FORWARD,
        REVERSED
    }

    public enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD
    }

    public enum ReadData {
        ONLY_UPDATE_HEADING
    }

    public GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceStatus = 0;
        this.loopTime = 0;
        this.xEncoderValue = 0;
        this.yEncoderValue = 0;
        this.xPosition = 0.0f;
        this.yPosition = 0.0f;
        this.hOrientation = 0.0f;
        this.xVelocity = 0.0f;
        this.yVelocity = 0.0f;
        this.hVelocity = 0.0f;
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(49));
        super.registerArmingStateCallback(false);
    }

    @Override // com.qualcomm.robotcore.hardware.HardwareDevice
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override // com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
    protected synchronized boolean doInitialize() {
        ((LynxI2cDeviceSynch) this.deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override // com.qualcomm.robotcore.hardware.HardwareDevice
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }

    private enum Register {
        DEVICE_ID(1),
        DEVICE_VERSION(2),
        DEVICE_STATUS(3),
        DEVICE_CONTROL(4),
        LOOP_TIME(5),
        X_ENCODER_VALUE(6),
        Y_ENCODER_VALUE(7),
        X_POSITION(8),
        Y_POSITION(9),
        H_ORIENTATION(10),
        X_VELOCITY(11),
        Y_VELOCITY(12),
        H_VELOCITY(13),
        MM_PER_TICK(14),
        X_POD_OFFSET(15),
        Y_POD_OFFSET(16),
        YAW_SCALAR(17),
        BULK_READ(18);

        private final int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum DeviceStatus {
        NOT_READY(0),
        READY(1),
        CALIBRATING(2),
        FAULT_X_POD_NOT_DETECTED(4),
        FAULT_Y_POD_NOT_DETECTED(8),
        FAULT_NO_PODS_DETECTED(12),
        FAULT_IMU_RUNAWAY(16),
        FAULT_BAD_READ(32);

        private final int status;

        DeviceStatus(int status) {
            this.status = status;
        }
    }

    private void writeInt(Register reg, int i) {
        this.deviceClient.write(reg.bVal, TypeConversion.intToByteArray(i, ByteOrder.LITTLE_ENDIAN));
    }

    private int readInt(Register reg) {
        return TypeConversion.byteArrayToInt(this.deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
    }

    private float byteArrayToFloat(byte[] byteArray, ByteOrder byteOrder) {
        return ByteBuffer.wrap(byteArray).order(byteOrder).getFloat();
    }

    private float readFloat(Register reg) {
        return byteArrayToFloat(this.deviceClient.read(reg.bVal, 4), ByteOrder.LITTLE_ENDIAN);
    }

    private byte[] floatToByteArray(float value, ByteOrder byteOrder) {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array();
    }

    private void writeByteArray(Register reg, byte[] bytes) {
        this.deviceClient.write(reg.bVal, bytes);
    }

    private void writeFloat(Register reg, float f) {
        byte[] bytes = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN).putFloat(f).array();
        this.deviceClient.write(reg.bVal, bytes);
    }

    private DeviceStatus lookupStatus(int s) {
        if ((DeviceStatus.CALIBRATING.status & s) == 0) {
            boolean xPodDetected = (DeviceStatus.FAULT_X_POD_NOT_DETECTED.status & s) == 0;
            boolean yPodDetected = (DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status & s) == 0;
            if (!xPodDetected && !yPodDetected) {
                return DeviceStatus.FAULT_NO_PODS_DETECTED;
            }
            if (!xPodDetected) {
                return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
            }
            if (yPodDetected) {
                if ((DeviceStatus.FAULT_IMU_RUNAWAY.status & s) == 0) {
                    if ((DeviceStatus.READY.status & s) == 0) {
                        if ((DeviceStatus.FAULT_BAD_READ.status & s) != 0) {
                            return DeviceStatus.FAULT_BAD_READ;
                        }
                        return DeviceStatus.NOT_READY;
                    }
                    return DeviceStatus.READY;
                }
                return DeviceStatus.FAULT_IMU_RUNAWAY;
            }
            return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        return DeviceStatus.CALIBRATING;
    }

    private Float isPositionCorrupt(float oldValue, float newValue, int threshold, boolean bulkUpdate) {
        boolean noData = bulkUpdate && this.loopTime < 1;
        boolean isCorrupt = noData || Float.isNaN(newValue) || Math.abs(newValue - oldValue) > ((float) threshold);
        if (isCorrupt) {
            this.deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
            return Float.valueOf(oldValue);
        }
        return Float.valueOf(newValue);
    }

    private Float isVelocityCorrupt(float oldValue, float newValue, int threshold) {
        boolean isCorrupt = Float.isNaN(newValue) || Math.abs(newValue) > ((float) threshold);
        if (this.loopTime <= 1) {
        }
        if (isCorrupt) {
            this.deviceStatus = DeviceStatus.FAULT_BAD_READ.status;
            return Float.valueOf(oldValue);
        }
        return Float.valueOf(newValue);
    }

    public void update() {
        float oldPosX = this.xPosition;
        float oldPosY = this.yPosition;
        float oldPosH = this.hOrientation;
        float oldVelX = this.xVelocity;
        float oldVelY = this.yVelocity;
        float oldVelH = this.hVelocity;
        byte[] bArr = this.deviceClient.read(Register.BULK_READ.bVal, 40);
        this.deviceStatus = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 0, 4), ByteOrder.LITTLE_ENDIAN);
        this.loopTime = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 4, 8), ByteOrder.LITTLE_ENDIAN);
        this.xEncoderValue = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 8, 12), ByteOrder.LITTLE_ENDIAN);
        this.yEncoderValue = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bArr, 12, 16), ByteOrder.LITTLE_ENDIAN);
        this.xPosition = byteArrayToFloat(Arrays.copyOfRange(bArr, 16, 20), ByteOrder.LITTLE_ENDIAN);
        this.yPosition = byteArrayToFloat(Arrays.copyOfRange(bArr, 20, 24), ByteOrder.LITTLE_ENDIAN);
        this.hOrientation = byteArrayToFloat(Arrays.copyOfRange(bArr, 24, 28), ByteOrder.LITTLE_ENDIAN);
        this.xVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 28, 32), ByteOrder.LITTLE_ENDIAN);
        this.yVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 32, 36), ByteOrder.LITTLE_ENDIAN);
        this.hVelocity = byteArrayToFloat(Arrays.copyOfRange(bArr, 36, 40), ByteOrder.LITTLE_ENDIAN);
        this.xPosition = isPositionCorrupt(oldPosX, this.xPosition, NanoHTTPD.SOCKET_READ_TIMEOUT, true).floatValue();
        this.yPosition = isPositionCorrupt(oldPosY, this.yPosition, NanoHTTPD.SOCKET_READ_TIMEOUT, true).floatValue();
        this.hOrientation = isPositionCorrupt(oldPosH, this.hOrientation, 120, true).floatValue();
        this.xVelocity = isVelocityCorrupt(oldVelX, this.xVelocity, 10000).floatValue();
        this.yVelocity = isVelocityCorrupt(oldVelY, this.yVelocity, 10000).floatValue();
        this.hVelocity = isVelocityCorrupt(oldVelH, this.hVelocity, 120).floatValue();
    }

    public void update(ReadData data) {
        if (data == ReadData.ONLY_UPDATE_HEADING) {
            float oldPosH = this.hOrientation;
            this.hOrientation = byteArrayToFloat(this.deviceClient.read(Register.H_ORIENTATION.bVal, 4), ByteOrder.LITTLE_ENDIAN);
            this.hOrientation = isPositionCorrupt(oldPosH, this.hOrientation, 120, false).floatValue();
            if (this.deviceStatus == DeviceStatus.FAULT_BAD_READ.status) {
                this.deviceStatus = DeviceStatus.READY.status;
            }
        }
    }

    public void setOffsets(double xOffset, double yOffset) {
        writeFloat(Register.X_POD_OFFSET, (float) xOffset);
        writeFloat(Register.Y_POD_OFFSET, (float) yOffset);
    }

    public void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit) {
        writeFloat(Register.X_POD_OFFSET, (float) distanceUnit.toMm(xOffset));
        writeFloat(Register.Y_POD_OFFSET, (float) distanceUnit.toMm(yOffset));
    }

    public void recalibrateIMU() {
        writeInt(Register.DEVICE_CONTROL, 1);
    }

    public void resetPosAndIMU() {
        writeInt(Register.DEVICE_CONTROL, 2);
    }

    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder) {
        if (xEncoder == EncoderDirection.FORWARD) {
            writeInt(Register.DEVICE_CONTROL, 32);
        }
        if (xEncoder == EncoderDirection.REVERSED) {
            writeInt(Register.DEVICE_CONTROL, 16);
        }
        if (yEncoder == EncoderDirection.FORWARD) {
            writeInt(Register.DEVICE_CONTROL, 8);
        }
        if (yEncoder == EncoderDirection.REVERSED) {
            writeInt(Register.DEVICE_CONTROL, 4);
        }
    }

    public void setEncoderResolution(GoBildaOdometryPods pods) {
        if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            writeByteArray(Register.MM_PER_TICK, floatToByteArray(goBILDA_SWINGARM_POD, ByteOrder.LITTLE_ENDIAN));
        }
        if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD) {
            writeByteArray(Register.MM_PER_TICK, floatToByteArray(goBILDA_4_BAR_POD, ByteOrder.LITTLE_ENDIAN));
        }
    }

    public void setEncoderResolution(double ticks_per_mm) {
        writeByteArray(Register.MM_PER_TICK, floatToByteArray((float) ticks_per_mm, ByteOrder.LITTLE_ENDIAN));
    }

    public void setEncoderResolution(double ticks_per_unit, DistanceUnit distanceUnit) {
        double resolution = distanceUnit.toMm(ticks_per_unit);
        writeByteArray(Register.MM_PER_TICK, floatToByteArray((float) resolution, ByteOrder.LITTLE_ENDIAN));
    }

    public void setYawScalar(double yawOffset) {
        writeByteArray(Register.YAW_SCALAR, floatToByteArray((float) yawOffset, ByteOrder.LITTLE_ENDIAN));
    }

    public Pose2D setPosition(Pose2D pos) {
        writeByteArray(Register.X_POSITION, floatToByteArray((float) pos.getX(DistanceUnit.MM), ByteOrder.LITTLE_ENDIAN));
        writeByteArray(Register.Y_POSITION, floatToByteArray((float) pos.getY(DistanceUnit.MM), ByteOrder.LITTLE_ENDIAN));
        writeByteArray(Register.H_ORIENTATION, floatToByteArray((float) pos.getHeading(AngleUnit.RADIANS), ByteOrder.LITTLE_ENDIAN));
        return pos;
    }

    public void setPosX(double posX, DistanceUnit distanceUnit) {
        writeByteArray(Register.X_POSITION, floatToByteArray((float) distanceUnit.toMm(posX), ByteOrder.LITTLE_ENDIAN));
    }

    public void setPosY(double posY, DistanceUnit distanceUnit) {
        writeByteArray(Register.Y_POSITION, floatToByteArray((float) distanceUnit.toMm(posY), ByteOrder.LITTLE_ENDIAN));
    }

    public void setHeading(double heading, AngleUnit angleUnit) {
        writeByteArray(Register.H_ORIENTATION, floatToByteArray((float) angleUnit.toRadians(heading), ByteOrder.LITTLE_ENDIAN));
    }

    public int getDeviceID() {
        return readInt(Register.DEVICE_ID);
    }

    public int getDeviceVersion() {
        return readInt(Register.DEVICE_VERSION);
    }

    public float getYawScalar() {
        return readFloat(Register.YAW_SCALAR);
    }

    public DeviceStatus getDeviceStatus() {
        return lookupStatus(this.deviceStatus);
    }

    public int getLoopTime() {
        return this.loopTime;
    }

    public double getFrequency() {
        if (this.loopTime != 0) {
            return 1000000.0d / ((double) this.loopTime);
        }
        return LynxServoController.apiPositionFirst;
    }

    public int getEncoderX() {
        return this.xEncoderValue;
    }

    public int getEncoderY() {
        return this.yEncoderValue;
    }

    public double getPosX() {
        return this.xPosition;
    }

    public double getPosX(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(this.xPosition);
    }

    public double getPosY() {
        return this.yPosition;
    }

    public double getPosY(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(this.yPosition);
    }

    public double getHeading() {
        return this.hOrientation;
    }

    public double getHeading(AngleUnit angleUnit) {
        return (angleUnit.fromRadians(((((double) this.hOrientation) + 3.141592653589793d) % 6.283185307179586d) + 6.283185307179586d) % 6.283185307179586d) - 3.141592653589793d;
    }

    public double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return unnormalizedAngleUnit.fromRadians(this.hOrientation);
    }

    public double getVelX() {
        return this.xVelocity;
    }

    public double getVelX(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(this.xVelocity);
    }

    public double getVelY() {
        return this.yVelocity;
    }

    public double getVelY(DistanceUnit distanceUnit) {
        return distanceUnit.fromMm(this.yVelocity);
    }

    public double getHeadingVelocity() {
        return this.hVelocity;
    }

    public double getHeadingVelocity(UnnormalizedAngleUnit unnormalizedAngleUnit) {
        return unnormalizedAngleUnit.fromRadians(this.hVelocity);
    }

    public float getXOffset(DistanceUnit distanceUnit) {
        return (float) distanceUnit.fromMm(readFloat(Register.X_POD_OFFSET));
    }

    public float getYOffset(DistanceUnit distanceUnit) {
        return (float) distanceUnit.fromMm(readFloat(Register.Y_POD_OFFSET));
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM, this.xPosition, this.yPosition, AngleUnit.RADIANS, ((((((double) this.hOrientation) + 3.141592653589793d) % 6.283185307179586d) + 6.283185307179586d) % 6.283185307179586d) - 3.141592653589793d);
    }

    public Pose2D getVelocity() {
        return new Pose2D(DistanceUnit.MM, this.xVelocity, this.yVelocity, AngleUnit.RADIANS, ((((((double) this.hVelocity) + 3.141592653589793d) % 6.283185307179586d) + 6.283185307179586d) % 6.283185307179586d) - 3.141592653589793d);
    }
}
