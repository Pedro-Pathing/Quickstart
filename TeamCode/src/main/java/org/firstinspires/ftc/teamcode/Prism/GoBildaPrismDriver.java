/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Prism;

import static com.qualcomm.robotcore.util.TypeConversion.byteArrayToInt;
import static com.qualcomm.robotcore.util.TypeConversion.unsignedByteToInt;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@I2cDeviceType
@DeviceProperties(
        name = "goBILDA® Prism RGB LED Driver",
        xmlTag = "goBILDAPrism",
        description ="Prism RGB LED Driver (6-30V Input, I²C / PWM Control)"
)
public class GoBildaPrismDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    private static final byte DEFAULT_ADDRESS = 0x38;
    private final int MAXIMUM_NUMBER_OF_ANIMATIONS = 10;
    private final int MAXIMUM_NUMBER_OF_ANIMATION_GROUPS = 8;
    private PrismAnimations.AnimationBase[] animations = new PrismAnimations.AnimationBase[MAXIMUM_NUMBER_OF_ANIMATIONS];

    //#region Public Types
    public enum LayerHeight
    {
        LAYER_0 (Register.ANIMATION_SLOT_0),
        LAYER_1 (Register.ANIMATION_SLOT_1),
        LAYER_2 (Register.ANIMATION_SLOT_2),
        LAYER_3 (Register.ANIMATION_SLOT_3),
        LAYER_4 (Register.ANIMATION_SLOT_4),
        LAYER_5 (Register.ANIMATION_SLOT_5),
        LAYER_6 (Register.ANIMATION_SLOT_6),
        LAYER_7 (Register.ANIMATION_SLOT_7),
        LAYER_8 (Register.ANIMATION_SLOT_8),
        LAYER_9 (Register.ANIMATION_SLOT_9),
        DISABLED(Register.NULL);

        /* Package Private */ final Register register;
        /* Package Private */ final int index;

        LayerHeight(Register register){
            this.register = register;
            if(register == Register.NULL)
                this.index = -1;
            else
                this.index = register.address - Register.ANIMATION_SLOT_0.address;
        }
    }

    public enum Artboard
    {
        ARTBOARD_0 (0,0),
        ARTBOARD_1 (1,1),
        ARTBOARD_2 (2,2),
        ARTBOARD_3 (3,3),
        ARTBOARD_4 (4,4),
        ARTBOARD_5 (5,5),
        ARTBOARD_6 (6,6),
        ARTBOARD_7 (7,7);

        /* Package Private */ final byte bitmask;
        public final int index;

        Artboard(int val,int index){
            this.bitmask = (byte)(1 << val);
            this.index = index;
        }
    }
    //#endregion

    //#region Package-Private types
    /**
     * Captures the length of each type of register used on the device.
     */
    /* Package Private */ enum RegisterType
    {
        INT8(1,255),
        INT16(2, 65535),
        INT24(3, 16777215),
        INT32(4, 2147483647);

        /* Package Private */ final int lengthBytes;
        /* Package Private */ final int maxValue;

        RegisterType(int lengthBytes, int maxValue){
            this.lengthBytes = lengthBytes;
            this.maxValue = maxValue;
        }
    }

    /* Package Private */ enum RegisterAccess
    {
        READ_ONLY,
        WRITE_ONLY,
        READ_AND_WRITE;
    }

    /**
     * Register map, including register address and register type
     */
    /* Package Private */ enum Register
    {
        DEVICE_ID         (0 , RegisterType.INT8 , RegisterAccess.READ_ONLY),
        FIRMWARE_VERSION  (1 , RegisterType.INT24, RegisterAccess.READ_ONLY),
        HARDWARE_VERSION  (2 , RegisterType.INT16, RegisterAccess.READ_ONLY),
        POWER_CYCLE_COUNT (3 , RegisterType.INT32, RegisterAccess.READ_ONLY),
        RUNTIME_IN_MINUTES(4 , RegisterType.INT32, RegisterAccess.READ_ONLY),
        STATUS            (5 , RegisterType.INT32, RegisterAccess.READ_ONLY),
        CONTROL           (6 , RegisterType.INT32, RegisterAccess.WRITE_ONLY),
        ARTBOARD_CONTROL  (7 , RegisterType.INT32, RegisterAccess.WRITE_ONLY),
        ANIMATION_SLOT_0  (8 , RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_1  (9 , RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_2  (10, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_3  (11, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_4  (12, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_5  (13, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_6  (14, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_7  (15, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_8  (16, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        ANIMATION_SLOT_9  (17, RegisterType.INT32, RegisterAccess.READ_AND_WRITE),
        NULL              (18, RegisterType.INT8 , RegisterAccess.READ_ONLY);

        /* Package Private */ final int address;
        /* Package Private */ final RegisterType registerType;
        /* Package Private */ final RegisterAccess registerAccess;

        Register(int address, RegisterType registerType, RegisterAccess registerAccess){
            this.address = address;
            this.registerType = registerType;
            this.registerAccess = registerAccess;
        }
    }

    private int readInt(Register register){
        return byteArrayToInt(deviceClient.read(register.address,register.registerType.lengthBytes),ByteOrder.LITTLE_ENDIAN);
    }

    //#endregion

    public GoBildaPrismDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.GoBilda;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        ((LynxI2cDeviceSynch)(deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "goBILDA® Prism RGB LED Driver";
    }

    /**
     * @return 3 if device is functional.
     */
    public int getDeviceID(){
        //return readInt(Register.DEVICE_ID);
        byte[] packet = deviceClient.read(Register.DEVICE_ID.address, Register.DEVICE_ID.registerType.lengthBytes);
        return packet[0];
    }

    /**
     * @return Firmware Version of device. Array[0] is the major version, Array[1] is the minor version
     * Array[2] is the patch version.
     */
    public int[] getFirmwareVersion(){
        byte[] packet = deviceClient.read(Register.FIRMWARE_VERSION.address,Register.FIRMWARE_VERSION.registerType.lengthBytes);
        int[] output = new int[3];
        output[0] = packet[2];
        output[1] = packet[1];
        output[2] = packet[0];
        return output;
    }

    /**
     * @return Hardware version of the device as a string.
     */
    public String getFirmwareVersionString(){
        byte[] packet = deviceClient.read(Register.FIRMWARE_VERSION.address,Register.FIRMWARE_VERSION.registerType.lengthBytes);
        int[] output = new int[3];
        output[0] = packet[2];
        output[1] = packet[1];
        output[2] = packet[0];
        return String.format("%d.%d.%d", output[0], output[1], output[2]);
    }

    /**
     * @return Hardware version of device. Array[0] is the major version, Array[1] is the minor version.
     */
    public int[] getHardwareVersion(){
        byte[] packet = deviceClient.read(Register.HARDWARE_VERSION.address,Register.HARDWARE_VERSION.registerType.lengthBytes);
        int[] output = new int[2];
        output[0] = packet[1];
        output[1] = packet[0];
        return output;
    }
    public String getHardwareVersionString(){
        byte[] packet = deviceClient.read(Register.HARDWARE_VERSION.address,Register.HARDWARE_VERSION.registerType.lengthBytes);
        int[] output = new int[2];
        output[0] = packet[1];
        output[1] = packet[0];
        return String.format("%d.%d", output[0], output[1]);
    }

    /**
     * @return The number of times the device has power cycled in its lifetime.
     */
    public int getPowerCycleCount(){
        return readInt(Register.POWER_CYCLE_COUNT);
    }

    /**
     * @return Total device runtime in minutes
     */
    public long getRunTime(TimeUnit timeUnit){
        return timeUnit.convert(readInt(Register.RUNTIME_IN_MINUTES),TimeUnit.MINUTES);
    }

    /**
     * @return total LEDs in strip
     */
    public int getNumberOfLEDs(){
       byte[] packet = deviceClient.read(Register.STATUS.address,Register.STATUS.registerType.lengthBytes);
       return unsignedByteToInt(packet[0]);
    }

    /**
     * @return Current Animation frames per second
     */
    public int getCurrentFPS(){
        byte[] inputPacket = deviceClient.read(Register.STATUS.address,Register.STATUS.registerType.lengthBytes);
        byte[] outputPacket = new byte[4];
        outputPacket[0] = inputPacket[1];
        outputPacket[1] = inputPacket[2];

        return byteArrayToInt(outputPacket,ByteOrder.LITTLE_ENDIAN);
    }

    /**
     * @return
     */
    public boolean fpsLimited(){
        byte[] packet = deviceClient.read(Register.STATUS.address, Register.STATUS.registerType.lengthBytes);
        //return packet[3] >> 7;
        return false;
    }

    /**
     * return What artboard is set as default boot animation?
     */
    public int getBootAnimationArtboard(){
        byte[] packet = deviceClient.read(Register.STATUS.address, Register.STATUS.registerType.lengthBytes);
        return 0;
    }


    /**
     * Inserts an animation into the specified slot in the animations array.
     * 
     * @param height the height where the animation should be inserted (0-9)
     * @param animation the AnimationBase object to insert into the array
     * @return true if the animation was successfully inserted, false if the index 
     *         is out of bounds or the animation is null
     */
    public boolean insertAnimation(LayerHeight height, PrismAnimations.AnimationBase animation)
    {
        if(height == LayerHeight.DISABLED || animation == null)
            return false;

        animations[height.index] = animation;
        animations[height.index].layerHeight = height;
        return true;
    }

    /**
     * Inserts an animation at the specified index and immediately updates it over I2C.
     * 
     * @param height the height where the animation should be inserted and updated (0-9)
     * @param animation the AnimationBase object to insert and update
     * @return true if both insertion and update operations were successful, 
     *         false if either operation failed
     */
    public boolean insertAndUpdateAnimation(LayerHeight height, PrismAnimations.AnimationBase animation)
    {
        if(insertAnimation(height, animation))
            return updateAnimationFromIndex(height, true);
        return false;
    }

    /**
     * Updates all animations in the manager by sending their data over I2C.
     * Iterates through all animation slots and updates any non-null animations.
     * 
     * @return true if the update process completed (currently always returns true)
     */
    public boolean updateAllAnimations()
    {
        for(int i = 0; i < MAXIMUM_NUMBER_OF_ANIMATIONS; i++){
            if(animations[i] != null && animations[i].layerHeight != LayerHeight.DISABLED)
                animations[i].updateAnimationOverI2C(this.deviceClient, false);
        }
        return true;    
    }
    
    /**
     * Updates a specific animation at the given Layer Height by sending its data over I2C.
     * 
     * @param height the height of the animation to update (0-9)
     * @return true if the animation was successfully updated, false if the index 
     *         is out of bounds or if no animation exists at the specified index
     */
    public boolean updateAnimationFromIndex(LayerHeight height)
    {
        return updateAnimationFromIndex(height, false);
    }

    public void clearAllAnimations()
    {
        byte[] packet = TypeConversion.intToByteArray(1 << 25, ByteOrder.LITTLE_ENDIAN);

        deviceClient.write(Register.CONTROL.address, packet);
        Arrays.fill(animations, null);
    }

    public void setTargetFPS(int targetFPS)
    {
        final int boundedTargetFps = Math.max(0, Math.min(targetFPS, 0x7FFF));
        final int ChangeTargetFpsBit = 1 << 15;
        final int ChangeTargetFpsCommand = ChangeTargetFpsBit | boundedTargetFps;
        byte[] packet = TypeConversion.intToByteArray(ChangeTargetFpsCommand, ByteOrder.LITTLE_ENDIAN);
        deviceClient.write(Register.CONTROL.address, packet);
    }

    public void setStripLength(int stripLength)
    {
        final int boundedStripLength = Math.max(0, Math.min(stripLength, 0xFF));
        final int ChangeStripLengthBit = 1 << 24;
        final int ChangeStripLengthCommand = ChangeStripLengthBit | (boundedStripLength << 16);
        byte[] packet = TypeConversion.intToByteArray(ChangeStripLengthCommand, ByteOrder.LITTLE_ENDIAN);
        deviceClient.write(Register.CONTROL.address, packet);
    }

    public void saveCurrentAnimationsToArtboard(Artboard artboard)
    {
        byte[] data = {
            artboard.bitmask,
            0,
            0,
            0
        };
        deviceClient.write(Register.ARTBOARD_CONTROL.address, data);
    }

    public void loadAnimationsFromArtboard(Artboard artboard)
    {
        byte[] data = {
            0,
            artboard.bitmask,
            0,
            0
        };
        deviceClient.write(Register.ARTBOARD_CONTROL.address, data);
    }

    public void setDefaultBootArtboard(Artboard artboard)
    {
        byte[] data = {
            0,
            0,
            artboard.bitmask,
            0
        };
        deviceClient.write(Register.ARTBOARD_CONTROL.address, data);
    }

    public void enableDefaultBootArtboard(boolean enable)
    {
        byte[] data = {
                0,
                0,
                0,
                enable ? (byte)0b00000001 : (byte)0b00000010
        };
        deviceClient.write(Register.ARTBOARD_CONTROL.address, data);
    }

    private boolean updateAnimationFromIndex(LayerHeight height, boolean isBeingInserted)
    {
        if(height == LayerHeight.DISABLED || animations[height.index] == null)
            return false;

        boolean animationEnabled = animations[height.index].layerHeight != LayerHeight.DISABLED;
        if(animationEnabled)
            animations[height.index].updateAnimationOverI2C(this.deviceClient, isBeingInserted);
        return animationEnabled;
    }
}
