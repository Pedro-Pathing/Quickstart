/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Intake;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Turret;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Drivetrain_.MecanumDrivetrainClass;

/*
 * This file is designed to work with our OpModes to handle all our hardware functionality to de-clutter our main scripts
 *
 * The logic goes in the OpModes and the hardware control is handled here.
 */
@Configurable
public class LoadHardwareClass {
    /* Declare OpMode members. */
    private final OpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Declare subclasses
    public final MecanumDrivetrainClass drivetrain;
    public final Turret turret;
    public final Intake intake;

    // Declare various enums & other variables that are useful across files
    public static Alliance selectedAlliance = null;
    public enum Alliance {
        RED,
        BLUE
    }

    /**
     * Copied from LinearOpMode.
     * @param milliseconds The number of milliseconds to sleep.
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Constructor that allows the OpMode to pass a reference to itself.
     * @param opmode The input for this parameter should almost always be "this".
     */
    public LoadHardwareClass(OpMode opmode) {
        myOpMode = opmode;
        this.drivetrain = new MecanumDrivetrainClass();
        this.turret     = new Turret();
        this.intake     = new Intake();
    }

    /**
     * Initializes all hardware for the robot.
     * Must be called once at the start of each op-mode.
     */
    public void init(Pose initialPose)    {
        // Initialize all subclasses
        drivetrain.init(myOpMode, initialPose);
        turret.init(myOpMode);
        intake.init(myOpMode);

        // Misc telemetry
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Initializes all hardware for the robot.
     * Must be called once at the start of each op-mode.
     */
    public void init(Pose initialPose, Follower follower)    {
        // Initialize all subclasses
        drivetrain.init(myOpMode, initialPose, follower);
        turret.init(myOpMode);
        intake.init(myOpMode);

        // Misc telemetry
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}
