
/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;

/**
 * Flywheel velocity test. Holds motorName at targetRPM using the shared Flywheel controller
 * (PID feedback plus feedforward, rate-limited power and a filtered RPM reading).
 * It uses the Flywheel default gains; use FlywheelTuner to tune them.
 */
@TeleOp(name = "Fly Me To The Wheel", group = "Tuning")
@Configurable
public class FlyMeToTheWheel extends OpMode {

    public static String motorName = "Launcher";
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    public static double targetRPM = 5000;

    private final SystemStatus status = new SystemStatus();
    private BulkCache bulkCache;
    private Flywheel flywheel;
    private TelemetryManager telemetryM;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        bulkCache = new BulkCache(hardwareMap);
        flywheel = new Flywheel(hardwareMap, motorName, direction);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        status.check("Flywheel", flywheel.missingDevices());

        telemetry.addData("Status", "Initialized");
        status.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        bulkCache.clear();
        runtime.reset();
        flywheel.reset();
    }

    @Override
    public void loop() {
        bulkCache.clear();
        flywheel.runToRPM(targetRPM);

        // Telemetry is only built when Panels is due to send it, to keep the loop fast
        if (telemetryM.getShouldUpdateLines()) {
            status.addTelemetry(telemetryM);
            telemetryM.addData("Run Time", runtime.toString());
            telemetryM.addData("targetRPM", targetRPM);
            telemetryM.addData("rawRPM", flywheel.rawRPM());
            telemetryM.addData("filteredRPM", flywheel.filteredRPM());
            telemetryM.addData("error", targetRPM - flywheel.filteredRPM());
            telemetryM.addData("power", flywheel.power());
            telemetryM.update(telemetry);
        }
    }

    @Override
    public void stop() {
        flywheel.stop();
    }
}
