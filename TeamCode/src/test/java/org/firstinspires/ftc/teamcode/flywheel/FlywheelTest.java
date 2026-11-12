package org.firstinspires.ftc.teamcode.flywheel;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;
import java.util.concurrent.atomic.AtomicLong;

/** Flywheel tests on a mocked motor, with a fake clock so no test has to wait. */
public class FlywheelTest {
    private static final double COUNTS_PER_REV = 28.0;
    private static final long STEP_NANOS = 10_000_000L;  // each simulated loop lasts 10 ms
    private static final double TARGET = 3000;

    private final AtomicLong nanos = new AtomicLong();
    private DcMotorEx motor;
    private Flywheel flywheel;

    @Before
    public void setUp() throws Exception {
        motor = mock(DcMotorEx.class);
        flywheel = new Flywheel(hardwareMapWith(motor), "PollenFlywheel",
                DcMotorSimple.Direction.REVERSE, nanos::get);
        flywheel.setRpmFilterTimeConstant(0);  // the filtered RPM follows the raw RPM exactly
        flywheel.reset();
    }

    /** A hardware map that returns the motor (or throws if it's null, like a missing device) and a battery. */
    private static HardwareMap hardwareMapWith(DcMotorEx motor) throws Exception {
        HardwareMap map = mock(HardwareMap.class);
        if (motor != null) {
            when(map.get(DcMotorEx.class, "PollenFlywheel")).thenReturn(motor);
        } else {
            when(map.get(DcMotorEx.class, "PollenFlywheel")).thenThrow(new IllegalArgumentException("not found"));
        }
        VoltageSensor battery = mock(VoltageSensor.class);
        when(battery.getVoltage()).thenReturn(12.0);
        @SuppressWarnings("unchecked")
        HardwareMap.DeviceMapping<VoltageSensor> sensors = mock(HardwareMap.DeviceMapping.class);
        when(sensors.iterator()).thenAnswer(i -> Collections.singletonList(battery).iterator());
        java.lang.reflect.Field field = HardwareMap.class.getField("voltageSensor");
        field.setAccessible(true);
        field.set(map, sensors);
        return map;
    }

    private void setRPM(double rpm) {
        when(motor.getVelocity()).thenReturn(rpm * COUNTS_PER_REV / 60.0);
    }

    /** Runs one 10 ms loop at the given measured speed. */
    private void loop(double measuredRPM) {
        nanos.addAndGet(STEP_NANOS);
        setRPM(measuredRPM);
        flywheel.runToRPM(TARGET);
    }

    private void loops(int count, double measuredRPM) {
        for (int i = 0; i < count; i++) loop(measuredRPM);
    }

    @Test
    public void readyOnlyAfterTheHoldTimeInTheWindow() {
        assertEquals(80, flywheel.getReadyHoldMs(), 0);

        loop(TARGET);  // enters the window: the hold starts now
        assertFalse(flywheel.isReady());
        loops(7, TARGET);  // 70 ms in
        assertFalse(flywheel.isReady());
        loop(TARGET);  // 80 ms in
        assertTrue(flywheel.isReady());
    }

    @Test
    public void leavingTheWindowRestartsTheHold() {
        loops(12, TARGET);
        assertTrue(flywheel.isReady());

        loop(TARGET * 1.05);  // out of the 2% window
        assertFalse(flywheel.isReady());

        loops(8, TARGET);  // back in, but only 70 ms since re-entering
        assertFalse(flywheel.isReady());
        loop(TARGET);
        assertTrue(flywheel.isReady());
    }

    @Test
    public void overshootingThroughTheWindowIsNotReady() {
        // Climb from 2500 to 3500 RPM in 10 ms steps: about 60 ms of it is inside 2940..3060 RPM
        for (double rpm = 2500; rpm <= 3500; rpm += 20) {
            loop(rpm);
            assertFalse("ready at " + rpm + " RPM", flywheel.isReady());
        }
    }

    @Test
    public void aTargetDriftingWithinTheToleranceKeepsTheHold() {
        loops(12, TARGET);
        assertTrue(flywheel.isReady());

        // The distance-based target changes a little every loop; that mustn't restart the hold
        for (int i = 0; i < 20; i++) {
            nanos.addAndGet(STEP_NANOS);
            setRPM(TARGET);
            flywheel.runToRPM(TARGET * (1 + 0.01 * Math.sin(i)));
            assertTrue("not ready on loop " + i, flywheel.isReady());
        }
    }

    @Test
    public void aTargetMovingBeyondTheToleranceRestartsTheHold() {
        loops(12, 3045);  // inside the 2% window round 3000
        assertTrue(flywheel.isReady());

        // 3090 is 3% above where the hold started; 3045 is inside its window, but it's a new speed to settle on
        nanos.addAndGet(STEP_NANOS);
        setRPM(3045);
        flywheel.runToRPM(3090);
        assertFalse(flywheel.isReady());
        for (int i = 0; i < 8; i++) {
            nanos.addAndGet(STEP_NANOS);
            flywheel.runToRPM(3090);
        }
        assertTrue(flywheel.isReady());
    }

    @Test
    public void holdTimeIsConfigurable() {
        flywheel.setReadyHoldMs(0);
        loop(TARGET);
        assertTrue(flywheel.isReady());

        flywheel.setReadyHoldMs(200);
        loop(2000);
        loops(19, TARGET);
        assertFalse(flywheel.isReady());
        loops(2, TARGET);  // 200 ms or more since entering
        assertTrue(flywheel.isReady());
    }

    @Test
    public void notReadyWithoutATarget() {
        assertFalse(flywheel.isReady());
        loops(12, TARGET);
        flywheel.stop();
        assertFalse(flywheel.isReady());
    }

    /** Runs to TARGET with the encoder reading nothing for long enough to trigger the fault check. */
    private void stallEncoder() {
        loops(400, 0);  // 4 s: the slew limit takes ~0.15 s to pass faultMinPower, then faultSeconds
    }

    @Test
    public void silentEncoderWithLowCurrentIsAnEncoderFault() {
        when(motor.getCurrent(CurrentUnit.AMPS)).thenReturn(0.4);
        stallEncoder();
        assertEquals(Flywheel.Health.ENCODER_FAULT, flywheel.getHealth());
        assertTrue(flywheel.power() > 0);  // still driving open loop
        assertTrue(flywheel.faultMessage().contains("open loop"));
    }

    @Test
    public void encoderFaultIsReadyOnceThePowerSettles() {
        when(motor.getCurrent(CurrentUnit.AMPS)).thenReturn(0.4);
        stallEncoder();
        assertEquals(Flywheel.Health.ENCODER_FAULT, flywheel.getHealth());
        // Nothing further to wait for after the settle time (the hold time does not apply here)
        loops((int) (Flywheel.openLoopSettleSeconds * 100) + 5, 0);
        assertTrue(flywheel.isReady());
        flywheel.shotFired();
        assertFalse(flywheel.isReady());
    }

    @Test
    public void silentEncoderWithHighCurrentIsJammedAndPowerIsCut() {
        when(motor.getCurrent(CurrentUnit.AMPS)).thenReturn(8.0);
        stallEncoder();
        assertEquals(Flywheel.Health.JAMMED, flywheel.getHealth());
        assertEquals(0, flywheel.power(), 0);
        verify(motor).setPower(0);
        assertFalse(flywheel.isReady());
        assertTrue(flywheel.faultMessage().contains("jammed"));

        // Stays off until reset() clears it
        loops(10, 0);
        assertEquals(0, flywheel.power(), 0);
        flywheel.reset();
        assertEquals(Flywheel.Health.OK, flywheel.getHealth());
    }

    @Test
    public void noFaultWhilePowerIsBelowTheMinimum() {
        // Target so low that the feedforward power stays under faultMinPower
        for (int i = 0; i < 400; i++) {
            nanos.addAndGet(STEP_NANOS);
            setRPM(0);
            flywheel.runToRPM(100);
        }
        assertEquals(Flywheel.Health.OK, flywheel.getHealth());
    }

    @Test
    public void readsRPMFromTicksPerSecond() {
        loop(1234);
        assertEquals(1234, flywheel.rawRPM(), 1e-6);
        assertEquals(1234, flywheel.filteredRPM(), 1e-6);
    }

    /** What the hub reports for a true velocity: it wraps into a signed 16-bit range. */
    private static double hubVelocity(double countsPerSecond) {
        return ((countsPerSecond + 32768) % 65536 + 65536) % 65536 - 32768;
    }

    /** Runs 10 ms loops with a Through Bore on encoderMotor turning at each RPM in turn. */
    private void throughBoreLoops(DcMotorEx encoderMotor, int sign, double... rpms) {
        int position = 0;
        for (double rpm : rpms) {
            double countsPerSecond = rpm * Flywheel.THROUGH_BORE_COUNTS_PER_REV / 60.0;
            position += (int) Math.round(countsPerSecond * STEP_NANOS / 1e9);
            nanos.addAndGet(STEP_NANOS);
            when(encoderMotor.getCurrentPosition()).thenReturn(sign * position);
            when(encoderMotor.getVelocity()).thenReturn(hubVelocity(sign * countsPerSecond));
            flywheel.runToRPM(TARGET);
        }
    }

    @Test
    public void throughBoreSpeedIsUnwrappedFromThe16BitVelocity() {
        flywheel.setCountsPerRev(Flywheel.THROUGH_BORE_COUNTS_PER_REV);
        flywheel.reset();
        // 3000 RPM is 409600 counts/s, which the hub wraps to 16384
        assertEquals(16384, hubVelocity(3000 * 8192 / 60.0), 0);
        throughBoreLoops(motor, 1, 2900, 2950, 3000);
        assertEquals(3000, flywheel.rawRPM(), 1e-6);
        // A ball slowing the wheel by 200 RPM in one loop is still read right
        throughBoreLoops(motor, 1, 3000, 2800);
        assertEquals(2800, flywheel.rawRPM(), 1e-6);
    }

    @Test
    public void reversedEncoderReadsForwards() {
        flywheel.setCountsPerRev(Flywheel.THROUGH_BORE_COUNTS_PER_REV);
        flywheel.setEncoderReversed(true);
        flywheel.reset();
        throughBoreLoops(motor, -1, 2900, 3000);
        assertEquals(3000, flywheel.rawRPM(), 1e-6);
    }

    @Test
    public void readsAnEncoderOnAnotherMotorsPort() throws Exception {
        HardwareMap map = hardwareMapWith(motor);
        DcMotorEx intake = mock(DcMotorEx.class);
        when(map.get(DcMotorEx.class, "Intake")).thenReturn(intake);
        flywheel = new Flywheel(map, "PollenFlywheel", DcMotorSimple.Direction.REVERSE, nanos::get);
        flywheel.setRpmFilterTimeConstant(0);
        flywheel.setCountsPerRev(Flywheel.THROUGH_BORE_COUNTS_PER_REV);
        flywheel.useEncoder(map, "Intake");
        flywheel.reset();

        setRPM(5000);  // the motor's own encoder is ignored
        throughBoreLoops(intake, 1, 2500);
        assertEquals(2500, flywheel.rawRPM(), 1e-6);
        assertEquals("", flywheel.missingDevices());
    }

    @Test
    public void missingEncoderIsReported() throws Exception {
        HardwareMap map = hardwareMapWith(motor);
        when(map.get(DcMotorEx.class, "Intake")).thenThrow(new IllegalArgumentException("not found"));
        flywheel.useEncoder(map, "Intake");
        assertEquals("\"Intake\"", flywheel.missingDevices());
        loop(TARGET);
        assertEquals(0, flywheel.rawRPM(), 0);
    }

    @Test
    public void missingMotorIsDisabledAndNeverReady() throws Exception {
        Flywheel missing = new Flywheel(hardwareMapWith(null), "PollenFlywheel",
                DcMotorSimple.Direction.REVERSE, nanos::get);
        missing.reset();
        assertFalse(missing.isAvailable());
        assertEquals("\"PollenFlywheel\"", missing.missingDevices());
        for (int i = 0; i < 20; i++) {
            nanos.addAndGet(STEP_NANOS);
            missing.runToRPM(TARGET);
            assertFalse(missing.isReady());
        }
        assertEquals(0, missing.filteredRPM(), 0);
        assertEquals(0, missing.power(), 0);
    }
}
