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
import java.util.Random;
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
        return hardwareMapWith(motor, 12.0);
    }

    private static HardwareMap hardwareMapWith(DcMotorEx motor, double batteryVolts) throws Exception {
        HardwareMap map = mock(HardwareMap.class);
        if (motor != null) {
            when(map.get(DcMotorEx.class, "PollenFlywheel")).thenReturn(motor);
        } else {
            when(map.get(DcMotorEx.class, "PollenFlywheel")).thenThrow(new IllegalArgumentException("not found"));
        }
        VoltageSensor battery = mock(VoltageSensor.class);
        when(battery.getVoltage()).thenReturn(batteryVolts);
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

    private double throughBorePosition = 0;  // counts, at the times the loop timer is read
    private final Random jitter = new Random(1);

    /**
     * Runs one 10 ms loop with a Through Bore on encoderMotor turning at rpm. As on the robot, the
     * position is sampled (during the bulk read) jitterMs away from when the loop timer is read.
     */
    private void throughBoreLoop(DcMotorEx encoderMotor, int sign, double rpm, double jitterMs) {
        double countsPerSecond = rpm * Flywheel.THROUGH_BORE_COUNTS_PER_REV / 60.0;
        throughBorePosition += countsPerSecond * STEP_NANOS / 1e9;
        double sampled = throughBorePosition + countsPerSecond * jitterMs / 1000.0;
        nanos.addAndGet(STEP_NANOS);
        when(encoderMotor.getCurrentPosition()).thenReturn(sign * (int) Math.round(sampled));
        when(encoderMotor.getVelocity()).thenReturn(hubVelocity(sign * countsPerSecond));
        flywheel.runToRPM(TARGET);
    }

    /** Loops at each RPM in turn with up to maxJitterMs of sampling jitter, checking every reading. */
    private void throughBoreLoopsReadExactly(DcMotorEx encoderMotor, int sign, double maxJitterMs, double... rpms) {
        for (double rpm : rpms) {
            throughBoreLoop(encoderMotor, sign, rpm, (jitter.nextDouble() * 2 - 1) * maxJitterMs);
            assertEquals(rpm, flywheel.rawRPM(), 1e-6);
        }
    }

    /** The speeds of a 1 s spin-up from rest to rpm, one per 10 ms loop, then holdLoops at rpm. */
    private static double[] spinUp(double rpm, int holdLoops) {
        return ramp(0, rpm, holdLoops);
    }

    /** The speeds of a 1 s ramp from one speed to another, one per 10 ms loop, then holdLoops at the end speed. */
    private static double[] ramp(double fromRPM, double toRPM, int holdLoops) {
        double[] rpms = new double[100 + holdLoops];
        for (int i = 0; i < rpms.length; i++) {
            rpms[i] = fromRPM + (toRPM - fromRPM) * Math.min(1.0, (i + 1) / 100.0);
        }
        return rpms;
    }

    private void useThroughBore() {
        flywheel.setCountsPerRev(Flywheel.THROUGH_BORE_COUNTS_PER_REV);
        flywheel.reset();
    }

    @Test
    public void throughBoreSpeedIsUnwrappedFromThe16BitVelocity() {
        useThroughBore();
        // 3000 RPM is 409600 counts/s, which the hub wraps to 16384
        assertEquals(16384, hubVelocity(3000 * 8192 / 60.0), 0);
        throughBoreLoopsReadExactly(motor, 1, 0, spinUp(3000, 50));
        // A ball slowing the wheel by 230 RPM in one loop, then recovering, is still read right
        throughBoreLoopsReadExactly(motor, 1, 0, 2770, 2800, 2850, 2900, 2950, 3000);
    }

    @Test
    public void noSpikesWhenThePositionIsSampledAtAVariableTime() {
        // Up to 4 ms either way in a 10 ms loop: 40% out, enough to put a speed estimated from
        // the change in position over one loop out by several wraps
        useThroughBore();
        throughBoreLoopsReadExactly(motor, 1, 4, spinUp(3000, 300));
        throughBoreLoopsReadExactly(motor, 1, 4, 2850, 2750, 2800, 2900, 3000, 3000, 3000);
        throughBoreLoopsReadExactly(motor, 1, 4, ramp(3000, 6000, 300));
    }

    @Test
    public void readsAWheelAlreadySpinningWhenTheOpModeStarts() throws Exception {
        // A new OpMode's flywheel has no previous reading, but the wheel is still coasting at
        // 3000 RPM from the last one. The first reads (the first 0.1 s) go by the change in position.
        throughBorePosition = 123456;
        when(motor.getCurrentPosition()).thenReturn(123456);
        when(motor.getVelocity()).thenReturn(hubVelocity(409600));
        flywheel = new Flywheel(hardwareMapWith(motor), "PollenFlywheel",
                DcMotorSimple.Direction.REVERSE, nanos::get);
        flywheel.setRpmFilterTimeConstant(0);
        useThroughBore();
        for (int i = 0; i < 10; i++) throughBoreLoop(motor, 1, 3000, 0);
        throughBoreLoopsReadExactly(motor, 1, 2, ramp(3000, 2000, 100));
    }

    @Test
    public void putsAMissedWrapRightWithinTheCheckWindow() {
        // The wheel speeds up by 300 RPM in one loop (more than half a wrap), so the first reading
        // after that is a wrap out; the check against the position puts it right within 0.6 s
        useThroughBore();
        throughBoreLoopsReadExactly(motor, 1, 0, spinUp(2700, 10));
        for (int i = 0; i < 60; i++) throughBoreLoop(motor, 1, 3000, 1);
        throughBoreLoopsReadExactly(motor, 1, 2, ramp(3000, 3000, 50));
    }

    @Test
    public void reversedEncoderReadsForwards() {
        flywheel.setEncoderReversed(true);
        useThroughBore();
        throughBoreLoopsReadExactly(motor, -1, 2, spinUp(3000, 20));
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
        throughBoreLoopsReadExactly(intake, 1, 2, spinUp(2500, 20));
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
    public void closedLoopFeedforwardIsScaledForTheBattery() throws Exception {
        // The pollen launcher's fitted gains (for 12 V), on a fresh 13.8 V battery, feedback off
        flywheel = new Flywheel(hardwareMapWith(motor, 13.8), "PollenFlywheel",
                DcMotorSimple.Direction.REVERSE, nanos::get);
        flywheel.setRpmFilterTimeConstant(0);
        flywheel.setPID(0, 0, 0);
        flywheel.setFeedforward(0.000151398, 0.211076);
        flywheel.reset();
        loops(100, TARGET);  // 1 s at 3000 RPM: long enough for the slew limit
        // 0.665 at 12 V; unscaled on a 13.8 V battery it overshot by about 650 RPM
        double atNominal = 0.211076 + 0.000151398 * TARGET;
        assertEquals(atNominal * 12.0 / 13.8, flywheel.power(), 0.002);
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
