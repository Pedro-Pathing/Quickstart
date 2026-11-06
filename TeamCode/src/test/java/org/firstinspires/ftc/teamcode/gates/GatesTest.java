package org.firstinspires.ftc.teamcode.gates;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class GatesTest {
    private double[] savedPositions;
    private long savedFeedMs;
    private Servo pollen, nectar;
    private Gates gates;

    @Before
    public void setUp() {
        savedPositions = new double[]{Gates.pollenOpenPosition, Gates.pollenClosedPosition,
                Gates.nectarOpenPosition, Gates.nectarClosedPosition};
        savedFeedMs = Gates.feedMs;

        // Mirrored servos: different values for each gate
        Gates.pollenOpenPosition = 0.9;
        Gates.pollenClosedPosition = 0.1;
        Gates.nectarOpenPosition = 0.2;
        Gates.nectarClosedPosition = 0.8;
        Gates.feedMs = 30;

        pollen = mock(Servo.class);
        nectar = mock(Servo.class);
        HardwareMap map = mock(HardwareMap.class);
        when(map.get(Servo.class, "pollenGate")).thenReturn(pollen);
        when(map.get(Servo.class, "nectarGate")).thenReturn(nectar);
        gates = new Gates(map);
    }

    @After
    public void restoreSettings() {
        Gates.pollenOpenPosition = savedPositions[0];
        Gates.pollenClosedPosition = savedPositions[1];
        Gates.nectarOpenPosition = savedPositions[2];
        Gates.nectarClosedPosition = savedPositions[3];
        Gates.feedMs = savedFeedMs;
    }

    @Test
    public void defaultsAreOpenOneClosedZero() {
        assertEquals(1.0, savedPositions[0], 0);
        assertEquals(0.0, savedPositions[1], 0);
        assertEquals(1.0, savedPositions[2], 0);
        assertEquals(0.0, savedPositions[3], 0);
    }

    @Test
    public void gatesStartClosedAtTheirOwnPositions() {
        verify(pollen).setPosition(0.1);
        verify(nectar).setPosition(0.8);
    }

    @Test
    public void eachGateOpensAtItsOwnPosition() {
        gates.feedPollen();
        verify(pollen).setPosition(0.9);
        verify(nectar, never()).setPosition(0.2);
        assertTrue(gates.isPollenOpen());
        assertFalse(gates.isNectarOpen());

        gates.feedNectar();
        verify(nectar).setPosition(0.2);
        assertTrue(gates.isNectarOpen());
    }

    @Test
    public void gatesCloseAfterFeedMsAtTheirOwnClosedPositions() throws InterruptedException {
        gates.feedPollen();
        gates.feedNectar();
        gates.update();
        assertTrue(gates.isPollenOpen());  // not yet
        assertTrue(gates.isNectarOpen());

        Thread.sleep(Gates.feedMs + 50);
        gates.update();
        assertFalse(gates.isPollenOpen());
        assertFalse(gates.isNectarOpen());
        // Closed again, each to its own value (the construction call is the first one)
        org.mockito.Mockito.verify(pollen, org.mockito.Mockito.times(2)).setPosition(0.1);
        org.mockito.Mockito.verify(nectar, org.mockito.Mockito.times(2)).setPosition(0.8);
    }
}
