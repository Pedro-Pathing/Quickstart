package org.firstinspires.ftc.teamcode.helpers;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class UnitsTest {
    @Test
    public void convertsBothWays() {
        assertEquals(39.37007874, Units.metresToInches(1.0), 1e-8);
        assertEquals(0.0254, Units.inchesToMetres(1.0), 1e-15);
        assertEquals(72.0, Units.metresToInches(Units.inchesToMetres(72.0)), 1e-12);
    }
}
