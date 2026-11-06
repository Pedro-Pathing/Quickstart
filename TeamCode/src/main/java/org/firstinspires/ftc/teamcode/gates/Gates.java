package org.firstinspires.ftc.teamcode.gates;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.Devices;

/**
 * The Pollen and Nectar feed gates. feedPollen() / feedNectar() open a gate for feedMs to let one
 * ball through; call update() every loop to close them again.
 * A gate whose servo isn't in the configuration is disabled: it never opens.
 */
@Configurable
public class Gates {
    public static double openPosition = 1.0;
    public static double closedPosition = 0.0;
    /** How long a gate stays open per ball. */
    public static long feedMs = 250;

    private static final String POLLEN_NAME = "pollenGate", NECTAR_NAME = "nectarGate";

    private final Servo pollen, nectar;  // null when that gate is disabled
    private final ElapsedTime pollenTimer = new ElapsedTime(), nectarTimer = new ElapsedTime();

    // Remembered states, so each servo is only written when its gate changes
    private boolean pollenOpen = false, nectarOpen = false;

    public Gates(HardwareMap hardwareMap) {
        pollen = Devices.get(hardwareMap, Servo.class, POLLEN_NAME, s -> s.setPosition(closedPosition));
        nectar = Devices.get(hardwareMap, Servo.class, NECTAR_NAME, s -> s.setPosition(closedPosition));
    }

    /** Names of missing gate servos in quotes, or "" if both were found. */
    public String missingDevices() {
        return Devices.join(Devices.missing(pollen, POLLEN_NAME), Devices.missing(nectar, NECTAR_NAME));
    }

    public void feedPollen() {
        if (pollen == null) return;
        if (!pollenOpen) {
            pollen.setPosition(openPosition);
            pollenOpen = true;
        }
        pollenTimer.reset();
    }

    public void feedNectar() {
        if (nectar == null) return;
        if (!nectarOpen) {
            nectar.setPosition(openPosition);
            nectarOpen = true;
        }
        nectarTimer.reset();
    }

    /** Call every loop: closes each gate again after feedMs. */
    public void update() {
        if (pollenOpen && pollenTimer.milliseconds() > feedMs) {
            pollen.setPosition(closedPosition);
            pollenOpen = false;
        }
        if (nectarOpen && nectarTimer.milliseconds() > feedMs) {
            nectar.setPosition(closedPosition);
            nectarOpen = false;
        }
    }

    public boolean isPollenOpen() {
        return pollenOpen;
    }

    public boolean isNectarOpen() {
        return nectarOpen;
    }
}
