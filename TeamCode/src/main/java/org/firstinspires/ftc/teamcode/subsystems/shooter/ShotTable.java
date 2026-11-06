package org.firstinspires.ftc.teamcode.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class ShotTable {
    public static class Shot {
        public final double distance;
        public final double velocity;

        Shot(double distance, double velocity) {
            this.distance = distance;
            this.velocity = velocity;
        }
    }

    private final double mergeDistance;
    private final List<Shot> recorded = new ArrayList<>();

    public ShotTable(double mergeDistance) {
        this.mergeDistance = mergeDistance;
    }

    public void record(double distance, double velocity) {
        if (Double.isNaN(distance) || Double.isNaN(velocity)) return;
        for (int i = recorded.size() - 1; i >= 0; i--) {
            if (Math.abs(recorded.get(i).distance - distance) <= mergeDistance) recorded.remove(i);
        }
        recorded.add(new Shot(distance, velocity));
    }

    public void clear() {
        recorded.clear();
    }

    public List<Shot> sorted() {
        List<Shot> shots = new ArrayList<>(recorded);
        shots.sort((a, b) -> Double.compare(a.distance, b.distance));
        return shots;
    }

    public int size() {
        return recorded.size();
    }

    public String distanceLine() {
        StringBuilder line = new StringBuilder("SHOOTER_DIST_IN = {");
        List<Shot> shots = sorted();
        for (int i = 0; i < shots.size(); i++) {
            if (i > 0) line.append(", ");
            line.append(String.format(Locale.US, "%.1f", shots.get(i).distance));
        }
        return line.append("};").toString();
    }

    public String velocityLine() {
        StringBuilder line = new StringBuilder("SHOOTER_RPM = {");
        List<Shot> shots = sorted();
        for (int i = 0; i < shots.size(); i++) {
            if (i > 0) line.append(", ");
            line.append(String.format(Locale.US, "%.0f", shots.get(i).velocity));
        }
        return line.append("};").toString();
    }
}
