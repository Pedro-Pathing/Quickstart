package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.HiveCells;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretHistory;

import java.util.List;

public final class CellLocator {
    public static class TagPoint {
        public final int id;

        public final int index;
        public final double forward;
        public final double left;
        public final double up;

        public TagPoint(int id, int index, double forward, double left, double up) {
            this.id = id;
            this.index = index;
            this.forward = forward;
            this.left = left;
            this.up = up;
        }
    }

    public static class CameraMount {
        public final double forward;
        public final double left;
        public final double up;
        public final double pitch;
        public final double yaw;
        public final double roll;

        public CameraMount(double forward, double left, double up, double pitch, double yaw) {
            this(forward, left, up, pitch, yaw, 0);
        }

        public CameraMount(double forward, double left, double up, double pitch, double yaw, double roll) {
            this.forward = forward;
            this.left = left;
            this.up = up;
            this.pitch = pitch;
            this.yaw = yaw;
            this.roll = roll;
        }
    }

    public static double[] cameraToTurret(double xImage, double yImage, double zForward, CameraMount mount) {
        double cr = Math.cos(mount.roll), sr = Math.sin(mount.roll);
        double xRight = xImage * cr + yImage * sr;
        double yDown = yImage * cr - xImage * sr;
        double cp = Math.cos(mount.pitch), sp = Math.sin(mount.pitch);
        double cy = Math.cos(mount.yaw), sy = Math.sin(mount.yaw);

        double[] fwd = {cp * cy, cp * sy, sp};
        double[] right = {sy, -cy, 0};
        double[] down = {sp * cy, sp * sy, -cp};
        return new double[]{
                mount.forward + xRight * right[0] + yDown * down[0] + zForward * fwd[0],
                mount.left + xRight * right[1] + yDown * down[1] + zForward * fwd[1],
                mount.up + xRight * right[2] + yDown * down[2] + zForward * fwd[2]
        };
    }

    public static double measureRowYaw(List<TagPoint> tags) {
        double sumF = 0, sumL = 0;
        for (int i = 0; i < tags.size(); i++) {
            for (int j = i + 1; j < tags.size(); j++) {
                TagPoint a = tags.get(i), b = tags.get(j);
                double span = HiveCells.TAG_OFFSETS_IN[b.index] - HiveCells.TAG_OFFSETS_IN[a.index];
                if (span == 0) continue;

                sumF += (b.forward - a.forward) * Math.signum(span);
                sumL += (b.left - a.left) * Math.signum(span);
            }
        }
        return sumF == 0 && sumL == 0 ? Double.NaN : Math.atan2(sumL, sumF);
    }

    public static double[] rowCenter(List<TagPoint> tags, double rowYaw) {
        double cos = Math.cos(rowYaw), sin = Math.sin(rowYaw);
        double f = 0, l = 0;
        for (TagPoint tag : tags) {
            double offset = HiveCells.TAG_OFFSETS_IN[tag.index];
            f += tag.forward - offset * cos;
            l += tag.left - offset * sin;
        }
        return new double[]{f / tags.size(), l / tags.size()};
    }

    public static double[] turretToField(double forward, double left, TurretHistory.Sample at,
                                         double pivotForward, double pivotLeft) {
        double ch = Math.cos(at.heading), sh = Math.sin(at.heading);
        double pivotX = at.x + pivotForward * ch - pivotLeft * sh;
        double pivotY = at.y + pivotForward * sh + pivotLeft * ch;
        double a = at.heading + at.turretAngle;
        return new double[]{
                pivotX + forward * Math.cos(a) - left * Math.sin(a),
                pivotY + forward * Math.sin(a) + left * Math.cos(a)
        };
    }

    public static CellObservation locate(HiveCells.Cell cell, List<TagPoint> tags, TurretHistory.Sample at,
                                         double pivotForward, double pivotLeft) {
        double measuredYaw = measureRowYaw(tags);
        double rowYaw = Double.isNaN(measuredYaw) ? wrap(cell.pose.heading() - at.heading - at.turretAngle) : measuredYaw;
        double[] center = rowCenter(tags, rowYaw);
        double[] field = turretToField(center[0], center[1], at, pivotForward, pivotLeft);
        int[] ids = new int[tags.size()];
        for (int i = 0; i < ids.length; i++) ids[i] = tags.get(i).id;
        return new CellObservation(cell, ids, center[0], center[1], measuredYaw,
                field[0], field[1], at.time, at.turretAngle, at.heading);
    }

    static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private CellLocator() {
    }
}
