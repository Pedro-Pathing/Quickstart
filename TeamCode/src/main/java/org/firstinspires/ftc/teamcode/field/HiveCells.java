package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

public final class HiveCells {
    public static final double[] TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    public enum Cell {
        RED_FAR(Field.Alliance.RED, 30, Field.RED_CELL_BOTTOM),
        RED_AUDIENCE(Field.Alliance.RED, 34, Field.RED_CELL_TOP),
        BLUE_AUDIENCE(Field.Alliance.BLUE, 38, Field.BLUE_CELL_TOP),
        BLUE_FAR(Field.Alliance.BLUE, 42, Field.BLUE_CELL_BOTTOM);

        public final Field.Alliance alliance;
        public final int firstTagId;
        public final Pose pose;

        Cell(Field.Alliance alliance, int firstTagId, Pose pose) {
            this.alliance = alliance;
            this.firstTagId = firstTagId;
            this.pose = pose;
        }

        public int tagIndex(int tagId) {
            int index = tagId - firstTagId;
            return index >= 0 && index < TAG_OFFSETS_IN.length ? index : -1;
        }
    }

    public static Cell forTag(int tagId) {
        for (Cell cell : Cell.values()) {
            if (cell.tagIndex(tagId) >= 0) return cell;
        }
        return null;
    }

    public static Cell forRobot(Field.Alliance alliance, Pose robotPose) {
        boolean top = robotPose.y() >= Field.CENTER_Y;
        if (alliance == Field.Alliance.RED) return top ? Cell.RED_AUDIENCE : Cell.RED_FAR;
        return top ? Cell.BLUE_AUDIENCE : Cell.BLUE_FAR;
    }

    private HiveCells() {
    }
}
