package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.launch.LaunchModel;

/**
 * A ball on the floor seen by the floor pipeline, relative to the robot when the image was taken.
 *
 * Robot frame: +X forward, +Y left, origin at the robot's centre of rotation. Metres and radians.
 */
public class FloorBall {
    public enum Kind {
        POLLEN(1, LaunchModel.Launcher.POLLEN.ballDiameter),
        RED_NECTAR(2, LaunchModel.Launcher.NECTAR.ballDiameter),
        BLUE_NECTAR(3, LaunchModel.Launcher.NECTAR.ballDiameter);

        /** The number floor_balls.py reports for this kind. */
        public final int code;
        public final double diameter;

        Kind(int code, double diameter) {
            this.code = code;
            this.diameter = diameter;
        }

        static Kind fromCode(int code) {
            for (Kind kind : values()) {
                if (kind.code == code) return kind;
            }
            return null;
        }
    }

    public final Kind kind;
    /** Position of the ball's centre on the floor in the robot frame. */
    public final double x, y;
    /** Touching the bottom of the image: the ball is about to pass out of view under the camera. */
    public final boolean atBottomEdge;

    public FloorBall(Kind kind, double x, double y, boolean atBottomEdge) {
        this.kind = kind;
        this.x = x;
        this.y = y;
        this.atBottomEdge = atBottomEdge;
    }

    /** Pollen, or Nectar of our alliance's colour. */
    public boolean isOurs(Alliance alliance) {
        return kind == Kind.POLLEN || isNectarOf(alliance);
    }

    /** Nectar of the other alliance's colour, which we must leave alone. */
    public boolean isTheirNectar(Alliance alliance) {
        return kind != Kind.POLLEN && !isNectarOf(alliance);
    }

    private boolean isNectarOf(Alliance alliance) {
        return (kind == Kind.RED_NECTAR && alliance == Alliance.RED)
                || (kind == Kind.BLUE_NECTAR && alliance == Alliance.BLUE);
    }

    public double distance() {
        return Math.hypot(x, y);
    }

    /** Angle from the robot's forward direction to the ball, positive to the left (CCW). */
    public double bearing() {
        return Math.atan2(y, x);
    }
}
