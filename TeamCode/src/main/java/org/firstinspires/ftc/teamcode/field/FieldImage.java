package org.firstinspires.ftc.teamcode.field;

import com.acmerobotics.dashboard.canvas.Canvas;

public final class FieldImage {
    public static final String BIOBUZZ = "/dash/biobuzz.png";

    public static void drawPedro(Canvas canvas) {
        double half = Field.WIDTH / 2;
        canvas.drawImage(BIOBUZZ, half, half, Field.WIDTH, Field.WIDTH, Math.PI / 2, half, half, true);
        canvas.setRotation(-Math.PI / 2);
        canvas.setTranslation(-half, half);
    }

    private FieldImage() {
    }
}
