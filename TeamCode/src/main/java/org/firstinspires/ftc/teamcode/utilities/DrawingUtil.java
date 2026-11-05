package org.firstinspires.ftc.teamcode.utilities;

import com.bylazar.field.FieldManager;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Envelope;

public class DrawingUtil {

  public static void drawRobotOnField(
      FieldManager field,
      double x,
      double y,
      double h,
      double turretH,
      double targetX,
      double targetY) {
    if (field == null) return;

    field.setStyle("none", "cyan", 2.0);
    field.moveCursor(x, y);
    field.line(targetX, targetY);

    double cosA = Math.cos(h);
    double sinA = Math.sin(h);
    double halfSize = config.sentinel.robot_width / 2.0;

    double xFL = x + (halfSize * cosA - halfSize * sinA);
    double yFL = y + (halfSize * sinA + halfSize * cosA);

    double xFR = x + (halfSize * cosA - (-halfSize) * sinA);
    double yFR = y + (halfSize * sinA + (-halfSize) * cosA);

    double xBR = x + (-halfSize * cosA - (-halfSize) * sinA);
    double yBR = y + (-halfSize * sinA + (-halfSize) * cosA);

    double xBL = x + (-halfSize * cosA - halfSize * sinA);
    double yBL = y + (-halfSize * sinA + halfSize * cosA);

    field.setStyle("none", "red", 2.0);
    field.moveCursor(xFL, yFL);
    field.line(xFR, yFR);
    field.moveCursor(xFR, yFR);
    field.line(xBR, yBR);
    field.moveCursor(xBR, yBR);
    field.line(xBL, yBL);
    field.moveCursor(xBL, yBL);
    field.line(xFL, yFL);

    double frontX = x + (halfSize * cosA);
    double frontY = y + (halfSize * sinA);

    field.setStyle("none", "yellow", 2.0);
    field.moveCursor(x, y);
    field.line(frontX, frontY);

    double turretCos = Math.cos(turretH);
    double turretSin = Math.sin(turretH);

    double turretX = x + (12.0 * turretCos);
    double turretY = y + (12.0 * turretSin);

    field.setStyle("none", "green", 2.0);
    field.moveCursor(x, y);
    field.line(turretX, turretY);

    field.update();
  }

  /**
   * Draws the BIOBUZZ landmarks this alliance cares about: the protected HIVE frame zones (red),
   * the LOADING ZONE (green), the GARDEN (yellow) and the two alliance-side FLOWERS.
   */
  public static void drawFieldZones(FieldManager field, Sentinel sentinel) {
    if (field == null || sentinel == null) return;
    field.setStyle("stroke", "red", 1.0);
    for (Envelope zone : sentinel.getProtectedZones()) {
      drawRect(field, zone.getMinX(), zone.getMinY(), zone.getMaxX(), zone.getMaxY());
    }

    Envelope loading = sentinel.field().loadingZone();
    field.setStyle("stroke", "green", 1.0);
    drawRect(field, loading.getMinX(), loading.getMinY(), loading.getMaxX(), loading.getMaxY());

    Envelope garden = sentinel.field().garden();
    field.setStyle("stroke", "yellow", 1.0);
    drawRect(field, garden.getMinX(), garden.getMinY(), garden.getMaxX(), garden.getMaxY());

    for (org.locationtech.jts.geom.Coordinate flower : sentinel.field().allianceSideFlowers()) {
      field.moveCursor(flower.x, flower.y);
      field.circle(2.0);
    }
  }

  private static void drawRect(FieldManager field, double x1, double y1, double x2, double y2) {
    field.moveCursor(x1, y1);
    field.line(x2, y1);
    field.moveCursor(x2, y1);
    field.line(x2, y2);
    field.moveCursor(x2, y2);
    field.line(x1, y2);
    field.moveCursor(x1, y2);
    field.line(x1, y1);
  }
}
