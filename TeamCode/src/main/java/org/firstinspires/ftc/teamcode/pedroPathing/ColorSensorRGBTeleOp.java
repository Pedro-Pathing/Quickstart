package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp для распознавания фиолетового и зеленого цветов
 * с помощью 3 цветовых датчиков (c1, c2, c3)
 * и управления 3 RGB светодиодами GoBilda (rgb1, rgb2, rgb3)
 */
@TeleOp(name = "Color Sensor RGB TeleOp", group = "Test")
public class ColorSensorRGBTeleOp extends LinearOpMode {

    /* ================= HARDWARE ================= */
    
    // Цветовые датчики
    private NormalizedColorSensor c1, c2, c3;
    
    // RGB светодиоды GoBilda
    // Каждый RGB светодиод управляется одним сервоприводом
    private Servo rgb1;  // RGB светодиод 1
    private Servo rgb2;  // RGB светодиод 2
    private Servo rgb3;  // RGB светодиод 3
    
    /* ================= COLOR DETECTION ================= */
    
    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }
    
    // Пороги для определения цветов
    private static final float MIN_INTENSITY = 0.1f;
    private static final float GREEN_RATIO_THRESHOLD = 0.35f;
    private static final float BLUE_RATIO_THRESHOLD = 0.35f;
    
    /* ================= RGB LED VALUES ================= */
    
    // Позиции сервопривода для разных цветов (0.0 - 1.0)
    // Эти значения могут потребовать настройки в зависимости от вашей конфигурации GoBilda RGB
    // Если RGB не горят, попробуйте изменить эти значения
    // Некоторые GoBilda RGB используют диапазон 0.5-1.0 или другие диапазоны
    
    // Вариант 1: Стандартный диапазон 0.0-1.0
    private static final double SERVO_POSITION_GREEN = 0.25;   // Позиция для зеленого цвета
    private static final double SERVO_POSITION_PURPLE = 0.66;  // Позиция для фиолетового цвета
    private static final double SERVO_POSITION_RED = 0.0;      // Позиция для красного цвета (нераспознанный цвет)
    
    // Вариант 2: Если не работает, попробуйте раскомментировать эти значения:
    // private static final double SERVO_POSITION_GREEN = 0.5;   // Попробуйте 0.5
    // private static final double SERVO_POSITION_PURPLE = 0.75; // Попробуйте 0.75
    // private static final double SERVO_POSITION_RED = 0.5;     // Попробуйте 0.5
    
    // Вариант 3: Или попробуйте полный диапазон:
    // private static final double SERVO_POSITION_GREEN = 0.33;
    // private static final double SERVO_POSITION_PURPLE = 0.66;
    // private static final double SERVO_POSITION_RED = 1.0;
    
    private static final double SERVO_POSITION_OFF = 0.0;      // Позиция для выключенного состояния (при старте/остановке)
    
    // Старые RGB значения (оставлены для совместимости, если понадобятся)
    private static final double PURPLE_RED = 0.5;
    private static final double PURPLE_GREEN = 0.0;
    private static final double PURPLE_BLUE = 0.5;
    
    private static final double GREEN_RED = 0.0;
    private static final double GREEN_GREEN = 1.0;
    private static final double GREEN_BLUE = 0.0;
    
    private static final double OFF_RED = 0.0;
    private static final double OFF_GREEN = 0.0;
    private static final double OFF_BLUE = 0.0;
    
    @Override
    public void runOpMode() {
        
        /* ---------- INIT ---------- */
        
        // Инициализация цветовых датчиков
        c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
        c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
        c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");
        
        // Установка усиления для датчиков
        c1.setGain(8);
        c2.setGain(8);
        c3.setGain(8);
        
        // Инициализация RGB светодиодов
        // Каждый RGB светодиод управляется одним сервоприводом
        try {
            rgb1 = hardwareMap.get(Servo.class, "rgb1");
            rgb2 = hardwareMap.get(Servo.class, "rgb2");
            rgb3 = hardwareMap.get(Servo.class, "rgb3");
            
            // Проверка инициализации
            if (rgb1 == null || rgb2 == null || rgb3 == null) {
                telemetry.addLine("❌ Ошибка: RGB сервоприводы не найдены!");
                telemetry.update();
                sleep(2000);
            }
        } catch (Exception e) {
            telemetry.addLine("❌ Ошибка инициализации RGB: " + e.getMessage());
            telemetry.update();
            sleep(2000);
        }
        
        // ТЕСТОВЫЙ РЕЖИМ: Перебираем все позиции для поиска рабочих значений
        telemetry.addLine("🔍 ТЕСТОВЫЙ РЕЖИМ RGB");
        telemetry.addLine("Перебираем позиции от 0.0 до 1.0");
        telemetry.addLine("Нажмите START для начала работы");
        telemetry.update();
        
        waitForStart();
        
        // Тест: перебираем позиции для каждого RGB
        telemetry.addLine("=== ТЕСТ RGB1 (красный) ===");
        telemetry.update();
        for (double pos = 0.0; pos <= 1.0; pos += 0.1) {
            if (!opModeIsActive()) break;
            setRGBColor(rgb1, pos);
            telemetry.addData("RGB1 позиция", "%.2f", pos);
            telemetry.update();
            sleep(500); // Даем время увидеть цвет
        }
        
        sleep(1000);
        
        telemetry.addLine("=== ТЕСТ RGB2 (зеленый) ===");
        telemetry.update();
        for (double pos = 0.0; pos <= 1.0; pos += 0.1) {
            if (!opModeIsActive()) break;
            setRGBColor(rgb2, pos);
            telemetry.addData("RGB2 позиция", "%.2f", pos);
            telemetry.update();
            sleep(500);
        }
        
        sleep(1000);
        
        telemetry.addLine("=== ТЕСТ RGB3 (фиолетовый) ===");
        telemetry.update();
        for (double pos = 0.0; pos <= 1.0; pos += 0.1) {
            if (!opModeIsActive()) break;
            setRGBColor(rgb3, pos);
            telemetry.addData("RGB3 позиция", "%.2f", pos);
            telemetry.update();
            sleep(500);
        }
        
        sleep(2000);
        
        // Устанавливаем начальные позиции
        setRGBColor(rgb1, SERVO_POSITION_RED);
        setRGBColor(rgb2, SERVO_POSITION_GREEN);
        setRGBColor(rgb3, SERVO_POSITION_PURPLE);
        
        telemetry.addLine("✅ Тест завершен");
        telemetry.addLine("Запомните позиции, при которых RGB горели правильно");
        telemetry.addLine("Теперь начинается основная работа");
        telemetry.update();
        sleep(2000);
        
        /* ---------- LOOP ---------- */
        
        while (opModeIsActive()) {
            
            // Определение цветов для каждого датчика
            DetectedColor color1 = detectColor(c1, "Sensor 1");
            DetectedColor color2 = detectColor(c2, "Sensor 2");
            DetectedColor color3 = detectColor(c3, "Sensor 3");
            
            // Управление RGB светодиодами в зависимости от распознанного цвета
            updateRGBLED(rgb1, color1);
            updateRGBLED(rgb2, color2);
            updateRGBLED(rgb3, color3);
            
            // Телеметрия
            telemetry.addLine("=== Color Detection ===");
            telemetry.addData("Sensor 1 (c1)", color1);
            telemetry.addData("Sensor 2 (c2)", color2);
            telemetry.addData("Sensor 3 (c3)", color3);
            telemetry.addLine();
            telemetry.addLine("=== RGB LEDs ===");
            telemetry.addData("RGB1", getColorName(color1));
            telemetry.addData("RGB2", getColorName(color2));
            telemetry.addData("RGB3", getColorName(color3));
            telemetry.addLine();
            telemetry.addLine("=== RGB Positions ===");
            telemetry.addData("RGB1 pos", getServoPosition(color1));
            telemetry.addData("RGB2 pos", getServoPosition(color2));
            telemetry.addData("RGB3 pos", getServoPosition(color3));
            telemetry.update();
        }
        
        // Выключить все светодиоды при остановке
        setRGBColor(rgb1, SERVO_POSITION_OFF);
        setRGBColor(rgb2, SERVO_POSITION_OFF);
        setRGBColor(rgb3, SERVO_POSITION_OFF);
    }
    
    /* ================= COLOR DETECTION ================= */
    
    /**
     * Определяет цвет по данным цветового датчика
     * @param sensor Цветовой датчик
     * @param sensorName Имя датчика для телеметрии
     * @return Распознанный цвет
     */
    private DetectedColor detectColor(NormalizedColorSensor sensor, String sensorName) {
        if (sensor == null) {
            telemetry.addData(sensorName + " Error", "Sensor not initialized");
            return DetectedColor.UNKNOWN;
        }
        
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        
        // Нормализация цветов
        if (colors.alpha > 0) {
            normRed = colors.red / colors.alpha;
            normGreen = colors.green / colors.alpha;
            normBlue = colors.blue / colors.alpha;
        } else {
            normRed = colors.red;
            normGreen = colors.green;
            normBlue = colors.blue;
        }
        
        // Проверка интенсивности света
        float totalIntensity = normRed + normGreen + normBlue;
        if (totalIntensity < MIN_INTENSITY) {
            return DetectedColor.UNKNOWN;
        }
        
        // Вычисление соотношений цветов
        float redRatio = normRed / totalIntensity;
        float greenRatio = normGreen / totalIntensity;
        float blueRatio = normBlue / totalIntensity;
        
        // Определение доминирующего компонента
        float maxComponent = Math.max(Math.max(normRed, normGreen), normBlue);
        
        // Логика определения ЗЕЛЕНОГО цвета:
        // - Зеленый компонент доминирует
        // - Зеленое соотношение > 0.35
        // - Зеленый значительно выше красного и синего
        boolean isGreen = (normGreen == maxComponent) && 
                         (greenRatio > GREEN_RATIO_THRESHOLD) &&
                         (normGreen > normRed * 1.2) && 
                         (normGreen > normBlue * 1.2) &&
                         (normGreen > 0.3);
        
        // Логика определения ФИОЛЕТОВОГО цвета:
        // - Синий компонент значителен
        // - Красный компонент значителен
        // - Синий >= зеленого
        // - Красный + Синий вместе превышают зеленый
        boolean isPurple = (normBlue > 0.2) &&                    // Синий значителен
                          (normRed > 0.15) &&                    // Красный значителен
                          (normBlue >= normGreen) &&             // Синий >= зеленого
                          (normBlue >= normRed * 0.6) &&         // Синий >= 60% красного
                          ((normRed + normBlue) > normGreen * 1.2) &&  // Красный+Синий > зеленый на 20%
                          (blueRatio > BLUE_RATIO_THRESHOLD);    // Синее соотношение > 0.35
        
        if (isGreen) {
            return DetectedColor.GREEN;
        } else if (isPurple) {
            return DetectedColor.PURPLE;
        } else {
            return DetectedColor.UNKNOWN;
        }
    }
    
    /* ================= RGB LED CONTROL ================= */
    
    /**
     * Обновляет цвет RGB светодиода в зависимости от распознанного цвета
     * @param rgbLED Сервопривод RGB светодиода
     * @param color Распознанный цвет
     */
    private void updateRGBLED(Servo rgbLED, DetectedColor color) {
        switch (color) {
            case GREEN:
                setRGBColor(rgbLED, SERVO_POSITION_GREEN);
                break;
            case PURPLE:
                setRGBColor(rgbLED, SERVO_POSITION_PURPLE);
                break;
            case UNKNOWN:
            default:
                // Когда цвет не распознан, горит красным
                setRGBColor(rgbLED, SERVO_POSITION_RED);
                break;
        }
    }
    
    /**
     * Устанавливает позицию сервопривода для RGB светодиода GoBilda
     * 
     * @param rgbLED Сервопривод RGB светодиода
     * @param position Позиция сервопривода (0.0 - 1.0)
     */
    private void setRGBColor(Servo rgbLED, double position) {
        if (rgbLED == null) {
            telemetry.addLine("⚠️ RGB LED is null!");
            return;
        }
        
        // Ограничение позиции в диапазоне 0.0 - 1.0
        position = Math.max(0.0, Math.min(1.0, position));
        
        // Установка позиции сервопривода
        try {
            rgbLED.setPosition(position);
        } catch (Exception e) {
            telemetry.addLine("❌ Ошибка установки позиции RGB: " + e.getMessage());
        }
    }
    
    /**
     * Устанавливает цвет RGB светодиода через RGB компоненты (альтернативный метод)
     * Преобразует RGB значения в позицию сервопривода
     * 
     * @param rgbLED Сервопривод RGB светодиода
     * @param red Значение красного канала (0.0 - 1.0)
     * @param green Значение зеленого канала (0.0 - 1.0)
     * @param blue Значение синего канала (0.0 - 1.0)
     */
    private void setRGBColor(Servo rgbLED, double red, double green, double blue) {
        if (rgbLED == null) return;
        
        // Преобразование RGB значений в одно значение для сервопривода
        // Используем взвешенное среднее для получения комбинированного цвета
        double combinedValue = (red * 0.33 + green * 0.33 + blue * 0.34);
        
        // Ограничение в диапазоне 0.0 - 1.0
        combinedValue = Math.max(0.0, Math.min(1.0, combinedValue));
        
        // Установка позиции сервопривода
        rgbLED.setPosition(combinedValue);
    }
    
    /**
     * Возвращает строковое представление цвета для телеметрии
     */
    private String getColorName(DetectedColor color) {
        switch (color) {
            case GREEN:
                return "GREEN";
            case PURPLE:
                return "PURPLE";
            case UNKNOWN:
            default:
                return "RED";
        }
    }
    
    /**
     * Возвращает позицию сервопривода для цвета
     */
    private double getServoPosition(DetectedColor color) {
        switch (color) {
            case GREEN:
                return SERVO_POSITION_GREEN;
            case PURPLE:
                return SERVO_POSITION_PURPLE;
            case UNKNOWN:
            default:
                return SERVO_POSITION_RED;
        }
    }
}
