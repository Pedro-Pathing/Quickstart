package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Объединенный TeleOp: BilordaHouston
 * Включает:
 * - Драйв, интейк, шутеры (из BilordaHouston)
 * - Управление турелью с HuskyLens Object Tracking PD контроллером (из tureltest)
 * - Распознавание цветов и управление RGB светодиодами (из ColorSensorRGBTeleOp)
 */
@TeleOp(name="BilordaHouston", group="FTC")
public class BilordaHouston extends OpMode {

    /* ================= HARDWARE - DRIVE & MECHANISMS ================= */
    
    private DcMotor left1, left2, right1, right2;
    private DcMotor intake;
    private DcMotorEx shut1, shut2;
    private Servo pod;

    /* ================= HARDWARE - TURRET ================= */
    
    private HuskyLens huskyLens;
    private DcMotorEx turret;

    /* ================= HARDWARE - COLOR SENSORS & RGB ================= */
    
    // Цветовые датчики
    private NormalizedColorSensor c1, c2, c3, c4;
    
    // RGB светодиоды GoBilda
    private Servo rgb1, rgb2, rgb3;

    /* ================= DRIVE TUNING ================= */
    
    // ===== ULTRA RESPONSIVE DRIVE TUNING =====
    // минимальный deadzone, чтобы реагировал на малейшее касание
    private static final double DRIVE_DEADZONE = 0.02;     // 0.02..0.03
    // чтобы мотор реально "трогался" сразу после deadzone
    private static final double MIN_OUT = 0.10;     // 0.08..0.14
    // манёвренность (усиление поворота)
    private static final double TURN_GAIN = 1.20;    // 1.1..1.4

    /* ================= TURRET CAMERA ================= */
    
    private static final double CENTER_X = 160.0;
    private static final int READ_PERIOD = 50; // миллисекунды между чтениями
    private Deadline rateLimit;

    /* ================= TURRET TUNING ================= */
    
    private static final double KP = 0.003;
    private static final double MAX_POWER = 0.4;
    private static final int TARGET_ID = 1;  // ID объекта для отслеживания

    /* ================= TURRET PD STATE ================= */
    
    private ElapsedTime loopTimer = new ElapsedTime();

    /* ================= SHOOTER STATE ================= */
    
    String last;
    private double liftPower = 1.0;
    private boolean lastYButtonState = false;
    private boolean lastXButtonState = false;
    private boolean farShooterActive = false;   // Дальний шутер (кнопка Y)
    private boolean midShooterActive = false;  // Средний шутер (кнопка X)

    /* ================= SHOOTER PID ================= */
    
    // Дальний шутер (кнопка Y на gamepad2)
    private static final double FAR_SHOOTER_P = 53;
    private static final double FAR_SHOOTER_F = 6;
    private static final double FAR_SHOOTER_VELOCITY = 1450;
    
    // Средний шутер (кнопка X на gamepad2)
    private static final double MID_SHOOTER_P = 27;
    private static final double MID_SHOOTER_F = 4;
    private static final double MID_SHOOTER_VELOCITY = 1150;

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
    // Если цвета не работают, попробуйте изменить эти значения
    // Альтернативные значения для зеленого: 0.15, 0.2, 0.25, 0.3, 0.4, 0.5
    // Альтернативные значения для красного: 0.05, 0.15, 0.2, 0.3, 0.5, 1.0
    private static final double SERVO_POSITION_GREEN = 0.25;   // Позиция для зеленого цвета (изменено с 0.2 на 0.25)
    private static final double SERVO_POSITION_PURPLE = 0.66;  // Позиция для фиолетового цвета
    private static final double SERVO_POSITION_RED = 0.1;      // Позиция для красного цвета
    private static final double SERVO_POSITION_OFF = 0.0;       // Позиция для выключенного состояния

    @Override
    public void init() {
        last = "";

        /* ---------- DRIVE & MECHANISMS INIT ---------- */
        
        left1 = hardwareMap.get(DcMotor.class, "left1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right1 = hardwareMap.get(DcMotor.class, "right2");
        right2 = hardwareMap.get(DcMotor.class, "right1");

        // ===== DRIVE: моментальный стоп (важно) =====
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");

        pod = hardwareMap.get(Servo.class, "pod");

        // Настройка моторов дальнего шута с PIDF
        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Настройка направления
        shut1.setDirection(DcMotorSimple.Direction.FORWARD);
        shut2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Применение PIDF коэффициентов к обоим моторам (по умолчанию выключено)
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(MID_SHOOTER_P, 0, 0, MID_SHOOTER_F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        /* ---------- TURRET INIT ---------- */
        
        try {
            huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
            turret = hardwareMap.get(DcMotorEx.class, "turret");

            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPower(0);

            if (!huskyLens.knock()) {
                telemetry.addLine("⚠️ HuskyLens NOT connected");
            } else {
                huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
                telemetry.addLine("✅ HuskyLens ready (Object Tracking, ID=" + TARGET_ID + ")");
            }
            
            // Инициализация rate limiter
            rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);
            rateLimit.expire();
        } catch (Exception e) {
            telemetry.addLine("⚠️ Turret/HuskyLens init error: " + e.getMessage());
        }

        /* ---------- COLOR SENSORS & RGB INIT ---------- */
        
        try {
            // Инициализация цветовых датчиков
            c1 = hardwareMap.get(NormalizedColorSensor.class, "c1");
            c2 = hardwareMap.get(NormalizedColorSensor.class, "c2");
            c3 = hardwareMap.get(NormalizedColorSensor.class, "c3");
            c4 = hardwareMap.get(NormalizedColorSensor.class, "c4");

            // Установка усиления для датчиков
            if (c1 != null) c1.setGain(8);
            if (c2 != null) c2.setGain(8);
            if (c3 != null) c3.setGain(8);
            if (c4 != null) c4.setGain(8);

            // Инициализация RGB светодиодов
            rgb1 = hardwareMap.get(Servo.class, "rgb1");
            rgb2 = hardwareMap.get(Servo.class, "rgb2");
            rgb3 = hardwareMap.get(Servo.class, "rgb3");

            // Выключить все светодиоды при старте
            setRGBColor(rgb1, SERVO_POSITION_OFF);
            setRGBColor(rgb2, SERVO_POSITION_OFF);
            setRGBColor(rgb3, SERVO_POSITION_OFF);

            telemetry.addLine("✅ Color sensors & RGB ready (c1, c2, c3, c4)");
        } catch (Exception e) {
            telemetry.addLine("⚠️ Color sensors/RGB init error: " + e.getMessage());
        }

        telemetry.addLine("✅ BilordaHouston готов");
        telemetry.update();
    }

    @Override
    public void loop() {
        
        /* ===================== ULTRA RESPONSIVE MECANUM DRIVE (только колёса) ===================== */
        // твоя инверсия (оставляю 1 в 1)
        double y = -gamepad1.right_stick_x;
        double x = -gamepad1.left_stick_y;
        double r = gamepad1.left_stick_x;

        // маленький deadzone
        y = applyDeadzone(y, DRIVE_DEADZONE);
        x = applyDeadzone(x, DRIVE_DEADZONE);
        r = applyDeadzone(r, DRIVE_DEADZONE);

        // "сразу трогаться" даже от микрокасания
        y = applyMinOutput(y, MIN_OUT);
        x = applyMinOutput(x, MIN_OUT);
        r = applyMinOutput(r, MIN_OUT);

        // манёвренность
        r *= TURN_GAIN;

        // hard stop
        if (y == 0 && x == 0 && r == 0) {
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);
        } else {
            double lf = y + x + r;
            double lb = y - x + r;
            double rf = y - x - r;
            double rb = y + x - r;

            double max = Math.max(1.0,
                    Math.max(Math.abs(lf),
                    Math.max(Math.abs(lb),
                    Math.max(Math.abs(rf), Math.abs(rb)))));

            left1.setPower(lf / max);
            left2.setPower(lb / max);
            right1.setPower(rf / max);
            right2.setPower(rb / max);
        }
        // =========================================================================================

        /* ---------- INTAKE CONTROL ---------- */
        
        // Управление intake на gamepad1 и gamepad2
        if (gamepad1.left_trigger > 0 || gamepad2.right_bumper) {
            intake.setPower(-0.5);
        } else if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
            intake.setPower(0.5);
        } else {
            intake.setPower(0.0);
        }

        /* ---------- SHOOTER CONTROL ---------- */
        
        // Toggle-логика для дальнего шутера (кнопка Y на gamepad2)
        boolean currentYButtonState = gamepad2.y;
        if (currentYButtonState && !lastYButtonState) {
            farShooterActive = !farShooterActive;
            if (farShooterActive) {
                // Выключаем средний шутер при включении дальнего
                midShooterActive = false;
                // Применяем PID параметры для дальнего шутера
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            }
        }
        lastYButtonState = currentYButtonState;

        // Toggle-логика для среднего шутера (кнопка X на gamepad2)
        boolean currentXButtonState = gamepad2.x;
        if (currentXButtonState && !lastXButtonState) {
            midShooterActive = !midShooterActive;
            if (midShooterActive) {
                // Выключаем дальний шутер при включении среднего
                farShooterActive = false;
                // Применяем PID параметры для среднего шутера
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(MID_SHOOTER_P, 0, 0, MID_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            }
        }
        lastXButtonState = currentXButtonState;

        // Управление моторами шутеров
        if (farShooterActive) {
            // Дальний шутер активен
            shut1.setVelocity(FAR_SHOOTER_VELOCITY);
            shut2.setVelocity(-FAR_SHOOTER_VELOCITY);
        } else if (midShooterActive) {
            // Средний шутер активен
            shut1.setVelocity(MID_SHOOTER_VELOCITY);
            shut2.setVelocity(-MID_SHOOTER_VELOCITY);
        } else {
            // Оба шутера выключены
            shut1.setVelocity(0);
            shut2.setVelocity(0);
        }

        // тут был данат
        /* ---------- POD CONTROL ---------- */
        
        if (gamepad2.a) {
            pod.setPosition(0.7);
        } else {
            pod.setPosition(0.5);
        }

       //тут был данат
        /* ---------- TURRET CONTROL ---------- */
        
        if (huskyLens != null && turret != null && rateLimit != null) {
            if (!rateLimit.hasExpired()) {
                // Пропускаем чтение, если не прошло достаточно времени
            } else {
                rateLimit.reset();
                
                if (huskyLens.knock()) {
                    HuskyLens.Block[] blocks = huskyLens.blocks();
                    
                    if (blocks.length > 0) {
                        HuskyLens.Block block = blocks[0];  // первый трек
                        if (block.id == TARGET_ID) {
                            double errorX = block.x - CENTER_X;
                            double power = -errorX * KP;
                            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
                            
                            turret.setPower(power);
                        } else {
                            turret.setPower(0);
                        }
                    } else {
                        turret.setPower(0);
                    }
                }
            }
        }

        /* ---------- COLOR SENSORS & RGB CONTROL ---------- */
        
        if (c1 != null && c2 != null && c3 != null && c4 != null) {
            // Определение цветов для каждого датчика
            DetectedColor color1 = detectColor(c1, "Sensor 1");
            DetectedColor color2 = detectColor(c2, "Sensor 2");
            DetectedColor color3 = detectColor(c3, "Sensor 3");
            DetectedColor color4 = detectColor(c4, "Sensor 4");

            // Управление RGB светодиодами в зависимости от распознанного цвета
            // rgb1 зависит от c1, rgb2 зависит от c2, rgb3 зависит от c3 И c4
            updateRGBLED(rgb1, color1);
            updateRGBLED(rgb2, color2);
            
            // RGB3 управляется обоими датчиками c3 и c4
            DetectedColor color3Combined = combineColors(color3, color4);
            updateRGBLED(rgb3, color3Combined);
        }

        /* ---------- TELEMETRY ---------- */
        
        telemetry.addLine("=== Drive ===");
        telemetry.addData("Far Shooter (Y)", farShooterActive);
        telemetry.addData("Mid Shooter (X)", midShooterActive);
        telemetry.addLine();
        
        if (turret != null && huskyLens != null) {
            telemetry.addLine("=== Turret ===");
            telemetry.addData("Target ID", TARGET_ID);
            if (huskyLens.knock()) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("Блоков", blocks.length);
                if (blocks.length > 0) {
                    HuskyLens.Block block = blocks[0];
                    if (block.id == TARGET_ID) {
                        double errorX = block.x - CENTER_X;
                        telemetry.addData("ID", block.id);
                        telemetry.addData("X", block.x);
                        telemetry.addData("Ошибка", "%.0f", errorX);
                        telemetry.addData("Power", "%.2f", turret.getPower());
                    } else {
                        telemetry.addData(">>", "Не ID " + TARGET_ID + ", а " + block.id);
                    }
                } else {
                    telemetry.addData(">>", "Объект потерян");
                }
            }
            telemetry.addLine();
        }
        
        if (c1 != null && c2 != null && c3 != null && c4 != null) {
            DetectedColor color1 = detectColor(c1, "Sensor 1");
            DetectedColor color2 = detectColor(c2, "Sensor 2");
            DetectedColor color3 = detectColor(c3, "Sensor 3");
            DetectedColor color4 = detectColor(c4, "Sensor 4");
            DetectedColor color3Combined = combineColors(color3, color4);
            
            telemetry.addLine("=== Color Detection ===");
            telemetry.addData("Sensor 1 (c1)", color1);
            telemetry.addData("Sensor 2 (c2)", color2);
            telemetry.addData("Sensor 3 (c3)", color3);
            telemetry.addData("Sensor 4 (c4)", color4);
            
            // Отладочная информация для первого датчика
            if (c1 != null) {
                NormalizedRGBA colors = c1.getNormalizedColors();
                float total = colors.red + colors.green + colors.blue;
                if (total > 0) {
                    telemetry.addLine();
                    telemetry.addLine("=== Sensor 1 RGB Values ===");
                    telemetry.addData("R", "%.3f", colors.red);
                    telemetry.addData("G", "%.3f", colors.green);
                    telemetry.addData("B", "%.3f", colors.blue);
                    telemetry.addData("Total", "%.3f", total);
                    if (total > 0) {
                        telemetry.addData("R ratio", "%.3f", colors.red / total);
                        telemetry.addData("G ratio", "%.3f", colors.green / total);
                        telemetry.addData("B ratio", "%.3f", colors.blue / total);
                    }
                }
            }
            telemetry.addLine();
            telemetry.addLine("=== RGB LEDs ===");
            telemetry.addData("RGB1 (c1)", getColorName(color1));
            telemetry.addData("RGB2 (c2)", getColorName(color2));
            telemetry.addData("RGB3 (c3+c4)", getColorName(color3Combined));
            telemetry.addLine();
            telemetry.addLine("=== RGB Positions ===");
            telemetry.addData("RGB1 pos", "%.3f", getServoPosition(color1));
            telemetry.addData("RGB2 pos", "%.3f", getServoPosition(color2));
            telemetry.addData("RGB3 pos", "%.3f", getServoPosition(color3Combined));
            if (rgb1 != null) telemetry.addData("RGB1 actual", "%.3f", rgb1.getPosition());
            if (rgb2 != null) telemetry.addData("RGB2 actual", "%.3f", rgb2.getPosition());
            if (rgb3 != null) telemetry.addData("RGB3 actual", "%.3f", rgb3.getPosition());
        }
        
        telemetry.update();
    }

    @Override
    public void stop() {
        // Выключить все светодиоды при остановке
        setRGBColor(rgb1, SERVO_POSITION_OFF);
        setRGBColor(rgb2, SERVO_POSITION_OFF);
        setRGBColor(rgb3, SERVO_POSITION_OFF);
        
        // Остановить турель
        if (turret != null) {
            turret.setPower(0);
        }
    }


    /* ================= COLOR DETECTION ================= */
    
    /**
     * Определяет цвет по данным цветового датчика
     */
    private DetectedColor detectColor(NormalizedColorSensor sensor, String sensorName) {
        if (sensor == null) {
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

        // Логика определения ЗЕЛЕНОГО цвета
        // Используем >= вместо == для учета погрешностей округления
        boolean isGreen = (normGreen >= maxComponent - 0.01f) && 
                         (greenRatio > GREEN_RATIO_THRESHOLD) &&
                         (normGreen > normRed * 1.2) && 
                         (normGreen > normBlue * 1.2) &&
                         (normGreen > 0.3);

        // Логика определения ФИОЛЕТОВОГО цвета
        boolean isPurple = (normBlue > 0.2) &&
                          (normRed > 0.15) &&
                          (normBlue >= normGreen) &&
                          (normBlue >= normRed * 0.6) &&
                          ((normRed + normBlue) > normGreen * 1.2) &&
                          (blueRatio > BLUE_RATIO_THRESHOLD);

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
     * Комбинирует цвета от двух датчиков для RGB3
     * Приоритет: Зеленый > Фиолетовый > Красный (Unknown)
     * Если любой из датчиков видит зеленый, результат зеленый
     * Если любой из датчиков видит фиолетовый (и нет зеленого), результат фиолетовый
     * Если оба видят неизвестный, результат неизвестный (красный)
     */
    private DetectedColor combineColors(DetectedColor color1, DetectedColor color2) {
        // Если любой из датчиков видит зеленый, результат зеленый
        if (color1 == DetectedColor.GREEN || color2 == DetectedColor.GREEN) {
            return DetectedColor.GREEN;
        }
        
        // Если любой из датчиков видит фиолетовый, результат фиолетовый
        if (color1 == DetectedColor.PURPLE || color2 == DetectedColor.PURPLE) {
            return DetectedColor.PURPLE;
        }
        
        // Если оба видят неизвестный, результат неизвестный (красный)
        return DetectedColor.UNKNOWN;
    }
    
    /**
     * Обновляет цвет RGB светодиода в зависимости от распознанного цвета
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
            telemetry.addLine("❌ Ошибка установки RGB позиции: " + e.getMessage());
        }
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
    
    /* ================= DRIVE UTILS ================= */
    
    private double applyDeadzone(double v, double dz) {
        return (Math.abs(v) < dz) ? 0 : v;
    }

    private double applyMinOutput(double v, double minOut) {
        if (v == 0) return 0;
        double sign = Math.signum(v);
        double a = Math.abs(v); // 0..1
        return sign * (minOut + (1.0 - minOut) * a);
    }
}
