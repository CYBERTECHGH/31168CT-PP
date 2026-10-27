package org.firstinspires.ftc.teamcode.pedro.TeleOp.Teste;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "Teste Motor", group = "MotorTeste")
public class TesteMotor extends LinearOpMode {
    // MOTORES (porta 0 a 3)
    // control hub (Hub A)
    private final String CONFIG_MOTOR_0A = "rightBack";
    private final String CONFIG_MOTOR_1A = "rightFront";
    private final String CONFIG_MOTOR_2A = "intake";
    private final String CONFIG_MOTOR_3A = "decontrol";

    // expansion hub (Hub B)
    private final String CONFIG_MOTOR_0B = "leftBack";
    private final String CONFIG_MOTOR_1B = "leftFront";
    private final String CONFIG_MOTOR_2B = "shooter";
    private final String CONFIG_MOTOR_3B = "contrarolo";


    // SERVOS (porta 0 a 5)
    // control hub (Hub A)
    private final String CONFIG_SERVO_0A = "none";
    private final String CONFIG_SERVO_1A = "none";
    private final String CONFIG_SERVO_2A = "none";
    private final String CONFIG_SERVO_3A = "none";
    private final String CONFIG_SERVO_4A = "none";
    private final String CONFIG_SERVO_5A = "none";
    // Expansion Hub (Hub B)
    private final String CONFIG_SERVO_0B = "servoCapuz";
    private final String CONFIG_SERVO_1B = "none";
    private final String CONFIG_SERVO_2B = "none";
    private final String CONFIG_SERVO_3B = "none";
    private final String CONFIG_SERVO_4B = "none";
    private final String CONFIG_SERVO_5B = "none";



    // ESTRUTURA
    private static class MotorSlot {
        String id, configName;
        DcMotorEx motor = null;
        boolean isConnected = false, isActive = false;
        double power = 0.0;
        DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

        MotorSlot(String id, String configName) {
            this.id = id; this.configName = configName;
        }
    }

    private static class ServoSlot {
        String id, configName;
        Servo servo = null;
        boolean isConnected = false, isActive = false;
        double position = 0.5; // Inicia no meio
        Servo.Direction direction = Servo.Direction.FORWARD;

        ServoSlot(String id, String configName) {
            this.id = id; this.configName = configName;
        }
    }

    private final List<MotorSlot> motorList = new ArrayList<>();
    private final List<ServoSlot> servoList = new ArrayList<>();

    // controle de aba e Cursore
    private enum AbaAtiva { MOTORES, SERVOS }
    private AbaAtiva abaAtual = AbaAtiva.MOTORES;

    private int motorCursor = 0;
    private int servoCursor = 0;

    // controle de debounce dos botões
    private boolean lastA = false, lastY = false;
    private boolean lastDpadUp = false, lastDpadDown = false;
    private boolean lastDpadLeft = false, lastDpadRight = false;

    @Override
    public void runOpMode() {
        inicializarListas();
        telemetry.addLine("=== INICIALIZANDO HARDWARE ===");

        // mapeia os motores
        for (MotorSlot slot : motorList) {
            if (slot.configName.equals("none")) continue;
            try {
                slot.motor = hardwareMap.get(DcMotorEx.class, slot.configName);
                slot.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slot.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slot.isConnected = true;
            } catch (Exception e) { slot.isConnected = false; }
        }

        // mapeia os servos
        for (ServoSlot slot : servoList) {
            if (slot.configName.equals("none")) continue;
            try {
                slot.servo = hardwareMap.get(Servo.class, slot.configName);
                slot.isConnected = true;
            } catch (Exception e) {
                slot.isConnected = false;
                telemetry.addData("ERRO", "Servo " + slot.configName + " falhou: " + e.getMessage());
            }
        }
        telemetry.update();

        telemetry.addLine("\nHardwares mapeados!");
        telemetry.addLine("Pressione PLAY para iniciar os testes.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            processarEntradas();
            atualizarHardware();
            exibirTelemetria();
        }
    }

    private void inicializarListas() {
        motorList.add(new MotorSlot("Motor 0a", CONFIG_MOTOR_0A));
        motorList.add(new MotorSlot("Motor 1a", CONFIG_MOTOR_1A));
        motorList.add(new MotorSlot("Motor 2a", CONFIG_MOTOR_2A));
        motorList.add(new MotorSlot("Motor 3a", CONFIG_MOTOR_3A));
        motorList.add(new MotorSlot("Motor 0b", CONFIG_MOTOR_0B));
        motorList.add(new MotorSlot("Motor 1b", CONFIG_MOTOR_1B));
        motorList.add(new MotorSlot("Motor 2b", CONFIG_MOTOR_2B));
        motorList.add(new MotorSlot("Motor 3b", CONFIG_MOTOR_3B));

        servoList.add(new ServoSlot("Servo 0a", CONFIG_SERVO_0A));
        servoList.add(new ServoSlot("Servo 1a", CONFIG_SERVO_1A));
        servoList.add(new ServoSlot("Servo 2a", CONFIG_SERVO_2A));
        servoList.add(new ServoSlot("Servo 3a", CONFIG_SERVO_3A));
        servoList.add(new ServoSlot("Servo 4a", CONFIG_SERVO_4A));
        servoList.add(new ServoSlot("Servo 5a", CONFIG_SERVO_5A));
        servoList.add(new ServoSlot("Servo 0b", CONFIG_SERVO_0B));
        servoList.add(new ServoSlot("Servo 1b", CONFIG_SERVO_1B));
        servoList.add(new ServoSlot("Servo 2b", CONFIG_SERVO_2B));
        servoList.add(new ServoSlot("Servo 3b", CONFIG_SERVO_3B));
        servoList.add(new ServoSlot("Servo 4b", CONFIG_SERVO_4B));
        servoList.add(new ServoSlot("Servo 5b", CONFIG_SERVO_5B));
    }

    private void processarEntradas() {
        boolean btnA = gamepad1.a;
        boolean btnY = gamepad1.y;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        // alternar entre as abas (motores <-> servos)
        if (dpadLeft && !lastDpadLeft) abaAtual = AbaAtiva.MOTORES;
        if (dpadRight && !lastDpadRight) abaAtual = AbaAtiva.SERVOS;

        // lógica baseada na aba selecionada
        if (abaAtual == AbaAtiva.MOTORES) {
            MotorSlot currentMotor = motorList.get(motorCursor);

            // navegação
            if (dpadUp && !lastDpadUp) { motorCursor = (motorCursor - 1 + motorList.size()) % motorList.size(); }
            if (dpadDown && !lastDpadDown) { motorCursor = (motorCursor + 1) % motorList.size(); }

            // ligar/desliga
            if (btnA && !lastA) currentMotor.isActive = !currentMotor.isActive;

            // inverte a direção
            if (btnY && !lastY) {
                for (MotorSlot slot : motorList) {
                    if (slot.isActive) {
                        slot.direction = (slot.direction == DcMotorSimple.Direction.FORWARD) ?
                                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
                    }
                }
            }
        }
        else if (abaAtual == AbaAtiva.SERVOS) {
            ServoSlot currentServo = servoList.get(servoCursor);

            // navegação
            if (dpadUp && !lastDpadUp) { servoCursor = (servoCursor - 1 + servoList.size()) % servoList.size(); }
            if (dpadDown && !lastDpadDown) { servoCursor = (servoCursor + 1) % servoList.size(); }

            // liga/desliga
            if (btnA && !lastA) currentServo.isActive = !currentServo.isActive;

            // inverte a direção
            if (btnY && !lastY) {
                for (ServoSlot slot : servoList) {
                    if (slot.isActive) {
                        slot.direction = (slot.direction == Servo.Direction.FORWARD) ?
                                Servo.Direction.REVERSE : Servo.Direction.FORWARD;
                    }
                }
            }
        }

        // atualização de borda dos botões
        lastA = btnA;
        lastY = btnY;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
    }

    private void atualizarHardware() {
        double inputY = -gamepad1.left_stick_y; // invertido para cima ser positivo

        // 1. atualiza os motores
        double motorPower = (Math.abs(inputY) < 0.05) ? 0.0 : inputY;
        for (MotorSlot slot : motorList) {
            if (slot.isConnected && slot.motor != null) {
                if (slot.isActive) {
                    slot.power = motorPower;
                    slot.motor.setDirection(slot.direction);
                    slot.motor.setPower(slot.power);
                } else {
                    slot.power = 0.0;
                    slot.motor.setPower(0.0);
                }
            }
        }

        // atualizaa servos (o analógico soma ou subtrai da posição atual
        double servoIncremento = 0.0;
        if (Math.abs(inputY) > 0.05) {
            servoIncremento = inputY * 0.015; // velocidade de movimento do servo
        }

        for (ServoSlot slot : servoList) {
            if (slot.isConnected && slot.servo != null) {
                if (slot.isActive) {
                    slot.position += servoIncremento;
                    // Limita a posição do servo entre 0.0 e 1.0
                    slot.position = Math.max(0.0, Math.min(1.0, slot.position));

                    slot.servo.setDirection(slot.direction);
                    slot.servo.setPosition(slot.position);
                }
            }
        }
    }

    private void exibirTelemetria() {
        telemetry.addLine("===== PAINEL DE TESTE =====");

        // Cabeçalho dinâmico indicando a aba
        if (abaAtual == AbaAtiva.MOTORES) {
            telemetry.addLine("► [ MOTORES ]       SERVOS  ");
        } else {
            telemetry.addLine("    MOTORES       ► [ SERVOS ]");
        }
        telemetry.addLine("--------------------------------------");

        if (abaAtual == AbaAtiva.MOTORES) {
            for (int i = 0; i < motorList.size(); i++) {
                MotorSlot slot = motorList.get(i);
                if (slot.configName.equals("none")) continue; // pula os vazios

                String cursor = (i == motorCursor) ? "► " : "  ";
                if (slot.isConnected) {
                    String status = slot.isActive ? "🟢 LIGADO" : "🔴 DESLIG.";
                    telemetry.addLine(String.format("%s%s (%s) %s", cursor, slot.id, slot.configName, status));
                    telemetry.addLine(String.format("   Pot: %.2f | Dir: %s | Enc: %d",
                            slot.power, slot.direction.toString(), slot.motor.getCurrentPosition()));
                } else {
                    telemetry.addLine(String.format("%s%s (%s) [NÃO CONECTADO]", cursor, slot.id, slot.configName));
                }
            }
        }
        else if (abaAtual == AbaAtiva.SERVOS) {
            for (int i = 0; i < servoList.size(); i++) {
                ServoSlot slot = servoList.get(i);
                if (slot.configName.equals("none")) continue; // pula os vazios

                String cursor = (i == servoCursor) ? "► " : "  ";
                if (slot.isConnected) {
                    String status = slot.isActive ? "🟢 LIGADO" : "🔴 DESLIG.";
                    telemetry.addLine(String.format("%s%s (%s) %s", cursor, slot.id, slot.configName, status));
                    telemetry.addLine(String.format("   Pos: %.3f | Dir: %s",
                            slot.position, slot.direction.toString()));
                } else {
                    telemetry.addLine(String.format("%s%s (%s) [NÃO CONECTADO]", cursor, slot.id, slot.configName));
                }
            }
        }

        telemetry.addLine("--------------------------------------");
        telemetry.addLine("  CONTROLES:");
        telemetry.addLine("• D-Pad Esq/Dir: Mudar entre Motores e Servos");
        telemetry.addLine("• D-Pad Cima/Baixo: Navegar na lista");
        telemetry.addLine("• Botão A: Ligar / Desligar controle do item");
        telemetry.addLine("• Botão Y: Inverter Direção");
        telemetry.addLine("• Analógico Esquerdo (Cima/Baixo):");

        telemetry.update();
    }
}
