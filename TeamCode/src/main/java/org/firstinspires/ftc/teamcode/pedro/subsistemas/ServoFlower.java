package org.firstinspires.ftc.teamcode.pedro.subsistemas;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoFlower {
    private CRServo latchEsq;
    private CRServo latchDir;
    private ElapsedTime timer = new ElapsedTime();

    public enum EstadoTrava {
        TRAVADO,
        DESTRAVADO,
        MOVENDO_PARA_TRAVAR,
        MOVENDO_PARA_DESTRAVAR
    }

    private EstadoTrava estadoAtual = EstadoTrava.TRAVADO; // o robô inicia travado
    private double duracaoMovimento = 0.6; // Tempo em segundos para a rotação de abrir e fechar

    private static final double POWER_TRAVAR = 0.7;
    private static final double POWER_DESTRAVAR = -1.0;

    //potencia pra após ele travar
    private static final double POWER_MANTER_TRAVADO = 0.0;

    public void init(HardwareMap hardwareMap) {
        latchEsq = hardwareMap.get(CRServo.class, "Servo1");
        latchDir = hardwareMap.get(CRServo.class, "Servo2");

        // Espelhamento dos servos
        latchEsq.setDirection(DcMotorSimple.Direction.FORWARD);
        latchDir.setDirection(DcMotorSimple.Direction.REVERSE);

        parar();
    }

    public void update() {
        // se estava se movendo para travar e o tempo acabou, trava e aplica força pra manter travada
        if (estadoAtual == EstadoTrava.MOVENDO_PARA_TRAVAR && timer.seconds() >= duracaoMovimento) {
            estadoAtual = EstadoTrava.TRAVADO;
            latchEsq.setPower(POWER_MANTER_TRAVADO);
            latchDir.setPower(POWER_MANTER_TRAVADO);
        }
        // se estava se movendo para destravar e o tempo acabou, fica solto em segurança
        else if (estadoAtual == EstadoTrava.MOVENDO_PARA_DESTRAVAR && timer.seconds() >= duracaoMovimento) {
            estadoAtual = EstadoTrava.DESTRAVADO;
            parar();
        }
    }

    // adicionei pra deixar mais limpo no código
    public void alternar() {
        if (estadoAtual == EstadoTrava.TRAVADO || estadoAtual == EstadoTrava.MOVENDO_PARA_TRAVAR) {
            destravar();
        } else {
            travar();
        }
    }

    public void travar() {
        // so executa se já não estiver travando ou travado
        if (estadoAtual != EstadoTrava.TRAVADO && estadoAtual != EstadoTrava.MOVENDO_PARA_TRAVAR) {
            estadoAtual = EstadoTrava.MOVENDO_PARA_TRAVAR;
            latchEsq.setPower(POWER_TRAVAR);
            latchDir.setPower(POWER_TRAVAR);
            timer.reset();
        }
    }

    public void destravar() {
        // so executa se já não estiver destravando ou destravado
        if (estadoAtual != EstadoTrava.DESTRAVADO && estadoAtual != EstadoTrava.MOVENDO_PARA_DESTRAVAR) {
            estadoAtual = EstadoTrava.MOVENDO_PARA_DESTRAVAR;
            latchEsq.setPower(POWER_DESTRAVAR);
            latchDir.setPower(POWER_DESTRAVAR);
            timer.reset();
        }
    }

    public void parar() {
        latchEsq.setPower(0.0);
        latchDir.setPower(0.0);
    }

    public boolean isTravado() {
        return estadoAtual == EstadoTrava.TRAVADO;
    }
}