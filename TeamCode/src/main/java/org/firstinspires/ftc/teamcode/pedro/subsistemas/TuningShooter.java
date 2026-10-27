package org.firstinspires.ftc.teamcode.pedro.subsistemas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Disabled
@TeleOp(name = "Shooter PIDF Tuning")
public class TuningShooter extends LinearOpMode {
    private DcMotorEx shooter;

    // Coeficientes ajustáveis via controle
    private double kF = 0;
    private double kP = 0;
    private double kI = 0;
    private  double kD = 0;

    private int SHOOTER_TICKS_POR_SEGUNDO = 1200;

    // Antirebound para os botões do D-Pad
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadUp2 = false;
    private boolean lastDpadDown2 = false;
    private boolean lastDpadRight2 = false;
    private boolean lastDpadLeft2 = false;

    @Override
    public void runOpMode() {
        // Envia os dados de telemetria para o Driver Hub E para o gráfico do Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Aplica os coeficientes iniciais
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));

        waitForStart();

        while (opModeIsActive()) {
            boolean mudou = false;

            // D-Pad Cima/Baixo: Ajusta kF em passos de 0.5
            if (gamepad1.dpad_up && !lastDpadUp) { kP += 1; mudou = true; }
            if (gamepad1.dpad_down && !lastDpadDown) { kP -= 1; mudou = true; }

            // D-Pad Direita/Esquerda: Ajusta kP em passos de 0.1
            if (gamepad1.dpad_right && !lastDpadRight) { kF += 1; mudou = true; }
            if (gamepad1.dpad_left && !lastDpadLeft) { kF -= 1; mudou = true; }

            if (gamepad2.dpad_up && !lastDpadUp2) { kD += 0.1; mudou = true; }
            if (gamepad2.dpad_down && !lastDpadDown2) { kD -= 0.1; mudou = true; }

            // D-Pad Direita/Esquerda: Ajusta kP em passos de 0.1
            if (gamepad2.dpad_right && !lastDpadRight2) { kI += 1; mudou = true; }
            if (gamepad2.dpad_left && !lastDpadLeft2) { kI -= 1; mudou = true; }

            // Atualiza estados dos botões
            lastDpadUp2 = gamepad2.dpad_up;
            lastDpadDown2 = gamepad2.dpad_down;
            lastDpadRight2 = gamepad2.dpad_right;
            lastDpadLeft2 = gamepad2.dpad_left;

            // Atualiza estados dos botões
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadRight = gamepad1.dpad_right;
            lastDpadLeft = gamepad1.dpad_left;

            // Só reconfigura o motor se o piloto pressionou algum botão
            if (mudou) {
                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(kP, kI, kD, kF));
            }

            // Segure RT no Gamepad para acelerar o motor
            if (gamepad1.right_trigger > 0.1) {
                shooter.setVelocity(SHOOTER_TICKS_POR_SEGUNDO);
            } else {
                shooter.setVelocity(0);
            }

            // Variáveis plottadas no Gráfico do Dashboard
            telemetry.addData("Velocidade Alvo", SHOOTER_TICKS_POR_SEGUNDO);
            telemetry.addData("Velocidade Atual", shooter.getVelocity());
            telemetry.addData("kP (D-Pad Cima/Baixo)", kP);
            telemetry.addData("kF (D-Pad Dir/Esq)", kF);
            telemetry.addData("kI (D-Pad Cima/Baixo)", kI);
            telemetry.addData("kD (D-Pad Dir/Esq)", kD);
            telemetry.update();
        }
    }
}