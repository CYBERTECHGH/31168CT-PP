package org.firstinspires.ftc.teamcode.pedro.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

import org.firstinspires.ftc.teamcode.pedro.subsistemas.Contrarolo;
import org.firstinspires.ftc.teamcode.pedro.subsistemas.Shooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp(name = "TeleOp Azul - FDC", group = "TeleOp")
public class TeleOpAzul extends LinearOpMode {

    private Limelight3A limelight;
    private Shooter shooter;
    private Contrarolo contrarolo;
    private Follower follower;
    private DcMotorEx intake;
    private DcMotorEx decontrol;
    private double holdX = 0.0;
    private double holdY = 0.0;
    private String lastTagLabel = "Nenhuma";
    private double lastTx = 0.0;
    private final double VELOCIDADE_ALVO = 1300;
    private final double VELOCIDADE_SOLO = 1200;
    private final double CONTRAROLO_VELL = 800;
    private double headingOffset = 0.0;

    // CONFIGURAÇÃO DE ESTADO
    private int intakeState = 0;
    private boolean lancando = false;
    private boolean shooterLigado = false;
    private boolean autoAlignAtivo = false;
    private boolean soShooterLigado = false; // ADICIONADO
    // gamepads
    private Gamepad currG1 = new Gamepad();
    private Gamepad prevG1 = new Gamepad();
    private Gamepad currG2 = new Gamepad();
    private Gamepad prevG2 = new Gamepad();

    // LANÇAMENTO AUTOMÁTICO COM VERIFICAÇÃO DE TRAVAMENTO
    private Command AutoShoot() {
        return sequential(
                instant(() -> {
                    shooter.atirar(VELOCIDADE_ALVO);
                    contrarolo.atirar(CONTRAROLO_VELL);
                }),
                waitMs(800),
                instant(() -> {
                    intake.setPower(-1);
                    decontrol.setPower(0.4);
                })
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 170));

        shooter = new Shooter(hardwareMap);
        contrarolo = new Contrarolo(hardwareMap);
        inicializarHardware();

        telemetry.addLine("Hardware pronto.");
        telemetry.addLine("Pronto para o Start!");
        telemetry.update();

        waitForStart();

        // LOOP DO TELEOP
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            Scheduler.execute();
            atualizarControles();

            // ALTERNANCIA
            if (autoAlignAtivo) {
                controlarAlinhamentoLimelight();
            } else {
                controlarChassi();
            }

            controlarLançamento();
            controlarIntake();
            controlarShooter();
            controlarSoShooterToggle(); // ADICIONADO
            exibirTelemetria();
        }
    }

    // CONTROLES E LÓGICA

    private void atualizarControles() {
        prevG1.copy(currG1);
        currG1.copy(gamepad1);

        prevG2.copy(currG2);
        currG2.copy(gamepad2);

        // ALINHAMENTO AUTOMÁTICO DA LIMELIGHT
        if (currG1.a && !prevG1.a) {
            autoAlignAtivo = !autoAlignAtivo;

            if (autoAlignAtivo) {
                holdX = follower.pose().x();
                holdY = follower.pose().y();
            }
        }

        // SISTEMA DE REALIGN
        if (currG1.y && !prevG1.y) {
            headingOffset = follower.pose().heading();
        }
    }

    private void controlarChassi() {
        // subtrai o headingOffset para ajustar o referencial do Field-Centric
        double headingCorrigido = follower.pose().heading() - headingOffset;

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                headingCorrigido
        );
        ManualDrive.driveOrHold(follower, powers);
    }

    private void controlarAlinhamentoLimelight() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            LLResultTypes.FiducialResult tag38 = null, tag41 = null, tag42 = null, tag45 = null;

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 38) tag38 = fiducial;
                else if (id == 41) tag41 = fiducial;
                else if (id == 42) tag42 = fiducial;
                else if (id == 45) tag45 = fiducial;
            }


            double erroHorizontal = 0.0;
            boolean alvoEncontrado = false;

            if (tag38 != null && tag41 != null) {
                erroHorizontal = (tag38.getTargetXDegrees() + tag41.getTargetXDegrees()) / 2.0;
                lastTagLabel = "Par 38 & 41";
                alvoEncontrado = true;
            } else if (tag42 != null && tag45 != null) {
                erroHorizontal = (tag42.getTargetXDegrees() + tag45.getTargetXDegrees()) / 2.0;
                lastTagLabel = "Par 42 & 45";
                alvoEncontrado = true;
            }

            if (alvoEncontrado) {
                lastTx = erroHorizontal;

                if (Math.abs(erroHorizontal) > 0.7) {
                    double novoHeading = follower.pose().heading() - Math.toRadians(erroHorizontal);
                    follower.hold(new Pose(holdX, holdY, novoHeading));
                } else {
                    follower.hold(new Pose(holdX, holdY, follower.pose().heading()));
                }
            } else {
                lastTagLabel = "Procurando pares...";
                follower.hold(new Pose(holdX, holdY, follower.pose().heading()));
            }
        } else {
            lastTagLabel = "Nenhuma tag detectada";
            follower.hold(new Pose(holdX, holdY, follower.pose().heading()));
        }
    }

    private void controlarIntake() {
        if (lancando || shooterLigado) return;

        if (currG2.right_trigger > 0.5 && !(prevG2.right_trigger > 0.5)) {
            intakeState = (intakeState == 1) ? 0 : 1;
        }

        if (currG2.right_bumper && !prevG2.right_bumper) {
            intakeState = (intakeState == -1) ? 0 : -1;
        }

        if (currG2.left_bumper && !prevG2.left_bumper) {
            intakeState = (intakeState == -2) ? 0 : -2;
        }

        if (currG2.left_trigger > 0.5 && !(prevG2.left_trigger > 0.5)) {
            intakeState = (intakeState == 2) ? 0 : 2;
        }

        switch (intakeState) {
            case 1:
                intake.setPower(1.0);
                decontrol.setPower(-1.0);
                break;
            case -1:
                intake.setPower(-1.0);
                decontrol.setPower(0);
                break;
            case 2:
                intake.setPower(0.5);
                decontrol.setPower(-1.0);
                break;
            case -2:
                intake.setPower(-0.5);
                decontrol.setPower(0);
                break;
            default:
                intake.setPower(0.0);
                decontrol.setPower(0.0);
                break;
        }
    }

    private void controlarLançamento() {
        if (shooterLigado) return;
        lancando = currG2.x;

        if (lancando) {
            intake.setPower(-1);
            decontrol.setPower(0.4);
        }
    }


    private void controlarShooter() {
        if (currG2.a && !prevG2.a) {
            shooterLigado = !shooterLigado;
            if (shooterLigado) {
                schedule(AutoShoot());
            } else {
                shooter.parar();
                contrarolo.parar();
                intake.setPower(0);
                decontrol.setPower(0);
            }
        }
    }

    // ADICIONADO: Y (gamepad2) liga/desliga somente o shooter
    private void controlarSoShooterToggle() {
        if (shooterLigado) {          // modo do A tem prioridade
            soShooterLigado = false;
            return;
        }
        if (currG2.y && !prevG2.y) {
            soShooterLigado = !soShooterLigado;
            if (soShooterLigado) {
                shooter.atirar(VELOCIDADE_SOLO);
                contrarolo.atirar(CONTRAROLO_VELL);
            } else {
                shooter.parar();
                contrarolo.parar();
            }
        }
    }

    private void inicializarHardware() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        decontrol = hardwareMap.get(DcMotorEx.class, "decontrol");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        decontrol.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void exibirTelemetria() {
        telemetry.addLine("=== ESTADOS ===");
        if (autoAlignAtivo) {
            telemetry.addData("Modo", "🟢 MIRA AUTOMÁTICA");
            telemetry.addData("Alvo Atual", lastTagLabel);
            telemetry.addData("Erro TX", String.format("%.2f°", lastTx));
        } else {
            telemetry.addData("Modo", "🔵 CONTROLE MANUAL");
        }

        telemetry.addLine("Realign (Y)");

        telemetry.addLine();
        telemetry.addLine("=== SHOOTER===");
        telemetry.addData("ESTADO:", shooterLigado ? "LIGADO" : "DESLIGADO");
        telemetry.addData("Velocidade Atual", shooter.getVelocity());
        if (shooterLigado) {
            telemetry.addData("Velocidade Alvo", VELOCIDADE_ALVO);
            telemetry.addLine("");
            telemetry.addData("Vell Alvo", CONTRAROLO_VELL);
            telemetry.addData("Vell Atual", contrarolo.getVelocity());
        }

        telemetry.update();
    }
}