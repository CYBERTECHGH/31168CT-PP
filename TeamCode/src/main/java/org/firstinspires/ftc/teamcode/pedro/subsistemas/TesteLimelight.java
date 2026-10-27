package org.firstinspires.ftc.teamcode.pedro.subsistemas;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Disabled
@TeleOp (name = "LimeLightTest")
public class TesteLimelight extends LinearOpMode {

    private Limelight3A limelight;
    private Follower follower;

    private String lastTagLabel = "Nenhuma";
    private double lastTx       = 0.0;
    private int    tagCount     = 0;

    private boolean guide = true; // Ativador do alinhamento automático

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Inicializa o Follower V3 e define a pose inicial
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Inicializado e pronto");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Atualiza a odometria e posição do PedroPathing
            follower.update();

            LLResult result = limelight.getLatestResult();

            // 2. Processa o resultado da visão se for válido
            if (result != null && result.isValid()) {
                processAprilTagResults(result);
            } else {
                tagCount = 0;
            }

            // 3. Exibe telemetria
            displayTelemetry(result);
            telemetry.update();
        }
    }

    private void processAprilTagResults(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        tagCount = fiducials.size();

        // Variáveis para guardar as tags específicas, caso sejam encontradas
        LLResultTypes.FiducialResult tag38 = null;
        LLResultTypes.FiducialResult tag41 = null;
        LLResultTypes.FiducialResult tag42 = null;
        LLResultTypes.FiducialResult tag45 = null;

        // Identifica as tags visíveis neste frame
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
            // Se vir o par (38 e 41), calcula o ponto médio entre as duas
            erroHorizontal = (tag38.getTargetXDegrees() + tag41.getTargetXDegrees()) / 2.0;
            lastTagLabel = "Par 38 & 41";
            alvoEncontrado = true;

        } else if (tag42 != null && tag45 != null) {
            // Se vir o par (42 e 45), calcula o ponto médio entre as duas
            erroHorizontal = (tag42.getTargetXDegrees() + tag45.getTargetXDegrees()) / 2.0;
            lastTagLabel = "Par 42 & 45";
            alvoEncontrado = true;
        }

        // Se encontrou algum PAR válido, realiza a rotação
        if (alvoEncontrado) {
            lastTx = erroHorizontal;
            alignRobotToCenter(erroHorizontal);
        } else {
            // Se não ver os pares completos, não faz nada
            if (tagCount > 0) {
                lastTagLabel = "Tags visíveis, mas nenhum par completo";
            } else {
                lastTagLabel = "Nenhuma tag alvo";
            }
        }
    }

    private void alignRobotToCenter(double erroX) {
        if (!guide) return;

        // Margem de tolerância (deadzone de 1.0°) para evitar oscilação/trepidação
        if (Math.abs(erroX) > 1.0) {

            // Obtém a pose atual do robô
            Pose currentPose = follower.pose();

            //rotação no proprio eixo
            double deltaRadianos = Math.toRadians(erroX);
            double novoHeading = currentPose.heading() - deltaRadianos;

            // corrige apenas o angulo
            follower.hold(new Pose(currentPose.x(), currentPose.y(), novoHeading));
        }
    }

    private void displayTelemetry(LLResult result) {
        telemetry.setMsTransmissionInterval(11);

        LLStatus status = limelight.getStatus();
        telemetry.addLine("=== LIMELIGHT 3A ===");
        telemetry.addData("Pipeline ativa", result != null ? result.getPipelineIndex() : "---");
        telemetry.addData("FPS", status != null ? String.format("%.1f", status.getFps()) : "---");

        telemetry.addLine("");
        telemetry.addLine("=== APRILTAGS BIO ===");

        if (result != null && result.isValid() && tagCount > 0) {
            telemetry.addData("🟢 Tags visíveis na tela", tagCount);
            telemetry.addData("Status do Alvo", lastTagLabel);

            // s exibe o erro se um par completo estiver em foco
            if (lastTagLabel.contains("Par")) {
                telemetry.addData("Erro para o centro (Tx)", String.format("%.2f°", lastTx));
            }

            telemetry.addLine("");
            telemetry.addLine("=== POSIÇÃO DO ROBÔ ===");
            Pose robotPose = follower.pose();
            telemetry.addData("X", String.format("%.2f", robotPose.x()));
            telemetry.addData("Y", String.format("%.2f", robotPose.y()));
            telemetry.addData("Heading", String.format("%.2f°", Math.toDegrees(robotPose.heading())));
        } else {
            telemetry.addLine("🔴 Nenhuma AprilTag detectada");
        }
    }
}