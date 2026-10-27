package org.firstinspires.ftc.teamcode.pedro.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsistemas.Contrarolo;
import org.firstinspires.ftc.teamcode.pedro.subsistemas.Shooter;

import java.util.List;

@Autonomous(name = "Auto Azul Park")
public class AutoAzulPark extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Limelight3A limelight;
    private Shooter shooter;
    private Contrarolo contrarolo;
    private DcMotor intake;
    private DcMotor decontrol;
    private final ElapsedTime tempoAuto = new ElapsedTime();
    private boolean estacionando = false;
    private final double TEMPO_PARK_S = 30;
    private final double VELOCIDADE_ALVO = 1200;
    private final double CONTRAROLO_VELL = 800;

    private final Pose startPose = poseFactory.of(109, 6, 90);
    private final Pose shootPose = poseFactory.of(103, 12, 115.5);
    private final Pose parkPose = poseFactory.of(132, 24, 180);

    private Path score() {
        return Paths.line(startPose, shootPose).linear(startPose, shootPose);
    }

    // lançamento automático
    private Command autoShoot() {
        return sequential(
                instant(() -> {
                    shooter.atirar(VELOCIDADE_ALVO);
                    contrarolo.atirar(CONTRAROLO_VELL);
                }),
                waitMs(700),
                instant(() -> {
                    intake.setPower(-1);
                    decontrol.setPower(0.4);
                })
        );
    }

    // desliga tudo
    private Command pararSistemas() {
        return instant(() -> {
            shooter.parar();
            contrarolo.parar();
            intake.setPower(0);
            decontrol.setPower(0);
        });
    }

    // forçar parada
    private Command park() {
        return sequential(
                pararSistemas(),
                follow(follower, Paths.line(follower.pose(), parkPose).linear(follower.pose(), parkPose))
        );
    }

    // verificação das apriltags
    private boolean viuTags() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = f.getFiducialId();
                if (id >= 38 && id <= 41) {
                    return true; // encontrou pelo menos uma tag entre 42 e 45
                }
            }
        }
        return false;
    }

    // ROTA
    private Command autoRoutine() {
        return sequential(
                // vai até a posição de lançamento
                follow(follower, score()),

                // fica parado até que a Limelight veja uma tag
                waitUntil(() -> viuTags()),
                waitMs(150),

                // qnd a tag for detetada, executa o tiro e para os sistemas
                autoShoot(),
                waitMs(1500),
                pararSistemas(),

                park()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        shooter = new Shooter(hardwareMap);
        contrarolo = new Contrarolo(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        intake = hardwareMap.get(DcMotor.class, "intake");
        decontrol = hardwareMap.get(DcMotor.class, "decontrol");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        decontrol.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Inicializado - Pronto");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        tempoAuto.reset();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            if (!estacionando && tempoAuto.seconds() >= TEMPO_PARK_S) {
                estacionando = true;
                Scheduler.reset();
                schedule(park());
            }

            follower.update();
            Scheduler.execute();

            telemetry.addData("Tempo", "%.1f s", tempoAuto.seconds());
            telemetry.addData("Tags 42-45 Vistas?", viuTags());
            telemetry.addData("Estacionando?", estacionando);
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("X Atual", follower.pose().x());
            telemetry.addData("Y Atual", follower.pose().y());
            telemetry.update();
        }
    }
}