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
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsistemas.Contrarolo;
import org.firstinspires.ftc.teamcode.pedro.subsistemas.Shooter;

import java.util.List;

@Autonomous(name = "Auto Azul Garden")
public class AutoAzulGardem extends LinearOpMode {

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
    private final long TIMEOUT_GARDEN_MS = 4000;
    private final double VELOCIDADE_ALVO = 1300;
    private final double CONTRAROLO_VELL = 800;

    // POSES
    private final Pose startPose = poseFactory.of(73.0, 129.0, 270);
    private final Pose shootPose = poseFactory.of(77.8, 126.4, 270);
    private final Pose gardenPose = poseFactory.of(124, 115.5, 90);
    private final Pose wallPose = poseFactory.of(124, 125.5, 90);
    private final Pose travelPose1 = poseFactory.of(106.5, 90.5, 270);

    // posição de lançamento alternativa (se n ver as tags)
    private final Pose travelPose2 = poseFactory.of(72.8, 11, 270);
    private final Pose alternateShootPose = poseFactory.of(72.8, 5, 90);
    private final Pose finishPose = poseFactory.of(130, 24, 180);

    private Path score() {
        return Paths.line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path alignGarden() {
        return Paths.line(shootPose, gardenPose).linear(shootPose, gardenPose);
    }

    private Path gardenToWall() {
        return Paths.line(gardenPose, wallPose).linear(gardenPose, wallPose);
    }

    private Path wallToGardenBack() {
        return Paths.line(wallPose, gardenPose).linear(wallPose, gardenPose);
    }

    private Path returnShoot() {
        return Paths.line(gardenPose, shootPose).linear(gardenPose, shootPose);
    }

    private Path route(){
        return Paths.line(shootPose, travelPose1).linear(shootPose, travelPose2);
    }

    private Path AlternateShoot() {
        return Paths.line(shootPose, travelPose2).linear(shootPose, travelPose2);
    }

    private Path routeAlternate() {
        return Paths.line(travelPose2, alternateShootPose).linear(travelPose2, alternateShootPose);
    }

    private Path parkPose() {
        return Paths.line(alternateShootPose, finishPose).linear(alternateShootPose, finishPose);
    }


    // lançamento automático
    private Command autoShoot() {
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

    // desligar tudo
    private Command pararSistemas() {
        return instant(() -> {
            shooter.parar();
            contrarolo.parar();
            intake.setPower(0);
            decontrol.setPower(0);
        });
    }

    // verificação das apriltags
    private boolean viuTags() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = f.getFiducialId();
                if (id >= 42 && id <= 45) {
                    return true; // encontrou pelo menos uma tag entre 41 e 44
                }
            }
        }
        return false; // não encontrou nenhuma tag
    }

    // força parada
    private Command park() {
        return sequential(
                pararSistemas(),
                follow(follower, Paths.line(follower.pose(), finishPose).linear(follower.pose(), finishPose))
        );
    }

    // ROTA
    private Command autoRoutine() {
        return sequential(
                // vai até a posição de lançamento inicial e atira
                follow(follower, score()),
                waitMs(500),
                autoShoot(),
                waitMs(1500),
                pararSistemas(),

                follow(follower, alignGarden()),
                // liga o intake, vai até a parede, volta coletando
                instant(() -> {
                    intake.setPower(-1.0);
                    decontrol.setPower(-0.1);
                }),

                // vai ao garden. se passar de 4s, desiste
                race(
                        follow(follower, gardenToWall()),
                        waitMs(TIMEOUT_GARDEN_MS)
                ),
                waitMs(300),
                follow(follower, wallToGardenBack()),

                // desliga tudo após sair do gardem
                follow(follower, returnShoot()),
                pararSistemas(),
                waitMs(1000),

                // condição para saber em qual hive vai lançar
                conditional(
                        () -> viuTags(),
                        // viu as tags, atira
                        sequential(
                                autoShoot(),
                                waitMs(1500),
                                pararSistemas(),
                                follow(follower, route())
                        ),
                        // não viu as tags, atravessa pelo centro e atira no outro hive
                        sequential(
                                follow(follower, AlternateShoot()),
                                follow(follower, routeAlternate()),
                                waitMs(1000),
                                autoShoot(),
                                waitMs(1500),
                                pararSistemas()
                        )
                ),
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
            telemetry.addData("Estacionando?", estacionando);
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("Tags Vistas?", viuTags());
            telemetry.addData("X Atual", follower.pose().x());
            telemetry.addData("Y Atual", follower.pose().y());
            telemetry.update();
        }
    }
}