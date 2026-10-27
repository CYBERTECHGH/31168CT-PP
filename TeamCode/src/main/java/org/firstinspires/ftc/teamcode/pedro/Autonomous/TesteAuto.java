package org.firstinspires.ftc.teamcode.pedro.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.subsistemas.Shooter;

@Disabled
@Autonomous(name = "Auto Azul 1")
public class TesteAuto extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Shooter shooter;
    private DcMotor shooterMotor;
    private DcMotor intake;
    private DcMotor decontrol;

    private final double VELOCIDADE_ALVO = 1200;

    // POSES
    private final Pose startPose = poseFactory.of(133.0, 110.0, 180);
    private final Pose shootPose = poseFactory.of(83.0, 136.0, 270);
    private final Pose gardenPose = poseFactory.of(138.0, 130.0, 0);


    private Path startToScore() {
        return Paths.line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path scoreToGarden() {
        return Paths.line(shootPose, gardenPose).linear(shootPose, gardenPose);
    }

    private Command lancarUmPolen() {
        return sequential(
                // liga o sistema para lançar
                instant(() -> {
                    intake.setPower(-1.0);
                    decontrol.setPower(0.6);
                }),
                // tempo para subir um pólen
                waitMs(200),
                // Desliga o sistema
                instant(() -> {
                    intake.setPower(0.0);
                    decontrol.setPower(0.0);
                }),
                waitMs(500)
        );
    }

    // --- ROTA PRINCIPAL DO AUTÔNOMO ---
    private Command autoRoutine() {
        return sequential(
                // vai até a posição de lançamento
                follow(follower, startToScore()),

                // liga o Shooter
                instant(() -> shooter.atirar(VELOCIDADE_ALVO)),
                waitMs(1000),

                // lança 3 pólens pausadamente
                lancarUmPolen(),
                lancarUmPolen(),
                lancarUmPolen(),

                // desliga o shooter após
                instant(() -> shooterMotor.setPower(0.0)),

                // liga o intake para coletar
                instant(() -> {
                    intake.setPower(1.0);
                    decontrol.setPower(-0.3);
                }),

                follow(follower, scoreToGarden()),

                waitMs(500),
                instant(() -> {
                    intake.setPower(0.0);
                    decontrol.setPower(0.0);
                })
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // Mapeamento de Hardware
        shooter = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        decontrol = hardwareMap.get(DcMotor.class, "decontrol");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        decontrol.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Inicializado - Pronto para lançar e coletar");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("X Atual", follower.pose().x());
            telemetry.addData("Y Atual", follower.pose().y());
            telemetry.update();
        }
    }
}