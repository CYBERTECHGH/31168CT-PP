package org.firstinspires.ftc.teamcode.pedro.subsistemas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Contrarolo {
    private final DcMotorEx motorContrarolo;
    public static double kP = 35;
    public static double kI =0;
    public static double kD = 5;
    public static double kF = 11;

    public Contrarolo(HardwareMap hardwareMap) {
        motorContrarolo = hardwareMap.get(DcMotorEx.class, "contrarolo");
        motorContrarolo.setDirection(DcMotorSimple.Direction.REVERSE);


        motorContrarolo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorContrarolo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorContrarolo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void atirar(double ticksPorSegundo) {
        motorContrarolo.setVelocity(ticksPorSegundo);
    }

    public void parar() {
        motorContrarolo.setVelocity(0);
    }

    public double getVelocity() {
        return motorContrarolo.getVelocity();
    }
}