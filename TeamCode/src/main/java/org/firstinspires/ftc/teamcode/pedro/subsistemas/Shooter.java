package org.firstinspires.ftc.teamcode.pedro.subsistemas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    private final DcMotorEx motorShooter;
    public static double kP = 54;
    public static double kI =0;
    public static double kD = 10;
    public static double kF = 14;

    public Shooter(HardwareMap hardwareMap) {
        motorShooter = hardwareMap.get(DcMotorEx.class, "shooter");

        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void atirar(double ticksPorSegundo) {
        motorShooter.setVelocity(ticksPorSegundo);
    }

    public void parar() {
        motorShooter.setVelocity(0);
    }

    public double getVelocity() {
        return motorShooter.getVelocity();
    }
}