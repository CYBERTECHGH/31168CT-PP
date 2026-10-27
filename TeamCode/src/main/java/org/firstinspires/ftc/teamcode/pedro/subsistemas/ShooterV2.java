package org.firstinspires.ftc.teamcode.pedro.subsistemas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterV2 {
    private DcMotorEx motor;


    public static double kP = 0.002;
    public static double kI = 0.00001;
    public static double kD = 0.0001;
    public static double kF = 0.0003;

    private double targetVelocity = 1200;
    private double integralSum = 0;
    private double lastError = 0;

    private ElapsedTime timer;

    public ShooterV2(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer = new ElapsedTime();
    }

    public void setTargetVelocity(double ticksPerSecond) {
        this.targetVelocity = ticksPerSecond;
        this.integralSum = 0;
        this.timer.reset();
    }


    public void stop() {
        this.targetVelocity = 0;
    }

    public void update() {
        if (targetVelocity == 0) {
            motor.setPower(0);
            integralSum = 0;
            return;
        }

        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        double dt = timer.seconds();

        if (dt == 0) return;

        double p = kP * error;

        integralSum += error * dt;
        double i = kI * integralSum;

        double derivative = (error - lastError) / dt;
        double d = kD * derivative;

        double f = kF * targetVelocity;

        double power = p + i + d + f;

        power = Math.max(-1.0, Math.min(1.0, power));

        motor.setPower(power);

        lastError = error;
        timer.reset();
    }

    public double getCurrentVelocity() {
        return motor.getVelocity();
    }
}