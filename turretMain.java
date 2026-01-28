package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "turret")
public class turretMain extends OpMode {
    double Kp = 0.025;
    double Ki = 0.005;
    double Kd = 24.0;
    //tuned with maxPower = 0.4;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx turretMotor;
    Limelight3A limelight;

    double maxPower = 0.4;
    double deadZone = 0.5;

    double lastTx = 0;
    boolean hadTarget = false;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        timer.reset();
        telemetry.addLine("initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 0.001;

        double error;

        if (result != null && result.isValid()) {
            error = result.getTx() + 0.5;
            lastTx = error;
            hadTarget = true;
        } else {
            if (hadTarget) {
                error = lastTx;
            } else {
                error = 5.0; // positive = rotate right, negative = left
            }
            integralSum = 0;
        }
        if (Math.abs(error) < deadZone) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = error;
            telemetry.addLine("locked");
            return;
        }
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -10, 10);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        output = Range.clip(output, -maxPower, maxPower);
        turretMotor.setPower(output);

        telemetry.addData("error(tx)", error);
        telemetry.addData("output", output);
        telemetry.addData("hasTarget", result != null && result.isValid());
    }
}

