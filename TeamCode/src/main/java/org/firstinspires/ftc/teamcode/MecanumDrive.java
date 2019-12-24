package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class MecanumDrive {
    private DcMotorEx fL,bL,fR,bR;
    private GyroSensor heading;
    private final static double DRIVE_MULT = 0;

    MecanumDrive(DcMotorEx fr, DcMotorEx fl, DcMotorEx br, DcMotorEx bl, GyroSensor heading) {
        fL = fl;
        bL = bl;
        fR = fr;
        bR = br;


        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.heading = heading;

    }
    void driveByXY(double x, double y, double turn) {
        double mag = Math.hypot(x,y);
        double angle = Math.atan2(-y,x); //fr - , fl + , bl -, bl +
        double[] speed = {
                DRIVE_MULT * (mag * Math.sin(angle - Math.PI/4) - turn),
                DRIVE_MULT * (mag * Math.sin(angle + Math.PI/4) + turn),
                DRIVE_MULT * (mag * Math.sin(angle - Math.PI/4) + turn),
                DRIVE_MULT * (mag * Math.sin(angle + Math.PI/4) - turn)};
        fR.setPower(speed[0]);
        fL.setPower(speed[1]);
        bR.setPower(speed[2]);
        bL.setPower(speed[3]);
    }

    double[] getSpeed() {
        return new double[]{fR.getPower(), fL.getPower(), bR.getPower(), bL.getPower()};
    }
    double[] getVelocity() {
        return new double[]{
            fR.getVelocity(AngleUnit.RADIANS), fL.getVelocity(AngleUnit.RADIANS),bR.getVelocity(AngleUnit.RADIANS),bL.getVelocity(AngleUnit.RADIANS)};
    }
    DcMotorEx[] getMotors() {
        return new DcMotorEx[]{fR,fL,bR,bL};
    }




}
