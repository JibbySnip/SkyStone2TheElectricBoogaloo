package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.openftc.revextensions2.ExpansionHubEx;

class MecanumDrive {
    DcMotorEx fL,bL,fR,bR;
    private BNO055IMU heading;
    private final static double DRIVE_MULT = 0;
    double track_width,wheel_base;
    Position position;
    private MatchTimer time;
    double wheel_diameter = 0.1;


    MecanumDrive(DcMotorEx fr, DcMotorEx fl, DcMotorEx br, DcMotorEx bl, BNO055IMU heading, MatchTimer time) {
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

    void initPosition(double x,double y){
       position = new Position(DistanceUnit.METER,x,y,0,time.getMatchTime());
    };

    double[] getSpeed() {
        return new double[]{fR.getPower(), fL.getPower(), bR.getPower(), bL.getPower()};
    }
    double[] getVelocities() {
        return new double[]{
            fR.getVelocity(AngleUnit.RADIANS), fL.getVelocity(AngleUnit.RADIANS),bR.getVelocity(AngleUnit.RADIANS),bL.getVelocity(AngleUnit.RADIANS)};
    }
    double radiansToMeters(double rps) {
        return rps*wheel_diameter;
    }

}
