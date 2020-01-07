package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import Jama.Matrix;

class Odometry {
    private double angle,track_width,wheel_base;
    private MecanumDrive dt;
    private DcMotorEx fl,fr,bl,br;
    private Position position;
    private MatchTimer time;



    Odometry(MecanumDrive dt, double start_x, double start_y, double start_angle,MatchTimer time){
        this.dt = dt;
        fl = dt.fL;
        br = dt.bR;
        fr = dt.fR;
        bl = dt.bL;

        angle = start_angle;
        track_width = dt.track_width;
        wheel_base = dt.wheel_base;
        position = dt.position;

    }
    Odometry(MecanumDrive dt, double start_x, double start_y){
        this.dt = dt;
        fl = dt.fL;
        br = dt.bR;
        fr = dt.fR;
        bl = dt.bL;
        position = dt.position;
        angle = 0;
    }
    double[] getRobotVelocity(){
        double[] wheel_velocities = dt.getVelocities();
        double track_base_val = 1/(track_width + wheel_base);

        Matrix coeffs = new Matrix(new double[][]{
                {1,1,1,1},
                {-1,1,-1,1},
                {-track_base_val,-track_base_val,track_base_val,track_base_val}});
        Matrix wheel_vels = new Matrix(new double[][]{
                {dt.radiansToMeters(wheel_velocities[1])},
                {dt.radiansToMeters(wheel_velocities[3])},
                {dt.radiansToMeters(wheel_velocities[2])},
                {dt.radiansToMeters(wheel_velocities[0])}});

        Matrix velocities = coeffs.times(wheel_vels).times(dt.wheel_diameter/4);
        return new double[]{velocities.get(0,0),velocities.get(0,1),velocities.get(0,2)};
    }
}
