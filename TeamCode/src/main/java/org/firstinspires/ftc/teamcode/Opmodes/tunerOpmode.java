package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Elevator;

@Config
public class tunerOpmode extends LinearOpMode {
    Elevator elevator;
    DcMotor left = hardwareMap.get(DcMotor.class,"leftLift"); //left and right are when viewed from the back
    DcMotor right = hardwareMap.get(DcMotor.class, "rightLift");
    public static double p,i,d,f;
    private PIDFCoefficients pidf;
    private double spoolDiameter;
    private double ticksPerRev;
    private double maxExtension;
    private boolean isLeftReversed;
    private boolean isRightReversed;
    private int tolerance = 15;


    @Override
    public void runOpMode(){
        elevator = new Elevator(left,right,spoolDiameter,ticksPerRev,maxExtension,tolerance
        )
    }
}
