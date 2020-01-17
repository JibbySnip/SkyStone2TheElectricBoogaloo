package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class tunerOpmode extends LinearOpMode {
    private Elevator elevator;
    private DcMotor left = hardwareMap.get(DcMotor.class,"leftLift"); //left and right are when viewed from the back
    private DcMotor right = hardwareMap.get(DcMotor.class, "rightLift");
    public static double p,i,d,f;
    private PIDFCoefficients pidf;
    private double spoolDiameter = 1.3;
    private double ticksPerRev = 145.6;
    private double maxExtension;
    private boolean isLeftReversed = true;
    private boolean isRightReversed = true;
    private int tolerance = 10;
    RevBlinkinLedDriver blinkin;

    @Override
    public void runOpMode(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        pidf = new PIDFCoefficients(p,i,d,f);
        elevator = new Elevator(left,right,pidf,spoolDiameter,ticksPerRev,maxExtension,isLeftReversed,isRightReversed,tolerance);
        elevator.setVelocity(1);
        while (elevator.getCurrentPos() <= 0.75*maxExtension) {
            elevator.update();
        }
        elevator.stop();
        elevator.setVelocity(-1);
        while (elevator.getCurrentPos() >= 0.15) {
            elevator.update();
        }
        elevator.stop();
    }
}
