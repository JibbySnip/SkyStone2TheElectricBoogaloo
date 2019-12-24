package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GyroSensor;

@Autonomous(name="ToastAuto")
public class toastAuto extends LinearOpMode {
    private GyroSensor heading;

    @Override
    public void runOpMode() throws InterruptedException{

        mecanumDrive drive = new mecanumDrive(
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),heading);
        Intake intake = new Intake(hardwareMap.get(DcMotorEx.class, "intakeR"),hardwareMap.get(DcMotorEx.class, "intakeL"));

    }

}
