package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
//ooga booga
public class field extends OpMode
{

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;


    BNO055IMU imu;

    Orientation angles;

    BNO055IMU.Parameters parameters = null;

    enum ImuInitState { NOT_STARTED, STARTING, INIT_STARTED, INIT_SUCCESS, INIT_ERROR, RETRY_ON_ERROR, INTEGRATION_STARTED };

    ImuInitState imuInitState = ImuInitState.NOT_STARTED;

    @Override
    public void init() {
        //my variables stuff
        front_left   = hardwareMap.get(DcMotor.class, "FL");
        front_right  = hardwareMap.get(DcMotor.class, "FR");
        back_left    = hardwareMap.get(DcMotor.class, "BL");
        back_right   = hardwareMap.get(DcMotor.class, "BR");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(parameters);
    }

    @Override
    public void loop() {

        double drive  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float pi = 3.1415926f;

        float gyro_degrees = angles.firstAngle;
        float gyro_radians = gyro_degrees * pi / 180;


        double newdrive = drive * Math.cos(gyro_radians) + strafe * Math.sin(gyro_radians);
        double newstrafe = drive * Math.sin(gyro_radians) - strafe * Math.cos(gyro_radians);

        double[] speeds = {
                (newdrive - newstrafe - twist),
                (newdrive + newstrafe + twist),
                (newdrive + newstrafe - twist),
                (newdrive - newstrafe + twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) )
            {
                max = Math.abs(speeds[i]);
            }
        }

        // If the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1)
        {
            for (int i = 0; i < speeds.length; i++)
            {
                speeds[i] /= max;
            }
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }
}
