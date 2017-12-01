/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOpMode", group="Iterative")  // @Autonomous(...) is the other common choice
@Disabled
public class TeleOpMode extends OpMode
{
    //HUB 1
    // Port 0 = left_front_drive
    // Port 1 = right_front_drive
    // Port 2 = left_back_motor
    // Port 3 = right_back_motor

    //HUB 2
    //MOTORS
    //Port 0 = left_arm
    //Port 1 = right_arm
    //Port 2 = linear_acc
    //Port 3 = syringe_motor
    //SERVOS
    //Port 0 = jewel_servo
    //Port 1 = claw_servo
    //Port 2 = claw_flipper
    //Port 3 = claw_rotator
    //I2C BUS 0 = sensor_color_digital
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1Motor = null;
    private DcMotor right1Motor = null;
    private DcMotor left2Motor = null;
    private DcMotor right2Motor = null;
    private double left1MotorPower = 0.0;
    private double right1MotorPower = 0.0;
    private double left2MotorPower = 0.0;
    private double right2MotorPower = 0.0;

    private final double DRIVE_HIGH_POWER = 0.8;
    private final double DRIVE_LOW_POWER = 0.3;
    private double drive_power_factor = DRIVE_HIGH_POWER;

    private final double ELEVATOR_POWER = 0.5;
    private final double ROTATOR_POWER = 0.5;
    private final double FLIPPER_POWER = 0.5;
    private final double EXTENDER_POWER = 0.5;

    private Servo elevator1 = null;         //Arm elevator motor 1
    private Servo elevator2 = null;         //Arm elevator motor 2

    private final double elevator_tolerance = 0.0;
    private final double elevator_degrees_per_count = 0;
    private final double elevator_min_pos = 0;
    private final double elevator_max_pos = 0;
    private double elevator1_prev_pos = 0.0;
    private double elevator2_prev_pos = 0.0;

    private Servo extender = null;          //Linear extender
    private Servo rotator = null;           //Rotates to select grabber or cup
    private Servo flipper = null;           //Stows/extends grabber/cup


    @Override
    public void init() {
        init_drive();

        init_elevator();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        get_drive_input();
    }

    protected void init_elevator() {
        elevator1 = hardwareMap.servo.get("elevator1");
        elevator2 = hardwareMap.servo.get("elevator2");

        elevator1.setDirection(Servo.Direction.REVERSE);
        elevator2.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Elevators Initialized");
    }

    protected void init_drive(){
        left1Motor  = hardwareMap.dcMotor.get("left_front_drive");
        right1Motor = hardwareMap.dcMotor.get("right_front_drive");
        left2Motor = hardwareMap.dcMotor.get("left_back_motor");
        right2Motor = hardwareMap.dcMotor.get("right_back_motor");

        left1Motor.setDirection(DcMotor.Direction.REVERSE);
        right1Motor.setDirection(DcMotor.Direction.FORWARD);
        left2Motor.setDirection(DcMotor.Direction.REVERSE);
        right2Motor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Drive Initialized");
    }

    protected void get_elevator_input() {

    }

    protected void get_drive_input() {
        drive_power_factor = (gamepad1.left_bumper ? DRIVE_LOW_POWER : DRIVE_HIGH_POWER);

        // Left Stick X
        double x1 = Range.clip(gamepad1.left_stick_x, -1, 1) * drive_power_factor;
        // Right Stick X
        double x2 = Range.clip(gamepad1.right_stick_x, -1, 1) * drive_power_factor;
        // Left Stick Y
        double y1 = Range.clip(-gamepad1.left_stick_y, -1, 1) * drive_power_factor;
        // Right Stick Y
        double y2 = Range.clip(-gamepad1.right_stick_y, -1,1) * drive_power_factor;

        right1MotorPower = y1 - x2 - x1;
        right2MotorPower =  y1 - x2 + x1;
        left1MotorPower = y1 + x2 + x1;
        left2MotorPower = y1 + x2 - x1;

        left1Motor.setPower(left1MotorPower);
        right1Motor.setPower(right1MotorPower);
        left2Motor.setPower(left2MotorPower);
        right2Motor.setPower(right2MotorPower);
    }

    @Override
    public void stop() {
    }

}
