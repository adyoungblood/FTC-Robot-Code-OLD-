
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

//******************************************************************************
//------------------------------------------------------------------------------
//Main routine for the Operating Mode

public class RobotG01V01 extends OpMode {

//*** Variable Definitions

// Initialization of motors:

    // * Tread motors:
    public DcMotorController motor_drive_right;
    public DcMotorController motor_drive_left;
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;

    // * Arm motors:
    public DcMotorController motor_controller_arm_1;
    public DcMotorController motor_controller_winch;
    public DcMotor motor_arm_1;
    public DcMotor motor_arm_2;
    public DcMotor motor_winch_1;
    public DcMotor motor_winch_2;

// Initialization of servos:

    public ServoController servo_controller_1;
    public double top_servo_position;
    public Servo top_servo;

/*    public double arm_servo2_position;
    public double arm_servo3_position;
    public double arm_servo4_position;
   public double arm_extension_position;

    public Servo arm_servo1;
    public Servo arm_servo2;
    public Servo arm_servo3;
    public Servo arm_servo4;
    public Servo arm_extension_servo;

*/
// Initilization of drive train variables:
    //public double power_forward;
    public double power_back;
    public double power_RT;
    public double power_LT;

// Initialization of joystick buttons:
    public double button_RT;
    public double button_LT;

    public boolean button_RB;
    public boolean button_LB;
    public boolean button_a;
    public boolean button_y;
    public boolean button_b;
    public boolean button_x;

    public double joystick_right_x;
    public double joystick_right_y;
    public double joystick_left_x;
    public double joystick_left_y;

    public double arm1_power;
    public double arm2_power;


    public double power_level;
//******************************************************************************
//------------------------------------------------------------------------------
//Routine for Operating Mode (no code)
    public RobotG01V01()

    {

    }

//******************************************************************************
//------------------------------------------------------------------------------
//Routine for Arm Control:

    public void arm_motor_control()
    {

        joystick_right_y = gamepad2.right_stick_y;
        joystick_left_y = gamepad2.left_stick_y;

        if(joystick_right_y > 0.2 || joystick_right_y < -0.2)
        {
            motor_arm_2.setPower(-joystick_right_y*0.15);
        }
        else
        {
            motor_arm_2.setPower(0);
        }

        if(joystick_left_y > 0.2 || joystick_left_y < -0.2)
        {
            motor_arm_1.setPower(-joystick_left_y*0.15);
        }
        else
        {
            motor_arm_1.setPower(0);
        }

        if (gamepad2.start)
        {
            motor_arm_1.setPower(0);
            motor_arm_2.setPower(0);
        }


    }

    //******************************************************************************
//------------------------------------------------------------------------------
//Routine for Winch Control:

    public void winch_motor_control()
    {


        if(gamepad2.right_trigger > 0)
        {
            motor_winch_1.setPower(-gamepad2.right_trigger);
            motor_winch_2.setPower(gamepad2.right_trigger);
        }

        if(gamepad2.left_trigger > 0)
        {
            motor_winch_1.setPower(gamepad2.left_trigger);
            motor_winch_2.setPower(-gamepad2.left_trigger);
        }

        if(gamepad2.left_trigger ==0 && gamepad2.right_trigger == 0) {
            motor_winch_2.setPower(0);
            motor_winch_1.setPower(0);
        }
    }



    //******************************************************************************
//------------------------------------------------------------------------------
//Routine for Wheel Servo Control:


    public void arm_servo_move()
    {

        if (gamepad2.dpad_left) {
            top_servo_position = top_servo_position - 0.005;
            top_servo_position = Range.clip(top_servo_position, 0.001, 1);
        }
        if (gamepad2.dpad_right) {
            top_servo_position = top_servo_position + 0.005;
            top_servo_position = Range.clip(top_servo_position, 0.001, 1);
        }


        top_servo.setPosition(top_servo_position);


/*        joystick_left_y = gamepad2.left_stick_y;


        if(joystick_left_y > 0.2)
        {

            arm_servo1_position = arm_servo1_position + 0.001;
            arm_servo2_position = arm_servo2_position + 0.001;
            arm_servo3_position = arm_servo3_position - 0.001;
            arm_servo4_position = arm_servo4_position - 0.001;

            arm_servo1_position = Range.clip(arm_servo1_position, 0, 0.17);
            arm_servo2_position = Range.clip(arm_servo2_position, 0, 0.17);
            arm_servo3_position = Range.clip(arm_servo3_position, 0.83, 1);
            arm_servo4_position = Range.clip(arm_servo4_position, 0.83, 1);
        }

        if(joystick_left_y <  -0.2)
        {

            arm_servo1_position = arm_servo1_position - 0.001;
            arm_servo2_position = arm_servo2_position - 0.001;
            arm_servo3_position = arm_servo3_position + 0.001;
            arm_servo4_position = arm_servo4_position + 0.001;

            arm_servo1_position = Range.clip(arm_servo1_position, 0, 0.17);
            arm_servo2_position = Range.clip(arm_servo2_position, 0, 0.17);
            arm_servo3_position = Range.clip(arm_servo3_position, 0.83, 1);
            arm_servo4_position = Range.clip(arm_servo3_position, 0.83, 1);
        }

            servo_controller_1.setServoPosition(1, arm_servo1_position);
            servo_controller_1.setServoPosition(2, arm_servo2_position);
            servo_controller_1.setServoPosition(3, arm_servo3_position);
            servo_controller_1.setServoPosition(4, arm_servo4_position);


*/

/*        if (gamepad2.left_bumper)
        {
            arm_servo1_position = arm_servo1_position - 0.001;
            arm_servo1_position = Range.clip(arm_servo1_position, 0, 1);
            servo_controller_1.setServoPosition(1, arm_servo1_position);

            arm_servo2_position = arm_servo2_position - 0.001;
            arm_servo2_position = Range.clip(arm_servo2_position, 0, 1);
            servo_controller_1.setServoPosition(2, arm_servo2_position);
            //wheel_servo.setPosition(servo1_position);

            arm_servo3_position = arm_servo3_position + 0.001;
            arm_servo3_position = Range.clip(arm_servo3_position, 0, 1);
            servo_controller_1.setServoPosition(3, arm_servo3_position);
            //wheel_servo.setPosition(servo1_position);

            arm_servo4_position = arm_servo4_position + 0.001;
            arm_servo4_position = Range.clip(arm_servo4_position, 0, 1);
            servo_controller_1.setServoPosition(4, arm_servo4_position);
            //wheel_servo.setPosition(servo1_position);


        }

        if (gamepad2.right_bumper)
        {
            arm_servo1_position = arm_servo1_position + 0.001;
            arm_servo1_position = Range.clip(arm_servo1_position, 0, 1);
            servo_controller_1.setServoPosition(1, arm_servo1_position);

            arm_servo2_position = arm_servo2_position + 0.001;
            arm_servo2_position = Range.clip(arm_servo2_position, 0, 1);
            servo_controller_1.setServoPosition(2, arm_servo2_position);
            //wheel_servo.setPosition(servo1_position);

            arm_servo3_position = arm_servo3_position - 0.001;
            arm_servo3_position = Range.clip(arm_servo3_position, 0, 1);
            servo_controller_1.setServoPosition(3, arm_servo3_position);
            //wheel_servo.setPosition(servo1_position);

            arm_servo4_position = arm_servo4_position - 0.001;
            arm_servo4_position = Range.clip(arm_servo4_position, 0, 1);
            servo_controller_1.setServoPosition(4, arm_servo4_position);
            //wheel_servo.setPosition(servo1_position);

        }


        //wheel_servo.close();

*/
    }



//******************************************************************************
//------------------------------------------------------------------------------
//Routine for Driving Robot with Front Buttons:

    public void drive_with_buttons() {

        // Define mode in which power is controlled:

        button_RT = gamepad1.right_trigger * power_level;
        power_back = 0.5 * power_level;
        button_RB = gamepad1.right_bumper;

        if (button_RT > 0) {
            motor_drive_right.setMotorPower(1, -button_RT);
            motor_drive_right.setMotorPower(2, -button_RT);

        } else if (button_RB) {
            motor_drive_right.setMotorPower(1, power_back);
            motor_drive_right.setMotorPower(2, power_back);
        } else {
            motor_drive_right.setMotorPowerFloat(1);
            motor_drive_right.setMotorPowerFloat(2);
        }

        button_LT = gamepad1.left_trigger * power_level;
        button_LB = gamepad1.left_bumper;

        if (button_LT > 0) {
            motor_drive_left.setMotorPower(1, button_LT);
            motor_drive_left.setMotorPower(2, button_LT);
        } else if (button_LB) {
            motor_drive_left.setMotorPower(1, -power_back);
            motor_drive_left.setMotorPower(2, -power_back);
        } else {
            motor_drive_left.setMotorPowerFloat(1);
            motor_drive_left.setMotorPowerFloat(2);
        }

        if (gamepad1.b) {
            motor_drive_right.setMotorPower(1, 0);
            motor_drive_right.setMotorPower(2, 0);
            motor_drive_left.setMotorPower(1, 0);
            motor_drive_left.setMotorPower(2, 0);
        }
    }

//******************************************************************************
//------------------------------------------------------------------------------
//Initialization routine for the Operating Mode
//Initialize hardware devices and global variables:

    @Override
    public void init()

    {
        //Initialize hardware components:
        motor_drive_left = hardwareMap.dcMotorController.get("Motor_Controller_Right");
        motor_drive_right = hardwareMap.dcMotorController.get("Motor_Controller_Left");

        motor1 = hardwareMap.dcMotor.get("Motor1");
        motor2 = hardwareMap.dcMotor.get("Motor2");
        motor3 = hardwareMap.dcMotor.get("Motor3");
        motor4 = hardwareMap.dcMotor.get("Motor4");

        motor_controller_arm_1 =  hardwareMap.dcMotorController.get("Motor_Controller_Arm_1");
        motor_controller_winch =  hardwareMap.dcMotorController.get("Motor_Controller_Winch");
        motor_arm_1 = hardwareMap.dcMotor.get("MotorArm1");
        motor_arm_2 = hardwareMap.dcMotor.get("MotorArm2");
        motor_winch_1 = hardwareMap.dcMotor.get("MotorWinch1");
        motor_winch_2 = hardwareMap.dcMotor.get("MotorWinch2");

     /*   motor_controller_intake =  hardwareMap.dcMotorController.get("Motor_Controller_Intake");
        motor_bottom = hardwareMap.dcMotor.get("MotorBottom");
        motor_conveyor = hardwareMap.dcMotor.get("MotorConveyor");
*/
        //Set power level for backing up (between 0 and 1):
        power_back = 0.5;
        arm1_power = 0;
        arm2_power = 0;
        power_level = 0.3;

        motor_winch_1.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motor_winch_2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        motor_arm_1.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motor_arm_2.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        motor_drive_right.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motor_drive_right.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motor_drive_left.setMotorChannelMode(1, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motor_drive_left.setMotorChannelMode(2, DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        //Initialize servos:
        servo_controller_1 = hardwareMap.servoController.get("ServoController1");
        top_servo = hardwareMap.servo.get("TopServo");
        top_servo_position = 0.5;

/*        servo_controller_1 = hardwareMap.servoController.get("ServoController1");
        arm_servo1 = hardwareMap.servo.get("ArmServo1");
        arm_servo2 = hardwareMap.servo.get("ArmServo2");
        arm_servo3 = hardwareMap.servo.get("ArmServo3");
        arm_servo4 = hardwareMap.servo.get("ArmServo4");
        arm_extension_servo = hardwareMap.servo.get("ArmExtension");
*/
       /* arm_servo1_position = 0.02; //servo1.getPosition();
        arm_servo2_position = 0.02; //servo1.getPosition();
        arm_servo3_position = 0.98;
        arm_servo4_position = 0.98; */

/*       arm_servo1_posit ion = 0; //servo1.getPosition();
        arm_servo2_position = 0; //servo1.getPosition();
        arm_servo3_position = 1;
        arm_servo4_position = 1;
        arm_extension_position = 0;

        servo_controller_1.setServoPosition(1, arm_servo1_position);
        servo_controller_1.setServoPosition(2, arm_servo2_position);
        servo_controller_1.setServoPosition(3, arm_servo3_position);
        servo_controller_1.setServoPosition(4, arm_servo4_position);
        servo_controller_1.setServoPosition(5, arm_extension_position);

*/

    }

//******************************************************************************
//------------------------------------------------------------------------------
//Routine for the Start Mode (no code)

    @Override
    public void start()

    {
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        // This method is designed to be overridden.

    }

//******************************************************************************
//------------------------------------------------------------------------------
// Main loop routine
// This is executed repeatedly

    @Override
    public void loop()

    {

// Call driving routine:
        drive_with_buttons();

//Call arm control routine:
        arm_motor_control();

//Call winch control routine:
        winch_motor_control();

// Call wheel servo routine:
        arm_servo_move();



        if (gamepad1.start) {
            power_level = 1;
        }
        else {
            power_level = 0.3;
        }

    //    telemetry.addData("Text", "Robot is working");
        // Display telemetry data on the driver phone:
/*        telemetry.addData("Text", "***Telemetry Data for Robot***");
        telemetry.addData("Button_LT", "Left Power Level:" + String.format("%.2f", button_LT));
        telemetry.addData("Button_RT", "Right Power Level:" + String.format("%.2f", button_RT));
        telemetry.addData("arm1_power", "arm1_power:" + String.format("%.2f", arm1_power));
        telemetry.addData("arm2_power", "arm2_power:" + String.format("%.2f", arm2_power));
        telemetry.addData("joystick_left_x", "joystick_left_x:" +String.format("%.2f", joystick_left_x));
        telemetry.addData("joystick_left_y", "joystick_left_y:" +String.format("%.2f", joystick_left_y));
        telemetry.addData("joystick_right_x", "joystick_right_x:" +String.format("%.2f", joystick_right_x));
        telemetry.addData("joystick_right_y", "joystick_right_y:" +String.format("%.2f", joystick_right_y));
        telemetry.addData("armservo_1_position", "armservo_1_position:" +String.format("%.2f", arm_servo1.getPosition()));
        telemetry.addData("armservo_2_position", "armservo_2_position:" +String.format("%.2f", arm_servo2.getPosition()));
        telemetry.addData("armservo_3_position", "armservo_3_position:" +String.format("%.2f", arm_servo3.getPosition()));
        telemetry.addData("armservo_4_position", "armservo_4_position:" +String.format("%.2f", arm_servo4.getPosition()));
        telemetry.addData("armservo_box_opening_position", "armservo_box_opening_position:" +String.format("%.2f", servo_box_open_position));
        telemetry.addData("armservo_box_position", "armservo_box_position:" +String.format("%.2f", servo_box_move_position));
        telemetry.addData("power_level", "power_level:" +String.format("%.2f", power_level));
  */
    }
//******************************************************************************
//------------------------------------------------------------------------------
//Stop routine (no code)

    @Override
    public void stop() {

    }

}