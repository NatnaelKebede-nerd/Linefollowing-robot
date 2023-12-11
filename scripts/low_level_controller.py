#user/bin/python

import rospy
from std_msgs.msg import Int32, Float32


class Myagv:
    """
        This is robot Myagv robot 
    """

    def __init__(self) -> None:

        rospy.init_node('AGV_arduino_proxy')

        #--- Create the Subscriber to AGV IR Sensors Cmd
        self.ros_sub_ir_sensor = rospy.Subscriber("/IR_Sensor_Feedback", Int32, self.error_callback)

        #----Create THe subscriber for ultrasonic sensor Module
        self.ros_sub_bluetooth_sensor  = rospy.Subscriber("/Ultrasonic_Input", Int32, self.obstacle_callback)

        #----Log subscriber connection
        rospy.loginfo("> Subscriber corrrectly initialized")

        #----Initialize the Kp, Ki, and Kd Values
        self.kp = 0.25
        self.ki = 0.045
        self.kd = 0.15

        #intialize the value for the PID logic
        self.error = 0
        self.proportional_gain = 0
        self.integral_gain = 0
        self.derivative_gain = 0
        self.pid_gain = 0
        self.previous_error = 0
     
    
        #----Dc left Motor dir and PWM val
        self.left_motor_cmd = 20
        self.right_motor_cmd = 20
        
        #----Dc right Motor dir and PWM val forward motion
        # self.left_motor_cmd_direction = "frd"
        # self.right_motor_cmd_direction = "frd"

        #-----Intialize The Publisher for left and right motor cmd
        self.ros_pub_leftmotor_cmd = rospy.Publisher("/cmd_left_motor", Float32 , queue_size=1)
        self.ros_pub_rightmotor_cmd = rospy.Publisher("/cmd_right_motor",Float32, queue_size=1)

        # self.ros_pub_leftmotor_cmd_direction = rospy.Publisher("/cmd_left_motor_direction", String , queue_size=1)
        # self.ros_pub_rightmotor_cmd_direction = rospy.Publisher("/cmd_right_motor_direction",String, queue_size=1)

        #---Intialize mission status
        self.mission_status_flag = 0

        #----Intialize Collusion avoidance statue
        self.obstacle_detection_distance = 20


    def error_callback(self, error_code):

        """
        update the error from subsscriber callback
        """
        self.error = error_code.data
        self.run()

    def obstacle_callback(self, obstacle_status):

        """
        update the error from subsscriber callback
        """
        self.obstacle_detection_distance = obstacle_status.data
       

    def agv_controller(self):
        """
        control the motor for the based on the feedback and PID controller
        """

        gained_error = self.error
        # obstacle_detection = self.obstacle_detection_distance

        print(">agvcontroller fired")

        if (gained_error == 200):

            if (self.previous_error > 0):
                self.send_leftmotor_cmd(20)
                self.send_rightmotor_cmd(-20)
            elif (self.previous_error < 0):
                self.send_leftmotor_cmd(-20)
                self.send_rightmotor_cmd(20)
            
            print(">passed zero IR")



        elif (gained_error == 300):

            self.ros_pub_leftmotor_cmd.publish(0)
            self.ros_pub_rightmotor_cmd.publish(0)
            self.mission_status_flag = 1

            print(">Paseed all IR")


        elif (gained_error == 0):

            self.send_leftmotor_cmd(20)
            self.send_rightmotor_cmd(20)

            print(">passed Zero Error")

        else:

            self.pid_controller(gained_error)

            print(">passed PID calc")

            pid_left_motor_cmd = self.left_motor_cmd - self.pid_gain
            pid_right_motor_cmd = self.right_motor_cmd + self.pid_gain


            self.send_leftmotor_cmd(pid_left_motor_cmd)
            self.send_rightmotor_cmd(pid_right_motor_cmd)

            print(">motor cmd published")


           

    def pid_controller(self, current_error):

        """
        PID controller logic
        """

        self.proportional_gain = current_error
        self.integral_gain = self.integral_gain + current_error
        self.derivative_gain = current_error - self.previous_error

        self.pid_gain = self.proportional_gain * self.kp + self.integral_gain * self.ki + self.derivative_gain * self.kd
        self.previous_error = current_error
        print("PID gain is :", self.pid_gain)




    def send_leftmotor_cmd(self, pwm):
        """
        Send PWM message for left Motor
        """

        # if (pwm > 0):
        #     left_motor_dir = "frd"
        # else: 
        #     left_motor_dir = "rev"
       
        self.left_motor_cmd = pwm
        # self.left_motor_cmd_direction = left_motor_dir
        self.ros_pub_leftmotor_cmd.publish(self.left_motor_cmd)
        # self.ros_pub_leftmotor_cmd_direction.publish(self.left_motor_cmd_direction)
        print("left_motor_cmd: ", pwm)

    def send_rightmotor_cmd(self, pwm):
        """
        Send PWM message for right Motor
        """
    
        # if (pwm > 0):
        #     right_motor_dir = "frd"
        # else: 
        #     right_motor_dir = "rev"

        self.right_motor_cmd = pwm
        # self.right_motor_cmd_direction = right_motor_dir
        self.ros_pub_rightmotor_cmd.publish(self.right_motor_cmd)
        # self.ros_pub_rightmotor_cmd_direction.publish(self.right_motor_cmd_direction)

        print("right_motor_cmd:", pwm)

    def run(self):
        """
        main function running inside the node
        """
        #--- Set the control rate
        # rate = rospy.Rate(4)
        # while not rospy.is_shutdown():
        if (self.obstacle_detection_distance < 10):

            self.ros_pub_leftmotor_cmd.publish(0)
            self.ros_pub_rightmotor_cmd.publish(0)
            self.mission_status_flag = 2
            print(">passed obstacle avioudance")

        else:
            self.agv_controller()
                
        #rate.sleep()

if __name__ == '__main__':
    ag = Myagv()
    rospy.spin()
    # ag.run()