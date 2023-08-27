#include <ros/ros.h>
#include <stdlib.h>
#include<unistd.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include <math.h>
#include <stdio.h>
#include <sys/time.h>

class PosePubDetected
{
    private:
        //Subscribers
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Subscriber subM;
        ros::Subscriber subG;
        //ros::Subscriber subGa;
        ros::Subscriber subCoor;

        //Publishers
        ros::Publisher pub;
        ros::Publisher pubM;
        ros::Publisher pubG;
        //ros::Publisher pubGa;

    public:
        int task = 0;
        int numObj = 0;

        struct timeval begin, end;

        //Wanted x,y,z pose
        float wantX;
        float wantY;
        float wantZ;
        float wantOZ;

        //Pose to goto
        float gox;
        float goy;
        float goz;  
        bool coorSaved = false;

        //Msg to store subscriber information
        std::string cameraToController;
        int gripperToController;
    // # "Ready!" - 1
    // # "Object Grasped" - 2
    // # "Object lost" - 3
    // # "Gripper opened" - 4
        std::string gaussianToController;
        tf2::Quaternion myQuaternion;

        PosePubDetected()
        {
            //Subscribers
            sub = nh.subscribe("/franka_state_controller/franka_states", 1000,&PosePubDetected::msgCallback, this);
            subM = nh.subscribe("/camera2controller", 1000,&PosePubDetected::msgCallbackCamera, this);
            subG = nh.subscribe("/gripper2controller", 1000,&PosePubDetected::msgCallbackGripper, this);
            //subGa = nh.subscribe("/gaussian2controller", 1000,&PosePubDetected::msgCallbackGaussian, this);
            subCoor = nh.subscribe("/objCoordinates", 1000,&PosePubDetected::msgCallbackCoor, this);
            
            //Publishers
            pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/desktop/target_pose_cov_stmp_panda_link0", 10);
            pubM = nh.advertise<std_msgs::String>("/controller2camera", 10);
            pubG = nh.advertise<std_msgs::String>("/controller2gripper", 10);
            //pubGa = nh.advertise<std_msgs::String>("/controller2gaussian", 10);
        }

        void msgCallback(const franka_msgs::FrankaState& msg)
        {
            //Current position of the robot
            float currX = msg.O_T_EE[12];
            float currY = msg.O_T_EE[13];
            float currZ = msg.O_T_EE[14];

            //Position where to drop of the object. Dropping more objects then 4 normally results in an error.
            //float dropPos = 0.0 + 0.1 * numObj;
            float dropPos = 0.0;

            //If the gripper looses (does not detect) the object
            if (gripperToController == 3) { //Object lost
                task = 105;
            }
            
            //Start ActionPlan
            switch(task) {
                //Initialization
                case 0:
                    pubGripper("Are you ready?");
                    //pubGaussian("Are you ready?");
                    pubCamera("Are you ready?");
                    myQuaternion.setRPY( 0, 0, M_PI);
                    if ((cameraToController == "Ready!") && (gripperToController == 1/*Ready*/)) { 
                        gettimeofday(&begin, 0);
                        pubPose(0.1, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                        if(isPoseReached(currX, currY, currZ, 0.1, 0.0, 0.7))
                        {
                            ROS_WARN("All nodes ready.");
                            //ROS_INFO("Task 1 finished.");
                            task = 1;

                        }
                        break;
                    }
                    break;
                case 1:
                    pubCamera("Awaiting coordinates");
                    //pubGaussian("Extract width");
                    pubGripper("Open gripper");
                    if ((cameraToController == "No objects visible")){
                        pubPose(0.1, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                        task = 1;
                    } 
                    else if (cameraToController == "Coordinates are being published..."  && gripperToController ==4/*gripper oppened*/) { //&& gaussianToController == "Retrieved camera width estimate"
                        ROS_INFO("Going to y=%.2f x=%.2f z=%.2f oz=%.2f", wantX, wantY, wantZ, wantOZ);
                        coorSaved = true;
                        myQuaternion.setRPY( 0, 0, wantOZ);
                        gox = wantX;
                        goy = wantY;
                        goz = wantZ;  
                        task = 2;
                    }
                    break;
                case 2:
                    if (!coorSaved) {
                        task = 1;
                    } else {
                        pubPose(gox, goy, 0.35, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                        if(isPoseReached(currX, currY, currZ, gox, goy, 0.35))
                        {
                            //ROS_INFO("Task 2 finished.");
                            coorSaved = false;
                            task = 3;
                        }
                    }
                    break;
                // case 111:
                //     pubCamera("Stop sending width");
                //     pubPose(gox, goy, 0.35, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                //     if(isPoseReached(currX, currY, currZ, gox, goy, 0.35))
                //     {
                //         ROS_INFO("Task 3 finished.");
                //         task = 5;
                //     }
                //     break;
                case 3:
                    pubCamera("Reset");
                    pubPose(gox, goy, goz, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached_Precise(currX, currY, currZ, gox, goy, goz))
                    {
                        //ROS_INFO("Task 3 finished.");
                        task = 4;
                    }
                    break;
                case 4:
                    pubGripper("Ready to grasp");
                    if (gripperToController == 2/*Object grasped*/) {
                        //ROS_INFO("Task 4 finished.");
                        task = 5;
                    }
                    break;
                case 5:
                    //pubGaussian("Extract width");
                    //pubGripper("Object grasped");
                    pubPose(gox, goy, 0.4, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, gox, goy, 0.4))
                    {
                        myQuaternion.setRPY( 0, 0, M_PI);
                        //ROS_INFO("Task 5 finished.");
                        task = 6;
                    }
                    break;
                case 6:
                    //pubGaussian("Extract width");
                    //pubGripper("Object grasped");
                    pubPose(0.75, dropPos, 0.35, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.75, dropPos, 0.35))
                    {
                        //ROS_INFO("Task 6 finished.");
                        task = 7;
                    }
                    break;
                case 7:
                    //pubGaussian("Extract width");
                    //pubGripper("Object grasped");
                    pubPose(0.75, dropPos, 0.2, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.75, dropPos, 0.2))
                    {
                        //ROS_INFO("Task 7 finished.");
                        task = 8;
                    }
                    break;
                case 8:
                    //pubGaussian("Extract width");
                    pubGripper("Open gripper");
                    if (gripperToController == 4/*Gripper opened*/) {
                        numObj = numObj + 1;
                        ROS_WARN("Object Number %d dropped off succesfully!", numObj);
                        gettimeofday(&end, 0);
                        long seconds = end.tv_sec - begin.tv_sec;
                        long microseconds = end.tv_usec - begin.tv_usec;
                        double elapsed = seconds + microseconds*1e-6;

                        //ROS_WARN("Time measured: %.3f seconds.\n", elapsed);
                        //ROS_INFO("Task 8 finished.");
                        task = 9;
                    }
                    break;
                case 105:
                    //pubGaussian("Reset width");
                    pubGripper("Open gripper");
                    if (gripperToController == 4/*Gripper opened*/) {
                        myQuaternion.setRPY( 0, 0, M_PI);
                        ROS_INFO("Task 105 finished.");
                        task = 115;
                    }
                    break;
                // case 8:
                //     //pubGaussian("Reset width");
                //     pubPose(0.7, 0.0, 0.5, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                //     if(isPoseReached(currX, currY, currZ, 0.7, 0.0, 0.5))
                //     {
                //         ROS_INFO("Task 98 finished.");
                //         gettimeofday(&begin, 0);
                //         task = 10;
                //     }
                //     break;
                case 115:
                    //pubGaussian("Reset width");
                    pubPose(gox, goy, 0.4, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, gox, goy, 0.4))
                    {
                        myQuaternion.setRPY( 0, 0, M_PI);
                        //ROS_INFO("Task 115 finished.");
                        task = 9;
                    }

                    break;
                case 9:
                    //pubGaussian("Reset width");
                    pubPose(0.1, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.1, 0.0, 0.7))
                    {
                        //ROS_INFO("Task 9 finished.");
                        task = 1;
                    }
                    break;
                default:
                    pubPose(0.1, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
            }


        }

        /*
        Function continously checking if the wanted pose is close to the current pose
        */
        bool isPoseReached(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
        {
            float allowedError = 0.035;
            // if((currX < wantX+allowedError && currX > wantX-allowedError) && (currY < wantY+allowedError && currY > wantY-allowedError) && (currZ < wantZ+allowedError && currZ > wantZ-allowedError))
            // {
            //     return true;

            // } else 
            // {
            //     return false;
            // }
            float distance = sqrt(pow(currX - wantX, 2) + pow(currY - wantY, 2) + pow(currZ - wantZ, 2));
            if (distance <= allowedError) {
                return true;
            } else {
                return false;
            }
        }

        bool isPoseReached_Precise(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
        {
            float allowedError = 0.025;
            // if((currX < wantX+allowedError && currX > wantX-allowedError) && (currY < wantY+allowedError && currY > wantY-allowedError) && (currZ < wantZ+allowedError && currZ > wantZ-allowedError))
            // {
            //     return true;

            // } else 
            // {
            //     return false;
            // }
            float distance = sqrt(pow(currX - wantX, 2) + pow(currY - wantY, 2) + pow(currZ - wantZ, 2));
            if (distance <= allowedError) {
                return true;
            } else {
                return false;
            }
        }

        /*
        Publisher functions
        */
        void pubCamera(std::string action) 
        {
            //Initialization
            std_msgs::String act;
            
            act.data = action;
            
            //Publish Hardcoded pose
            pubM.publish(act);
        }

        void pubGripper(std::string actionGripper) 
        {
            //Initialization
            std_msgs::String actG;
            
            actG.data = actionGripper;
            
            //Publish Hardcoded pose
            pubG.publish(actG);
        }

        // void pubGaussian(std::string actionGaussian) 
        // {
        //     //Initialization
        //     std_msgs::String actGa;
            
        //     actGa.data = actionGaussian;
            
        //     //Publish Hardcoded pose
        //     pubGa.publish(actGa);
        // }

        void pubPose(double px, double py, double pz, double ox, double oy, double oz, double ow) 
        {
            //Initialization
            geometry_msgs::PoseWithCovarianceStamped msg;
            
            msg.pose.pose.position.x  = px;
            msg.pose.pose.position.y  = py;
            msg.pose.pose.position.z  = pz;
            msg.pose.pose.orientation.x = ox;
            msg.pose.pose.orientation.y = oy;
            msg.pose.pose.orientation.z = oz;
            msg.pose.pose.orientation.w = ow;
            
            //Publish Hardcoded pose
            pub.publish(msg);
        }

        /*
        Subscriber functions
        */
        void msgCallbackCamera(const std_msgs::String& act)
        {
            cameraToController = act.data;
        }

        void msgCallbackGripper(const std_msgs::Int32& actG)
        {
            gripperToController = actG.data;
        }

        void msgCallbackGaussian(const std_msgs::String& actGa)
        {
            gaussianToController = actGa.data;
        }

        void msgCallbackCoor(const geometry_msgs::Twist& msgCoor)
        {
            //Transformation to World coordinates //0.021 factor to take into account the lateral offset from center of the gripper to end effector of the robot.
            wantOZ = (msgCoor.angular.z > M_PI/2) ? msgCoor.angular.z : msgCoor.angular.z + M_PI;  
            wantX = msgCoor.linear.y + 0.355 +0.026*sin(wantOZ); //Y distance (m) in global coordinates from center of robot to origin of the conveyor belt (top left corner)
            wantY = msgCoor.linear.x - 0.481 +0.026*cos(wantOZ); //X distance (m) in global coordinates from center of robot to origin of the conveyor belt (top left corner)
            wantZ = (wantY > 0.45) ? msgCoor.linear.z + 0.26 : msgCoor.linear.z + 0.25; //Height we will gras objects from
                 
        }

};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "publish_detected_coordinates");
    PosePubDetected ppd;
    ros::spin();
    return 0;
};