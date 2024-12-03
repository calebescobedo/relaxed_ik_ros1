#include <iostream>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <thread>
#include <franka/gripper.h>
#include "cnpy.h"
#include <chrono>
#include <franka/rate_limiting.h>
#include <franka/robot_state.h>

std::tuple<std::vector<double>, std::vector<std::array<double, 7>>, std::vector<std::array<double, 7>>, int> get_npz()
{
    std::string filename = "/home/caleb/ros_relaxed_ik_ws/src/relaxed_ik_ros1/scripts/shy.npz";
    srand(time(NULL));
    cnpy::npz_t npz = cnpy::npz_load(filename);
    std::vector<double> initial_joint_config;
    std::vector<std::array<double, 7>> joint_velocities_vector;
    std::vector<std::array<double, 7>> joint_angles_vector;
    cnpy::NpyArray joint_velocities_npz = npz["qdot"];
    cnpy::NpyArray joint_angles_npz = npz["q"];
    cnpy::NpyArray initial_joint_config_array_npz = npz["q_init"];
    cnpy::NpyArray num_steps_npz = npz["num_steps"];
    int* loaded_data_num_steps = num_steps_npz.data<int>();
    int num_steps = *loaded_data_num_steps;
    num_steps = 4611;
    std::array<double, 7>* loaded_data_initial_joint_config= initial_joint_config_array_npz.data<std::array<double, 7>>();
    std::cout<<num_steps<<std::endl;
    for (int i = 0; i< 7; i++) 
    {
        initial_joint_config.push_back(loaded_data_initial_joint_config[0][i]);
    }
    std::array<double, 7> *joint_velocities = joint_velocities_npz.data<std::array<double, 7>>();
    for (int i = 0; i< 4097; i++)
    {
        joint_velocities_vector.push_back(joint_velocities[i]);
    }
    std::array<double, 7> *joint_angles = joint_angles_npz.data<std::array<double, 7>>();
    for (int i = 0; i< 4611; i++)
    {
        joint_angles_vector.push_back(joint_angles[i]);
    }
    
    return std::make_tuple(initial_joint_config, joint_angles_vector, joint_velocities_vector, num_steps);
}

void set_joint_and_collision_behaviour(franka::Robot *robot)
    {
    robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot->setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);
    }

int main() {
    try
    {  
        std::string name = "fr3";
        franka::Robot robot("192.168.0.100");
        franka::Gripper gripper("192.168.0.100");
        franka::Model model = robot.loadModel();
        //robot.automaticErrorRecovery();
        set_joint_and_collision_behaviour(&robot);
        std::vector<std::array<double, 7>> joint_accelerations_real;
        std::vector<std::array<double, 7>> joint_velocities_real;
        std::vector<std::array< double, 7>> joint_configs_real;
        std::array<double, 7> initial_joint_config_real;
        std::vector<double> time_real;
        for(int i=0;i<1;i++)
        {
            std::tuple<std::vector<double>, std::vector<std::array<double, 7>>, std::vector<std::array<double, 7>>, int> traj = get_npz();
            std::vector<std::array<double, 7>> joint_velocities_npz = std::get<2>(traj);
            std::vector<std::array<double, 7>> joint_angles_npz = std::get<1>(traj);
            int num_steps_npz = joint_angles_npz.size(); 
            std::cout<<num_steps_npz<<"AAA"<<std::endl;
            double time_initial = 0.0;
            int traj_index = 0;
            franka::JointPositions output_joint_q =  joint_angles_npz[0];
            double time = 0.0;
            int step_count = 1;
            auto start = std::chrono::high_resolution_clock::now(); 
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::high_resolution_clock::now() - start);
            auto joint_velocity_call_back = [&](const franka::RobotState &robot_state, franka::Duration period)
                -> franka::JointVelocities
            {
                //joint_velocities_real.push_back(robot_state.dq);
                joint_configs_real.push_back(robot_state.q);
                if (step_count < num_steps_npz)
                {
                    duration = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::high_resolution_clock::now() - start);
                    std::cout<<duration.count()<<std::endl;
                    if(duration.count() > 666)
                    {
                        step_count+=1;
                        //for(int j=0;j<7;j++)
                        //{
                        //    joint_velocities_npz[step_count][j] = joint_velocities_npz[step_count][j]/3;
                        //     //std::cout<<joint_velocities_npz[traj_index][j]<<",";
                        //}
                        start = std::chrono::high_resolution_clock::now();
                    }
                    
                    //std::cout<<std::endl;
                    // traj_index = np.where(time_npz == time)
                    // int idx = 0;
                    //for (auto t : time_npz) {
                        //std::cout<<t<<", "<<time<<std::endl;
                        // if (std::round(time*1000)/1000 == std::round(t*1000)/1000) {
                            // break;
                        // }
                        // idx++;
                    // }
                    // std::cout<<idx<<std::endl;
                    //std::cout<<time<<", "<<time_npz[traj_index]<<std::endl;
                    //franka::JointVelocities output_joint_v = {0,0,0,0,0,0,0};                    
                    //franka::JointPositions output_joint_q = {joint_angles_npz[step_count-1][0]+((duration.count()/66)*(joint_angles_npz[step_count][0]-joint_angles_npz[step_count-1][0])),joint_angles_npz[step_count-1][1]+((duration.count()/66)*(joint_angles_npz[step_count][1]-joint_angles_npz[step_count-1][1])),joint_angles_npz[step_count-1][2]+((duration.count()/66)*(joint_angles_npz[step_count][2]-joint_angles_npz[step_count-1][2])),joint_angles_npz[step_count-1][3]+((duration.count()/66)*(joint_angles_npz[step_count][3]-joint_angles_npz[step_count-1][3])),joint_angles_npz[step_count-1][4]+((duration.count()/66)*(joint_angles_npz[step_count][4]-joint_angles_npz[step_count-1][4])),joint_angles_npz[step_count-1][5]+((duration.count()/66)*(joint_angles_npz[step_count][5]-joint_angles_npz[step_count-1][5])),joint_angles_npz[step_count-1][6]+((duration.count()/66)*(joint_angles_npz[step_count][6]-joint_angles_npz[step_count-1][6]))};
                    franka::JointVelocities output_joint_v = {joint_velocities_npz[step_count-1][0]+((duration.count()/666)*(joint_velocities_npz[step_count][0]-joint_velocities_npz[step_count-1][0])),joint_velocities_npz[step_count-1][1]+((duration.count()/666)*(joint_velocities_npz[step_count][1]-joint_velocities_npz[step_count-1][1])),joint_velocities_npz[step_count-1][2]+((duration.count()/666)*(joint_velocities_npz[step_count][2]-joint_velocities_npz[step_count-1][2])),joint_velocities_npz[step_count-1][3]+((duration.count()/666)*(joint_velocities_npz[step_count][3]-joint_velocities_npz[step_count-1][3])),joint_velocities_npz[step_count-1][4]+((duration.count()/666)*(joint_velocities_npz[step_count][4]-joint_velocities_npz[step_count-1][4])),joint_velocities_npz[step_count-1][5]+((duration.count()/666)*(joint_velocities_npz[step_count][5]-joint_velocities_npz[step_count-1][5])),joint_velocities_npz[step_count-1][6]+((duration.count()/666)*(joint_velocities_npz[step_count][6]-joint_velocities_npz[step_count-1][6]))};
                    //joint_velocities_npz[step_count][j] = joint_velocities_npz[step_count][j]/3;
                    //std::cout<<joint_velocities_npz[traj_index][j]<<",";
                    //traj_index+=1;
                    //time += 0.001;//period.toSec();
                    //franka::JointVelocities output_joint_v = joint_velocities_npz[step_count];
                    return output_joint_v;
                }
                else
                {
                    std::cout<<"motion finished"<<std::endl;
                    franka::JointVelocities output = {0, 0, 0, 0, 0, 0, 0};
                    //franka::JointPositions output = joint_angles_npz[0];
                    return franka::MotionFinished(output);
                }
            };

            robot.control(joint_velocity_call_back);
            
        }
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}



/*
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <unistd.h>    
#include <fstream>
#include <time.h>
#include <string>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/rate_limiting.h>
#include <franka/robot_state.h>
#include <cmath>
#include <sstream>
#include <ruckig/ruckig.hpp>
#include <Eigen/QR>    
#include <Eigen/LU>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
// include kdl tree
#include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "common.h" // MotionGenerator lives here

#include "cnpy.h"

// TODO: MUST DO A RAMP UP AND RAMP DOWN

// get a random edge tuple of initial ee pose, duration, and ee_velocity from npz file


std::tuple<std::vector<double>, std::vector<std::array<double, 7>>, std::vector<std::array<double, 7>>, int> get_npz()
{
    std::string filename = "/home/ricciardo/panda-planning/assets/psmp.npz";
    srand(time(NULL));
    cnpy::npz_t npz = cnpy::npz_load(filename);
    std::vector<double> initial_joint_config;
    std::vector<std::array<double, 7>> joint_velocities_vector;
    std::vector<std::array<double, 7>> joint_angles_vector;
    cnpy::NpyArray joint_velocities_npz = npz["qdot"];
    cnpy::NpyArray joint_angles_npz = npz["q"];
    cnpy::NpyArray initial_joint_config_array_npz = npz["start_q"];
    cnpy::NpyArray num_steps_npz = npz["num_steps"];
    int* loaded_data_num_steps = num_steps_npz.data<int>();
    int num_steps = *loaded_data_num_steps;
    std::array<double, 7>* loaded_data_initial_joint_config= initial_joint_config_array_npz.data<std::array<double, 7>>();
    for (int i = 0; i< 7; i++) 
    {
        initial_joint_config.push_back(loaded_data_initial_joint_config[0][i]);
    }
    
    std::array<double, 7> *joint_velocities = joint_velocities_npz.data<std::array<double, 7>>();
    for (int i = 0; i< num_steps; i++)
    {
        joint_velocities_vector.push_back(joint_velocities[i]);
    }
    std::array<double, 7> *joint_angles = joint_angles_npz.data<std::array<double, 7>>();
    for (int i = 0; i< num_steps; i++)
    {
        joint_angles_vector.push_back(joint_angles[i]);
    }
    
    return std::make_tuple(initial_joint_config, joint_angles_vector, joint_velocities_vector, num_steps);
}

void set_joint_and_collision_behaviour(franka::Robot *robot)
{
    robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot->setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void move_to_default_pose(franka::Robot *robot)
{
    std::array<double, 7> q_default_goal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
    MotionGenerator motion_generator(0.5, q_default_goal); // speed factor, goal
    robot->control(motion_generator);
}

void move_to_initial_config(franka::Robot *robot, std::vector<double> initial_joint_config_npz )
{
    std::array<double, 7> q_initial_goal = {initial_joint_config_npz[0],initial_joint_config_npz[1],
        initial_joint_config_npz[2],initial_joint_config_npz[3],initial_joint_config_npz[4],
            initial_joint_config_npz[5],initial_joint_config_npz[6]};
    MotionGenerator motion_generator(0.5, q_initial_goal); // speed factor, goal
    robot->control(motion_generator);
}


int main()
{
    try
    {  
        std::string name = "panda";
        franka::Robot robot("192.168.0.2");
        franka::Gripper gripper("192.168.0.2");
        franka::Model model = robot.loadModel();

        set_joint_and_collision_behaviour(&robot);
        move_to_default_pose(&robot);
        //int no_of_edges = 20;
        std::vector<std::array<double, 7>> joint_accelerations_real;
        std::vector<std::array<double, 7>> joint_velocities_real;
        std::vector<std::array< double, 7>> joint_configs_real;
        std::array<double, 7> initial_joint_config_real;
        std::vector<double> time_real;
        for(int i=0;i<1;i++)
        {
            get_npz();
            std::tuple<std::vector<double>, std::vector<std::array<double, 7>>, std::vector<std::array<double, 7>>, int> traj = get_npz();
            std::vector<std::array<double, 7>> joint_velocities_npz = std::get<2>(traj);
            std::vector<std::array<double, 7>> joint_angles_npz = std::get<1>(traj);
            std::vector<double> initial_joint_config_npz = std::get<0>(traj);
            int num_steps_npz = std::get<3>(traj);
            //std::vector<float> time_npz = std::get<1>(traj);
            //double end_time = time_npz[time_npz.size()-1];
            // HARDCODE
            
            // double edgeDuration = 0.395985;

            //int test = get_20_random_edges();


            double time_initial = 0.0;
            int traj_index = 0;
            double time = 0.0;
            int step_count = 0;
            move_to_initial_config(&robot, initial_joint_config_npz); 
            auto joint_velocity_call_back = [&](const franka::RobotState &robot_state, franka::Duration period)
                -> franka::JointVelocities
            {
                // std::cout<<period.toSec()<<std::endl;
                //std::cout<<robot_state.tau_J[0]<<std::endl;
                joint_velocities_real.push_back(robot_state.dq);
                joint_configs_real.push_back(robot_state.q);
                if (step_count < num_steps_npz)
                {
                    // for(int j=0;j<7;j++)
                    // {
                    //     std::cout<<joint_velocities_npz[traj_index][j]<<",";
                    // }
                    // std::cout<<std::endl;
                    // traj_index = np.where(time_npz == time)
                    // int idx = 0;
                    // for (auto t : time_npz) {
                        // std::cout<<t<<", "<<time<<std::endl;
                        // if (std::round(time*1000)/1000 == std::round(t*1000)/1000) {
                            // break;
                        // }
                        // idx++;
                    // }
                    // std::cout<<idx<<std::endl;
                    //std::cout<<time<<", "<<time_npz[traj_index]<<std::endl;
                    franka::JointVelocities output_joint_v = joint_velocities_npz[step_count];   
                    //traj_index+=1;
                    step_count+=1;
                    //time += 0.001;//period.toSec();

                    return output_joint_v;
                }
                else
                {
                    std::cout<<"motion finished"<<std::endl;
                    franka::JointVelocities output = {0, 0, 0, 0, 0, 0, 0};
                    return franka::MotionFinished(output);
                }
            };

            robot.control(joint_velocity_call_back);
            
        }
    }
    catch (const franka::Exception &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
    return 0;
}*/