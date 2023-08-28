/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2021 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

// Include all the libraries that will be used by the robot
// and will be required when calculating the waypoints
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

#include "utilities.h"

#include <boost/asio.hpp>
#include <stack>
#include <ctime>

#define PORT 10000

// modified transformation matrix, including the dimensions of the end-effector
const float pi = 3.14159265358979323846f;
const float offset_vert = 0.297f;
const float offset_horz = 0.015f;

// ring dimensions
const float diameter = 0.010f;
const float radius = diameter/2;

// to assure that the end-effector abducts then adducts
// explained further more into the code
int scale;

namespace k_api = Kinova::Api;
// for the end-effector
using namespace boost::asio;

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{60};

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise_cart.get_future.get() to wait and get the value
// Inform if the action was ABORTED (stopped halfway through and exited safely)
// or ENDED (finished the full action successfully)
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise_cart)
{
    return [&finish_promise_cart] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise_cart.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                returnAction = action_event;
                break;
            default:
                break;
        }
    };
}

// defining the indices for the waypoints array defined later
static constexpr int posX = 0;
static constexpr int posY = 1;
static constexpr int posZ = 2;
static constexpr int posBlendingRadius = 3;
static constexpr int posThetaX = 4;
static constexpr int posThetaY = 5;
static constexpr int posThetaZ = 6;

std::stack<clock_t> tictoc_stack;
// calculate the time taken to do the task
void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}


// Helper function to populate Cartesian waypoint
// so when multiple waypoints are stacked this will appropriately group them
void populateCartesianCoordinate(k_api::Base::CartesianWaypoint* cartesianCoordinate, std::vector<float> waypointDefinition)
{
    static const k_api::Common::CartesianReferenceFrame kReferenceFrame = k_api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;


    cartesianCoordinate->mutable_pose()->set_x(waypointDefinition[posX]);
    cartesianCoordinate->mutable_pose()->set_y(waypointDefinition[posY]);
    cartesianCoordinate->mutable_pose()->set_z(waypointDefinition[posZ]);
    cartesianCoordinate->set_blending_radius(waypointDefinition[posBlendingRadius]);

    cartesianCoordinate->mutable_pose()->set_theta_x(waypointDefinition[posThetaX]);
    cartesianCoordinate->mutable_pose()->set_theta_y(waypointDefinition[posThetaY]);
    cartesianCoordinate->mutable_pose()->set_theta_z(waypointDefinition[posThetaZ]);


    cartesianCoordinate->set_reference_frame(kReferenceFrame);
}

/// Puncturing Task
bool example_trajectory_one(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool success = false;
    // refresh the robot to be able to access the current configuration of the robot
    auto feedback = base_cyclic->RefreshFeedback();

    // get current poses of robot
    float actual_x = feedback.base().tool_pose_x();
    float actual_y = feedback.base().tool_pose_y();
    float actual_z = feedback.base().tool_pose_z();
    float actual_theta_x = feedback.base().tool_pose_theta_x();
    float actual_theta_y = feedback.base().tool_pose_theta_y();
    float actual_theta_z = feedback.base().tool_pose_theta_z();

    std::vector<std::vector<float>> waypointsDefinition;
    const float kTheta_x = actual_theta_x;
    const float kTheta_y = actual_theta_y;
    const float kTheta_z = actual_theta_z;

    // A checking test is done to assure that the values of theta y is correct
    // so when the angle is incremented or decremented from the set value
    // the movement is correct and it rotates away from the ring before rotating towards it
    if (kTheta_y <= 15 && kTheta_y >= -15){
        scale = 1;
    }
    else if ((kTheta_y <= -165 && kTheta_y >= -180) || (kTheta_y <= 180 && kTheta_y >= 165)){
        scale = -1;
    }
    else{
        std::cout<<"Not Correct Configuration; Way Off Scale. Fix the End-Effector"<<endl;
        return true;
    }
            // first move the robot top left
    waypointsDefinition = {{actual_x + 0.010f,  actual_y,  actual_z + 0.0200f,  0.00f,  kTheta_x, kTheta_y, kTheta_z},
            // now rotate it, while considering the RCM to be at the tip of the end-effector, so the offset from the rotation is considered
            // recall that cartesian displacements do not affect the RCM as the end-effector and the tool frame are within the same axes
                           {actual_x + 0.015f - offset_vert*(sin(30*pi/180)),  actual_y,  actual_z + 0.0200f - offset_vert*(1-cos(30*pi/180)),  0.00f,  kTheta_x, kTheta_y + scale * 30.0f, kTheta_z},
            // then start returning to original pose but with a rotation, inserting the needle a bit to the ring's silicone
                           {actual_x + 0.005f - offset_vert*(sin(15*pi/180)),  actual_y,  actual_z + 0.0000f - offset_vert*(1-cos(15*pi/180)),  0.00f,  kTheta_x, kTheta_y + scale * 15.0f, kTheta_z},
            // go further into the hole, and position the end-effector properly (this will probably be different in the case of an actual tissue)
                           {actual_x + 0.000f - offset_vert*(sin(00*pi/180)),  actual_y,  actual_z - (radius) - offset_vert*(1-cos(00*pi/180)),  0.00f,  kTheta_x, kTheta_y + scale * 00.0f, kTheta_z},
            // rotate to let the tip go out on the other side
                           {actual_x - (offset_horz) + offset_vert*(sin(30*pi/180)),  actual_y,  actual_z - (radius) - offset_vert*(1-cos(30*pi/180)),  0.00f,  kTheta_x, kTheta_y - scale * 30.0f, kTheta_z}};

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create the trajectory
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();
    wpts.set_duration(0.0); // as fast as possible
    wpts.set_use_optimal_blending(false);

    // Start waypoint list creation
    int index  = 0;
    for(std::vector<std::vector<float>>::iterator it = waypointsDefinition.begin();
        it != waypointsDefinition.end(); ++it, ++index)
    {
        k_api::Base::Waypoint *wpt = wpts.add_waypoints();
        if(wpt != nullptr)
        {
            wpt->set_name(std::string("waypoint_") + std::to_string(index));
            Kinova::Api::Base::CartesianWaypoint* coordinate = wpt->mutable_cartesian_waypoint();
            if(coordinate != nullptr)
            {
                populateCartesianCoordinate(coordinate, *it);
            }
        }
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart_end = base->OnNotificationActionTopic( create_event_listener_by_promise(finish_promise_cart),
                                                                                 k_api::Common::NotificationOptions());

    // Verify validity of waypoints
    auto result = base->ValidateWaypointList(wpts);

    if(result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {
        // Execute action
        try
        {
            // Move arm with waypoints list
            std::cout << "Moving the arm creating a trajectory of " << waypointsDefinition.size() << " cartesian waypoints" << std::endl;
            base->ExecuteWaypointTrajectory(wpts);
        }
        catch(k_api::KDetailedException& ex)
        {
            std::cout << "Try catch error executing normal trajectory" << std::endl;
            // In this case that an error was obtained, it is best to be printed to understand the fault
            auto error_info = ex.getErrorInfo().getError();
            std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

            std::cout << "KError error_code: " << error_info.error_code() << std::endl;
            std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
            std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

            // Error codes by themselves are not very verbose if their corresponding enum value isnt seen
            // Google::protobuf helpers are thus used to get the string enum element for every error code and sub-code
            std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
            std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
            return false;
        }
        // Wait for future value from promise
        const auto cart_end_status = finish_future_cart.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle_cart_end);

        // if the robot isnt ready for any possible reason, the cartesian trajectory isnt executed and the notification is printed
        if(cart_end_status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait for cartesian waypoint trajectory" << std::endl;
            return false;
        }

        // if robot is ready, send the action to let the robot execute the action
        const auto cart_promise_event = finish_future_cart.get();
        std::cout << "Cartesian Waypoint Trajectory Completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(cart_promise_event) << std::endl;
        success = true;

    }
    else
    {
        std::cout << "Error found in trajectory" << std::endl;
        // print the error
        result.trajectory_error_report().PrintDebugString();
    }

    return success;
}

/// Repositioning of the Robot
bool example_trajectory_two(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool success = false;
    auto feedback = base_cyclic->RefreshFeedback();

    // get current poses of robot
    float actual_x = feedback.base().tool_pose_x();
    float actual_y = feedback.base().tool_pose_y();
    float actual_z = feedback.base().tool_pose_z();
    float actual_theta_x = feedback.base().tool_pose_theta_x();
    float actual_theta_y = feedback.base().tool_pose_theta_y();
    float actual_theta_z = feedback.base().tool_pose_theta_z();

    std::vector<std::vector<float>> waypointsDefinition;
    const float kTheta_x = actual_theta_x;
    const float kTheta_y = actual_theta_y;
    const float kTheta_z = actual_theta_z;

    if (kTheta_y <= -15 && kTheta_y >= -45){
        scale = 1;
    }
    else if (kTheta_y >= -165 && kTheta_y <= -135){
        scale = -1;
    }
    else{
        std::cout<<"Not Correct Configuration; Way Off Scale."<<endl;
        return true;
    }

    // start moving the robot away and reposition the robot toward the ring with a horizontal position
    waypointsDefinition = {{actual_x + 0.050f,  actual_y,  actual_z + 0.062f,  0.00f,  kTheta_x, kTheta_y + 00.0f, kTheta_z},
                           // start going towards the ring and stop on top of it
                           {actual_x + 0.337f - offset_vert*sin(60*pi/180),  actual_y,  actual_z + 0.2105f - offset_vert*(1 - cos(60*pi/180)),  0.00f,  kTheta_x, kTheta_y + scale * 30.0f, kTheta_z},
                           {actual_x + 0.2485f - offset_vert*sin(30*pi/180),  actual_y,  actual_z + 0.1018f - offset_vert*(1 - cos(30*pi/180)),  0.00f,  kTheta_x, kTheta_y + scale * 00.0f, kTheta_z},
                           {actual_x - 0.160f + offset_vert*sin(75*pi/180),  actual_y,  actual_z + 0.062f - offset_vert*(1 - cos(75*pi/180)),  0.00f,  kTheta_x, kTheta_y - scale * 45.0f, kTheta_z}};

    // Make sure the arm is in Single Level Servoing before executing an ActionY
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create the trajectory
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();
    wpts.set_duration(0.0); // as fast as possible
    wpts.set_use_optimal_blending(true);

    // Start waypoint list creation
    int index  = 0;
    for(std::vector<std::vector<float>>::iterator it = waypointsDefinition.begin();
        it != waypointsDefinition.end(); ++it, ++index)
    {
        k_api::Base::Waypoint *wpt = wpts.add_waypoints();
        if(wpt != nullptr)
        {
            wpt->set_name(std::string("waypoint_") + std::to_string(index));
            Kinova::Api::Base::CartesianWaypoint* coordinate = wpt->mutable_cartesian_waypoint();
            if(coordinate != nullptr)
            {
                populateCartesianCoordinate(coordinate, *it);
            }
        }
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart_end = base->OnNotificationActionTopic( create_event_listener_by_promise(finish_promise_cart),
                                                                                 k_api::Common::NotificationOptions());

    // Verify validity of waypoints
    auto result = base->ValidateWaypointList(wpts);

    if(result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {
        // Execute action
        try
        {
            // Move arm with waypoints list
            std::cout << "Moving the arm creating a trajectory of " << waypointsDefinition.size() << " cartesian waypoints" << std::endl;
            base->ExecuteWaypointTrajectory(wpts);
        }
        catch(k_api::KDetailedException& ex)
        {
            std::cout << "Try catch error executing normal trajectory" << std::endl;
            // Print the error information and error codes
            auto error_info = ex.getErrorInfo().getError();
            std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

            std::cout << "KError error_code: " << error_info.error_code() << std::endl;
            std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
            std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

            std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
            std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
            return false;
        }
        // Wait for future value from promise
        const auto cart_end_status = finish_future_cart.wait_for(TIMEOUT_DURATION);

        base->Unsubscribe(promise_notification_handle_cart_end);

        if(cart_end_status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait for cartesian waypoint trajectory" << std::endl;
            return false;
        }

        const auto cart_promise_event = finish_future_cart.get();
        std::cout << "Cartesian Waypoint Trajectory Completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(cart_promise_event) << std::endl;
        success = true;

    }
    else
    {
        std::cout << "Error found in trajectory" << std::endl;
        result.trajectory_error_report().PrintDebugString();
    }

    return success;
}

/// Moving away from puncture site
bool example_trajectory_three(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool success = false;
    auto feedback = base_cyclic->RefreshFeedback();

    // get current poses of robot
    float actual_x = feedback.base().tool_pose_x();
    float actual_y = feedback.base().tool_pose_y();
    float actual_z = feedback.base().tool_pose_z();
    float actual_theta_x = feedback.base().tool_pose_theta_x();
    float actual_theta_y = feedback.base().tool_pose_theta_y();
    float actual_theta_z = feedback.base().tool_pose_theta_z();

    std::vector<std::vector<float>> waypointsDefinition;
    const float kTheta_x = actual_theta_x;
    const float kTheta_y = actual_theta_y;
    const float kTheta_z = actual_theta_z;

                            // first move the robot above and towards the left
    waypointsDefinition = {{actual_x + 0.02f,  actual_y,  actual_z + 0.025f,  0.00f,  kTheta_x, kTheta_y, kTheta_z},
                           {actual_x + 0.02f,  actual_y,  actual_z + 0.025f,  0.00f,  kTheta_x, kTheta_y, kTheta_z},
                           // then rotate it away
                           {actual_x + 0.02f,  actual_y,  actual_z + 0.025f,  0.00f,  kTheta_x + 15.0f, kTheta_y, kTheta_z}};

    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Create the trajectory
    k_api::Base::WaypointList wpts = k_api::Base::WaypointList();
    wpts.set_duration(0.0); // as fast as possible
    wpts.set_use_optimal_blending(true);

    // Start waypoint list creation
    int index  = 0;
    for(std::vector<std::vector<float>>::iterator it = waypointsDefinition.begin();
        it != waypointsDefinition.end(); ++it, ++index)
    {
        k_api::Base::Waypoint *wpt = wpts.add_waypoints();
        if(wpt != nullptr)
        {
            wpt->set_name(std::string("waypoint_") + std::to_string(index));
            Kinova::Api::Base::CartesianWaypoint* coordinate = wpt->mutable_cartesian_waypoint();
            if(coordinate != nullptr)
            {
                populateCartesianCoordinate(coordinate, *it);
            }
        }
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> finish_promise_cart;
    auto finish_future_cart = finish_promise_cart.get_future();
    auto promise_notification_handle_cart_end = base->OnNotificationActionTopic( create_event_listener_by_promise(finish_promise_cart),
                                                                                 k_api::Common::NotificationOptions());

    // Verify validity of waypoints
    auto result = base->ValidateWaypointList(wpts);

    if(result.trajectory_error_report().trajectory_error_elements_size() == 0)
    {
        // Execute action
        try
        {
            // Move arm with waypoints list
            std::cout << "Moving the arm creating a trajectory of " << waypointsDefinition.size() << " cartesian waypoints" << std::endl;
            base->ExecuteWaypointTrajectory(wpts);
        }
        catch(k_api::KDetailedException& ex)
        {
            std::cout << "Try catch error executing normal trajectory" << std::endl;
            // Print the error information and error codes
            auto error_info = ex.getErrorInfo().getError();
            std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;

            std::cout << "KError error_code: " << error_info.error_code() << std::endl;
            std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
            std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

            std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
            std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
            return false;
        }
        // Wait for future value from promise
        const auto cart_end_status = finish_future_cart.wait_for(TIMEOUT_DURATION);

        base->Unsubscribe(promise_notification_handle_cart_end);

        if(cart_end_status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait for cartesian waypoint trajectory" << std::endl;
            return false;
        }

        const auto cart_promise_event = finish_future_cart.get();
        std::cout << "Cartesian Waypoint Trajectory Completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(cart_promise_event) << std::endl;
        success = true;

    }
    else
    {
        std::cout << "Error found in trajectory" << std::endl;
        result.trajectory_error_report().PrintDebugString();
    }

    return success;
}

int main(int argc, char **argv)
{
    // connect to the arduino board of the end-effector
    io_service io;
    serial_port port(io, "COM3"); //connect to the correct port where the end-effector is attached
    port.set_option(serial_port_base::baud_rate(9600)); // specify the correct baud rate
    string tool = "2\n"; // in this experiment, the needle holder (2) was used, so this was defined
    string open = "o\n"; // to open the end-effector
    string close = "c\n"; // to close the end-effector

    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto device_manager = new k_api::DeviceManager::DeviceManagerClient(router);
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

    // Trajectory start
    bool success = true;

    // Time recording
    tic();
    // call to the arduino board and specify the tool that will be used
    write(port, buffer(tool.c_str(), tool.size()));
    // assure that the end-effector is closed
    write(port, buffer(close.c_str(), close.size()));
    /// Puncturing Task
    success &= example_trajectory_one(base, base_cyclic);
    // Open the end-effector
    write(port, buffer(open.c_str(), open.size()));
    toc();

    // After making sure that the string is pulled correctly, start repositioning the robot
    std::cout<<"Type Y for First Step."<<std::endl;
    char answer;
    char response = 'Y';
    std::cin>>answer;
    if(answer==response)
    {
        // recording for tying task
        tic();
        // Reposition the robot to go on top of the ring
        success &= example_trajectory_two(base, base_cyclic);
    }

    std::cout<<"Type Y for Next Step."<<std::endl;
    char answer_1;
    char response_1 = 'Y';
    std::cin>>answer_1;
    if(answer_1==response_1)
    {
        // close the end-effector after the string has been put
        write(port, buffer(close.c_str(), close.size()));
    }

    std::cout<<"Type Y for Next Step."<<std::endl;
    char answer_2;
    char response_2 = 'Y';
    std::cin>>answer_2;
    if(answer_2==response_2)
    {
        // form the knot
        success &= example_trajectory_three(base, base_cyclic);
    }
    toc();

    // open the end-effector to take out the string
    write(port, buffer(open.c_str(), open.size()));

    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete device_manager;
    delete session_manager;
    delete router;
    delete transport;

    return success? 0: 1;
}