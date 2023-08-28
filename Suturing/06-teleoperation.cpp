// Created by Toqa B. AbuBaker
// ################################################//
//          KINOVA ROBOT INITIALIZATION            //
/// the associated libraries to initialize the API are
/// included, as well as calling the library to connect
/// to the arduino board to control the end-effector
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>

#include <InterconnectConfigClientRpc.h>
#include <SessionManager.h>
#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

#include <thread>
#include <chrono>
#include "utilities.h"
#include <future>

#include <boost/asio.hpp>
#include <iostream>
#include <stack>

#define PORT 10000

namespace k_api = Kinova::Api;
using namespace boost::asio; // for arduino connection
// for safety purposes, the robot waits few seconds between actions
// so the robot doesn't go immediately to the second consecutive action
// the maximum wait duration is set to be 20 milliseconds
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};
// ################################################//

// ################################################//
//         XBOX CONTROLLER INITIALIZATION          //
/// to connect to the xbox controller, so libraries
/// such as windows and its associated XInput for the
/// connected controller to be utilized correctly
/// (the controller is connected to the laptop through a USB cable)
#include <iostream>
#include <windows.h>
#include <XInput.h>
#include <cmath>

#pragma comment(lib, "XInput.lib")
// ################################################//



// ################################################//
//             KINOVA ROBOT SETTINGS               //
/// all the functions that will be called by the controller to the robot are added here

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
// Inform if the action was ABORTED (stopped halfway through and exited safely)
// or ENDED (finished the full action successfully)
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

// code that sets to the admittance cartesian mode
bool example_admittance_cartesian(k_api::Base::BaseClient* base)
{
    // call the admittance from Kinova's base
    auto admittance = k_api::Base::Admittance();
    // Set the mode to Admittance Cartesian
    admittance.set_admittance_mode(k_api::Base::AdmittanceMode::CARTESIAN);
    base->SetAdmittance(admittance);
    return true;
}

// code that sets to admittance joint mode
bool example_admittance_joint(k_api::Base::BaseClient* base)
{
    // call the admittance from Kinova's base
    auto admittance = k_api::Base::Admittance();
    // Set the mode to Admittance Joint
    admittance.set_admittance_mode(k_api::Base::AdmittanceMode::JOINT);
    base->SetAdmittance(admittance);
    return true;
}

// code that sets to admittance nullspace mode
bool example_admittance_nullspace(k_api::Base::BaseClient* base)
{
    // call the admittance from Kinova's base
    auto admittance = k_api::Base::Admittance();
    // Set the mode to Admittance Nullspace
    admittance.set_admittance_mode(k_api::Base::AdmittanceMode::NULL_SPACE);
    base->SetAdmittance(admittance);
    return true;
}

// The two following functions are not used in the final teleoperation system
// but it is added her to show that the teleoperation system can be further used
// to execute the action immediately from the button
// this could be helpful in adding the suturing task here and have it executable through
// a button from the hand-controller, immediately after positioning the end-effector
bool example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach Home position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
                create_event_listener_by_promise(finish_promise),
                k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}

bool example_move_to_retract_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Retract")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach Home position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
                create_event_listener_by_promise(finish_promise),
                k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}

// Translation Motion
bool example_cartesian_action_movement(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, float x, float y, float z)
{
    // enable the command to be Twist Command, which allows the robot to move in more than
    // one axis simultaneously
    auto command = k_api::Base::TwistCommand();
    // Set the correct reference frame to be tool so that the command is transformed
    // from the base to the tool. assuring that the RCM is at the tool frame
    // in this case, since movements are in cartesian, a displacement in the tool frame
    // will be equal to the displacement at the end of the end-effector, so no overestimation happens
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    command.set_duration(0);  // Unlimited time to execute the action let the robot decide how long the action will take

    auto twist = command.mutable_twist();
    // the x y and z values are the displacements that the robot will move with
    twist->set_linear_x(x);
    twist->set_linear_y(y);
    twist->set_linear_z(z);
    twist->set_angular_x(0.00f);
    twist->set_angular_y(0.00f);
    twist->set_angular_z(0.00f);
    base->SendTwistCommand(command);

    // Let time for twist to be finished
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Make movement stop
    base->Stop();
    return true;
}

// Rotational Motion
bool example_rotational_action_movement(k_api::Base::BaseClient* base, float x, float y, float z)
{
    auto command = k_api::Base::TwistCommand();
    // in this case, rotation at the tool frame will differ from at the end of the end-effector
    // however, since the displacement is very small (0.5 degrees), the offset will not be significant
    // Nevertheless, while this teleoperation system didnt include the transformation matrix here
    // in general, it should be always considered
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    command.set_duration(0);  // Unlimited time to execute

    auto twist = command.mutable_twist();
    twist->set_linear_x(0.0f);
    twist->set_linear_y(0.0f);
    twist->set_linear_z(0.0f);
    twist->set_angular_x(x);
    twist->set_angular_y(y);
    twist->set_angular_z(z);
    base->SendTwistCommand(command);

    // Let time for twist to be executed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Make movement stop
    base->Stop();
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return true;
}
// ################################################//

// ################################################//
//                CONTROLLER SETTINGS              //
/// setting up the xbox controller and receiving the state of
/// the controller and its values is set here
class Controller
{
private:
    // number of controllers
    int n;
    // state of controller
    XINPUT_STATE state;

public:
    // Values that will be extracted from the robot are added here
    Controller(int num)
    {
        n = num; // the number of controllers attached to the laptop
    }

    XINPUT_STATE GetState()
    {
        ZeroMemory(&state, sizeof(XINPUT_STATE));
        XInputGetState(n, &state); //get the state of the specific controller
        // states includes the values of the joysticks, which buttons are being pressed, how much
        // are they joysticks being rotated, how many buttons are being pressed together, etc
        return state;
    }

    // check if there is a connection between the xbox controller
    // and the laptop
    bool IsConnected()
    {
        ZeroMemory(&state, sizeof(XINPUT_STATE));
        DWORD statenow = XInputGetState(n, &state);

        if(statenow == ERROR_SUCCESS) return true;

        return false;
    }
};
Controller* xone;
// ################################################//

int main(int argc, char **argv)
{
    // connect to the arduino board of the end-effector
    io_service io;
    serial_port port(io, "COM3"); //connect to the correct port where the end-effector is attached
    port.set_option(serial_port_base::baud_rate(9600)); // specify the correct baud rate
    string tool = "2\n"; // in this experiment, the needle holder (2) was used, so this was defined
    string open = "o\n"; // to open the end-effector
    string close = "c\n"; // to close the end-effector
    // call to the arduino board and specify the tool that will be used
    write(port, buffer(tool.c_str(), tool.size()));

    // ################################################//
    //          KINOVA ROBOT INITIALIZATION            //
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username); // admin
    create_session_info.set_password(parsed_args.password); // admin
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
    auto device_manager = new k_api::DeviceManager::DeviceManagerClient(router);
    bool success = true;
    // ################################################//

    // define the controller, '0' denotes player number 1, which is the first controller that is connected
    xone = new Controller(0);
    // check if the controller is connected first, before the system is called
    if(xone->IsConnected())
    {
        std::cout<<"-------------------------------------------"<<std::endl;
        std::cout<<"-------------------------------------------"<<std::endl;
        std::cout<<"Controller Guide:"<<std::endl;
        std::cout<<"A: Admittance Cartesian Mode."<<std::endl;
        std::cout<<"B: Admittance Joint Mode"<<std::endl;
        std::cout<<"Y: Retract Position."<<std::endl;
        std::cout<<"X: Admittance Null Space Mode."<<std::endl;
        std::cout<<"-------------------------------------------"<<std::endl;
        std::cout<<"Right Arrow: Positive Cartesian Y Movement."<<std::endl;
        std::cout<<"Left Arrow: Negative Cartesian Y Movement."<<std::endl;
        std::cout<<"Up Arrow: Positive Cartesian Z Movement."<<std::endl;
        std::cout<<"Down Arrow: Negative Cartesian Z Movement."<<std::endl;
        std::cout<<"Clicking Left Thumb: Negative Cartesian X Movement."<<std::endl;
        std::cout<<"Clicking Right Thumb: Positive Cartesian X Movement."<<std::endl;
        std::cout<<"---------------------------------------------------"<<std::endl;
        std::cout<<"Moving Left Thumb Horizontally: Angular X Movement."<<std::endl;
        std::cout<<"Moving Right Thumb Horizontally: Angular Y Movement."<<std::endl;
        std::cout<<"Moving Right Thumb Vertically: Angular Z Movement."<<std::endl;
        std::cout<<"---------------------------------------------------"<<std::endl;
        std::cout<<"Right Shoulder: Opening End-Effector."<<std::endl;
        std::cout<<"Left Shoulder: Closing End-Effector."<<std::endl;
        std::cout<<"---------------------------------------------------"<<std::endl;
        std::cout<<"Press Right Button Next Logo to Move to Home Position."<<std::endl;
        std::cout<<"Press Left Button Next Logo to EXIT."<<std::endl;
        std::cout<<"Are You Ready? Type Y."<<std::endl;
        char answer;
        char response = 'Y';
        std::cin>>answer;
        if(answer==response)
        {
            std::cout<<"---------------------------------------------------"<<std::endl;
            std::cout<<"You Can Control the Robot Now :D ! \n---------------------------------------------------"<<std::endl;
            while (xone->IsConnected()) {
                // continuously update the current state of the controller, to see the changes that happen
                XINPUT_STATE state;
                DWORD result = XInputGetState(0, &state);

                // get the values of the joysticks
                SHORT L_X = state.Gamepad.sThumbLX;
                SHORT L_Y = state.Gamepad.sThumbLY;
                SHORT R_X = state.Gamepad.sThumbRX;
                SHORT R_Y = state.Gamepad.sThumbRY;

                // define the rotational offsets that will be inputted into the functions
                float moveX = 0.0f;
                float moveY = 0.0f;
                float moveZ = 0.0f;

                // check that the buttons were pressed
                bool Up = (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP) != 0;
                bool Down = (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN) != 0;
                bool Left = (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) != 0;
                bool Right = (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;
                bool LT = (state.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB) != 0;
                bool RT = (state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) != 0;

                // define the cartesian offsets that will be inputted into the functions
                float cartX = 0.0f;
                float cartY = 0.0f;
                float cartZ = 0.0f;

                // link every button pressed to the appropiate motion
                // it is worth mentioning that the x, y, z axes were defined with respect to the user
                // as such, some values were given negative integeres, or some axes were switched, in order
                // to suit the user's perspective and reference
                if (Right) {cartX = 0.0025f;}
                if (Left) {cartX = -0.0025f;}
                if (Up) {cartZ = -0.0025f;}
                if (Down) {cartZ = 0.0025f;}
                if (LT) {cartY = -0.0025f;} // backward
                if (RT) {cartY = 0.0025f;} // forward


                if(cartX != 0.0f || cartY != 0.0f || cartZ != 0.0f) {
                    // send the values to the function
                    success &= example_cartesian_action_movement(base, base_cyclic, cartX, cartY, cartZ);
                    // reset the values so it will not accumulate
                    cartX = 0.0f;
                    cartY = 0.0f;
                    cartZ = 0.0f;
                }

                // a similar action is done for rotational motion. however, the joystick values were
                // filtered to assure that the end-effector will only move when the joystick is rotated fully
                if(L_Y>=32000 || L_Y<=-32000)
                {
                    // Rotational X Movement, forward and backward
                    if (L_Y>=32000) {moveX = -0.5f;}
                    if (L_Y<=-32000) {moveX = 0.5f;}
                }
                else{
                    moveX=0.0f;
                }

                if(R_Y>=32000 || R_Y<=-32000)
                {
                    // Rotational Z Movement, rotating around the long axis of the end-effector
                    if (R_Y>=32000) {moveZ = 0.5f;}
                    if (R_Y<=-32000) {moveZ = -0.5f;}
                }
                else{
                    moveZ=0.0f;
                }

                if(R_X>=32000 || R_X<=-32000)
                {
                    // Rotational Y Movement, rotating sideways
                    if (R_X>=32000) {moveY = -0.5f;}
                    if (R_X<=-32000) {moveY = 0.5f;}
                }
                else{
                    moveY=0.0f;
                }

                if(moveX != 0.0f || moveY != 0.0f || moveZ != 0.0f) {
                    // send all the values to the function
                    success &= example_rotational_action_movement(base, moveX, moveY, moveZ);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_A) {
                    // Admittance Cartesian Mode
                    success &= example_admittance_cartesian(base);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_B) {
                    // Admittance Joint Mode
                    success &= example_admittance_joint(base);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_Y) {
                    /// While the final teleoperation system doesnt execute this
                    /// function, it is kept in order to show how the robot can
                    /// respond to executing a full action from just a press of a button
                    // std::cout << "Moving to Retract Position. \n";
                    //success &= example_move_to_retract_position(base);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_X) {
                    // Admittance NullSpace Mode
                    success &= example_admittance_nullspace(base);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) {
                    // Opening End-Effector
                    write(port, buffer(open.c_str(), open.size()));
                    // so the end-effector will not open more or accidentally
                    // and to allow for the end-effector to finish its movement
                    // the system sleeps for 100 milliseconds where it doesnt take commands
                    Sleep(100);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) {
                    // Closing End-Effector
                    write(port, buffer(close.c_str(), close.size()));
                    // so the end-effector will not open more or accidentally
                    // and to allow for the end-effector to finish its movement
                    // the system sleeps for 100 milliseconds where it doesnt take commands
                    Sleep(100);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_START) {
                    // std::cout << "Moving to Home Position. \n";
                    // success &= example_move_to_home_position(base);
                }

                if (xone->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_BACK) {
                    // exit the teleoperation system and close the API
                    std::cout << "EXIT. \n";
                    break;
                }
            }
        }
    }
    // if the controller wasn't connected, system is paused and exited
    else std::cout<<"Controller Not Found"<<std::endl;
    delete(xone);
    system("pause");

    // ################################################//
    //Close API session
    session_manager->CloseSession();
    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;
    return success? 0: 1;
    // ################################################//

    return 0;
}
