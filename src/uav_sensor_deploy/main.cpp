#include <common/cvm_argument_parser.hpp>
#include <common/navigational_state.hpp>
#include <common/location_processor.hpp>
#include <common/position.hpp>
#include <thread>
#include <common/msg_queue.hpp>
#include <mavlink-interface/position_controller.h>
#include <mavlink-interface/multithreaded_interface.h>
#include "states.hpp"
#include <iostream>
#include <fstream>
#include <ctime>
#include "attitude_controller.h"

#define DEFAULT_SPEED_UP 0.20 //in m/s. (10cm/s)

bool shutdown = false;
clock_t startimageCaputre = clock();
clock_t lastDetectedTime = clock();
clock_t startedDataAcquisition = clock();
time_t t;

NavigationalState<State> *ap = new AutoPilotState();
NavigationalState<State> *na = new NormalApproach();
NavigationalState<State> *da = new DataAcquisitionState();
NavigationalState<State> *po = new PullOutState();

NavigationalState<State> *xt = new CrossTest();
NavigationalState<State> *ct = new CircleTest();
NavigationalState<State> *ph = new PositionHold();

Position_Controller *pc;
AttitudeController *ac;
Serial_Port serial_port;

char enable_magnet = 'e';
char release_magnet = 'r';

bool running = true;
ofstream vibration_file;
ofstream logFile;

thread read_th;

void reader_thread()
{
    bool success;
    vibration_file.open("vib_measure.csv");

    uint8_t nKByte = 0;

    while (running)
    {
        success = serial_port._read_port(nKByte);
        if (success)
        {
            vibration_file << nKByte;
        }
    }
    vibration_file.close();
}

void start_reader_thread()
{
    read_th = thread([=]() {
        reader_thread();
    });
}

int main(int argc, const char **argv)
{
    cout << std::fixed << std::showpoint;
    cout << std::setprecision(3);

    CVMArgumentParser argparse(argc, argv, true, false, false, false);
    logFile.open("logfile.csv");

    LocationProcessor lp = LocationProcessor(argparse.calib_file_path, argparse.deviceID, 15, 20);
    NavigationalState<State> *ns = ap;

    Multithreaded_Interface mti;
    mti.start("/dev/ttyS1", 57600);

    /*
    serial_port.uart_name = "/dev/ttyACM0";
    serial_port.baudrate = 57600;
    serial_port.start();
    start_reader_thread();
    */
   
    pc = new Position_Controller(&mti);
    ac = new AttitudeController;


    int count = 0;

    Position current_position;
    Position desired_position;
    State lastState = AP;
    State currentState = AP;

    bool firstDA = true;
    bool firstPostDA = false;
    float lastyaw = 0;
    auto start = std::chrono::high_resolution_clock::now();
    double total_time = 0;
    int downsample = 0;

    while (true)
    {
        startimageCaputre = clock();
        current_position = lp.processImage();

	    if (argparse.saveVideo){
            imwrite("captured/" + to_string(count) + ".jpg", lp.original);
            imwrite("captured/" + to_string(count) + "marked.jpg", lp.arProc.drawMarkersAndAxis(lp.original));
        }        

        /*
        if (!current_position.emptyPosition)
        {
            lastDetectedTime = clock();
            //cout << "current position azi " << -current_position.azi << endl;
            lastyaw = pc->getLastAttitudeYaw();
            
            if (!argparse.quiet)
            {
                cout << "angle of uav " << lastyaw * 180 / 3.14 << endl;
                cout << current_position.w_z << " " << current_position.w_x << endl;
                cout << "world_z: " << -current_position.w_y << " z in frame: " << -current_position.y << endl;
            }

            //pc->update_current_position(x_new, y_new, -current_position.w_y, lastyaw, startimageCaputre);
            //cout << current_position.w_x << " " << current_position.w_z << " " << -current_position.w_y << endl;
        }
        */
        
        ns = ns->returnNextState(current_position);
        currentState = ns->currentState();
        
        /*
        downsample++;
        if (currentState == DA)
        {   
            firstPostDA = true;
            if (downsample == 10){
                serial_port._write_port(&enable_magnet, 1);
                downsample = 0;
            }
                
            if (firstDA)
            {
                vibration_file << "NEW VIBRATION" << endl
                               << "NEW VIBRATION" << endl
                               << "NEW VIBRATION" << endl;
                firstDA = false;
            }
        }
        else
        {
            if (firstPostDA){
                serial_port._write_port(&release_magnet, 1);
                firstPostDA = false;
                downsample = 10;
            }
            if (downsample == 10){
                serial_port._write_port(&release_magnet, 1);
                downsample = 0;
            }
                
        }
        */


        /*
        if (currentState == AP && lastState != AP)
            pc->toggle_offboard_control(false);
        else if (lastState == AP && currentState != AP)
            pc->toggle_offboard_control(true);
        */

       

        desired_position = ns->computeDesiredPosition(current_position);

        ac->run_loop({current_position.w_x, current_position.w_y, current_position.w_z}, {desired_position.x, desired_position.y, desired_position.z});

        ac->acceleration_to_attitude(-ac->acc_desi.z, ac->acc_desi.x, -current_position.azi);

        float rel_vert_vel = (ac->get_desired_velocity().y/DEFAULT_SPEED_UP);

        float yaw_rate = -current_position.angle_in_frame() * 0.4;

        if (!ac->reinitialize_state){
            pc->update_attitude_target(ac->pitch_target, ac->roll_target, ac->yaw_target, rel_vert_vel, yaw_rate, true);
        }
        
        lastState = currentState;
        logFile << time(0) << ',' << ac->get_state_string() << endl;
        count++;

        if (!argparse.quiet)
        {
            //cout << ac->get_state_string() << endl << endl;
            cout << ac->forward_acc << ',' << ac->right_acc << ',   ' << ' ';

            cout << current_position.azi << ',   ' << ' '; 

            cout << ac->rot_forward_acc << ',' << ac->rot_right_acc << ',   ' << ' ';
            
            cout << yaw_rate << ',' << rel_vert_vel << endl << endl;
            /*
            cout << currentState << endl;
            cout << "angle in frame " << desired_position.azi << endl;
            cout << "current angle global " << desired << endl;
            cout << "x_current: " << desired_position.z << "y_current: " << desired_position.x << endl;

            //cout << "desired angle " << -current_position.azi + desired_position.azi << endl;
            cout << "desired angle global " << desired << endl;
            cout << "x_desired_before: " << desired_position.z << "y_desired_before: " << desired_position.x << endl;
            cout << "x_desired_after: " << x_new << "y_desired_after: " << y_new << "z_desired_after" << -desired_position.y << endl;
            */
        }
        if (argparse.saveTiming)
        {
            std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;
            total_time += elapsed.count();
            std::cout << "Elapsed time: " << elapsed.count() << " s Averge time: " << total_time / count << "s\n";

            start = std::chrono::high_resolution_clock::now();
        }
    }
    mti.shutdown();
    logFile.close();
    running = false;
    return -1;
}
