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

Position_Controller *pc;
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
    CVMArgumentParser argparse(argc, argv, true, false, false, false);
    logFile.open("logfile.csv");

    LocationProcessor lp = LocationProcessor(argparse.calib_file_path, argparse.deviceID, 10, 20);
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
        //imwrite("captured/" + to_string(count) + ".jpg", lp.original);
        logFile << time(0) << ',' << current_position.getInfoString();

        if (!current_position.emptyPosition)
        {
            lastDetectedTime = clock();
            //cout << "current position azi " << -current_position.azi << endl;
            lastyaw = pc->getLastAttitudeYaw();

            float x_new = -current_position.w_z * cos(lastyaw) - current_position.w_x * sin(lastyaw);
            float y_new = -current_position.w_z * sin(lastyaw) + current_position.w_x * cos(lastyaw);

            if (!argparse.quiet)
            {
                cout << "angle of uav " << lastyaw * 180 / 3.14 /*+ -current_position.azi*/ << endl;
                cout << current_position.w_z << " " << current_position.w_x << endl;
                cout << "world_z: " << -current_position.w_y << " z in frame: " << -current_position.y << endl;
            }
            pc->update_current_position(x_new, y_new, -current_position.w_y, lastyaw, startimageCaputre);
            //cout << current_position.w_x << " " << current_position.w_z << " " << -current_position.w_y << endl;
        }

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
        float desired = desired_position.azi + lastyaw * 180 / 3.14;
        if (desired < -180)
        {
            desired = desired + 360;
        }
        else if (desired > 180)
        {
            desired = desired - 360;
        }
        float x_new = -desired_position.z * cos(lastyaw) - desired_position.x * sin(lastyaw);
        float y_new = -desired_position.z * sin(lastyaw) + desired_position.x * cos(lastyaw);

        pc->update_desired_position(x_new, y_new, -desired_position.y, desired * 3.14 / 180);
        lastState = currentState;
        count++;

        if (!argparse.quiet)
        {
            cout << currentState << endl;
            cout << "angle in frame " << desired_position.azi << endl;
            //cout << "desired angle " << -current_position.azi + desired_position.azi << endl;
            cout << "desired angle global " << desired << endl;
            cout << "x_desired_before: " << desired_position.z << "y_desired_before: " << desired_position.x << endl;
            cout << "x_desired_after: " << x_new << "y_desired_after: " << y_new << "z_desired_after" << -desired_position.y << endl;
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
