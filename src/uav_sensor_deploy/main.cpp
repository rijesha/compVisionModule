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

bool shutdown = false;
clock_t lastDetectedTime = clock();
clock_t startedDataAcquisition = clock();

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

    LocationProcessor lp = LocationProcessor(argparse.calib_file_path, argparse.deviceID);
    NavigationalState<State> *ns = ap;

    Multithreaded_Interface mti;
    mti.start("/dev/ttyS1", 57600);

    serial_port.uart_name = "/dev/ttyACM0";
    serial_port.baudrate = 57600;
    serial_port.start();

    start_reader_thread();
    pc = new Position_Controller(&mti);

    int count = 0;

    Position current_position;
    Position desired_position;
    State lastState = AP;
    State currentState = AP;

    bool firstDA = true;

    while (true)
    {
        current_position = lp.processImage();
        imwrite("captured/" + to_string(count) + ".png", lp.original);
        logFile << current_position.getInfoString();

        if (!current_position.emptyPosition)
        {
            lastDetectedTime = clock();
            pc->update_current_position(current_position.w_x, current_position.w_z, -current_position.w_y, -current_position.azi * 3.14 / 180);
            //cout << current_position.w_x << " " << current_position.w_z << " " << -current_position.w_y << endl;
        }

        ns = ns->returnNextState(current_position);
        currentState = ns->currentState();

        if (currentState == DA)
        {
            serial_port._write_port(&enable_magnet, 1);
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
            serial_port._write_port(&release_magnet, 1);
        }

        if (currentState == AP && lastState != AP)
            pc->toggle_offboard_control(false);
        else if (lastState == AP && currentState != AP)
            pc->toggle_offboard_control(true);

        desired_position = ns->computeDesiredPosition(current_position);
        pc->update_desired_position(desired_position.x, desired_position.z, -desired_position.y, desired_position.azi * 3.14 / 180);

        lastState = ns->currentState();
        count++;
    }
    mti.shutdown();
    logFile.close();
    running = false;
    return -1;
}
