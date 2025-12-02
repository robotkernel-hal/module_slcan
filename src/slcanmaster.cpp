//! robotkernel module slcanmaster slcanmaster
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of module_slcan.
 *
 * module_slcan is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * module_slcan is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with module_slcan; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "slcanmaster.h"

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

MODULE_DEF(module_slcan, module_slcan::slcanmaster);

using namespace robotkernel;
using namespace module_slcan;
using namespace std;

#define DEBUG_BUFFER_LEN    1024

//! construction
/*!
 * \param node yaml intialization node
 */
slcanmaster::slcanmaster(const std::string& name, const YAML::Node& node) :
    module_base("module_slcan", name, node), 
    trigger_base(node["trigger"])
{
    tty_name = get_as<string>(node, "tty_name");
    baudrate = get_as<int>(node, "baudrate");
   
    if (node["slave_streams"]) {
        // parsing slave configurations
        for (const auto& stream_node : node["slave_streams"]) {
            std::string stream_name = stream_node.as<std::string>();
            slave_stream_names.push_back(stream_name); 
        }
    }
}

//! destruction 
slcanmaster::~slcanmaster() {
}

//! module trigger callback
void slcanmaster::tick() {
    CANAPI_Message_t message;

    // get frames to send
    for (const auto& kv : slave_streams) {
        can::frame_t frame;
        ssize_t rd = kv.second->read((char *)&frame, sizeof(frame));

        if (rd == 0) {
            continue; // next slave 
        }

        message.id = frame.hdr;
        message.xtd = 0;
        message.rtr = frame.rtr;
        message.dlc = frame.dlc;
        if (frame.dlc > 0)
            memcpy(message.data, frame.data, frame.dlc);

        CANAPI_Return_t retval;
        if ((retval = serial_can.WriteMessage(message)) != CSerialCAN::NoError) {
            log(warning, "message could not be sent!\n");
        }
    }
}

//! State transition from SAFEOP to PREOP
void slcanmaster::set_state_safeop_2_preop() {
    stop();

    slave_streams.clear();
}

//! State transition from PREOP to INIT
void slcanmaster::set_state_preop_2_init() {
    CANAPI_Return_t retval;
    if ((retval = serial_can.TeardownChannel()) != CSerialCAN::NoError) {
        log(error, "error: interface could not be shutdown\n");
    }
}

//! State transition from INIT to PREOP
void slcanmaster::set_state_init_2_preop() {
    CANAPI_Return_t retval = 0;
    CANAPI_OpMode_t opmode = {};
    opmode.byte = CANMODE_DEFAULT;
    CANAPI_Bitrate_t bitrate = {};
    bitrate.index = baudrate;

    if ((retval = serial_can.InitializeChannel(tty_name.c_str(), opmode)) != CSerialCAN::NoError) {
        throw runtime_error("error: interface could not be initialized");
    }
    
    if ((retval = serial_can.StartController(bitrate)) != CSerialCAN::NoError) {
        (void)serial_can.TeardownChannel();
        throw runtime_error("error: interface could not be started");
    }
}

//! State transition from PREOP to SAFEOP
void slcanmaster::set_state_preop_2_safeop() {
    for (const auto& stream_name : slave_stream_names) {
        const auto& s = robotkernel::get_device<stream>(stream_name);

        if (!s) {
            throw runtime_error(string_printf(" module not found %s\n", stream_name.c_str()));
        }

        slave_streams[stream_name] = s;
    }

    start();
}

//! run receiver thread 
void slcanmaster::run() {
    CANAPI_Return_t retval;
    CANAPI_Message_t message;

    while (running()) {
        if ((retval = serial_can.ReadMessage(message, CANREAD_INFINITE)) == CSerialCAN::NoError) {
            can::frame_t frame;
            frame.hdr = message.id;
            frame.rtr = message.rtr;
            frame.dlc = message.dlc;
            if (message.dlc > 0) {
                memcpy(frame.data, message.data, message.dlc);
            }

            // process received frame
            for (const auto& kv : slave_streams) {
                if (kv.second->write((char *)&frame, sizeof(frame)))
                    break; // frames should only be processed once
            }
        } else if (retval != CSerialCAN::ReceiverEmpty) {
            log(warning, "read message returned %i", retval);
        }
    }
}

