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

#include "can_api.h"
#include "can_btr.h"

#include "CANAPI_Defines.h"
#include "SerialCAN_Defines.h"

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

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
    runnable(node)
{
    this->node = YAML::Clone(node);

    tty_name = get_as<string>(node, "tty_name");
    int tmp_baudrate = get_as<int>(node, "baudrate");
    thread_name = name + "_recv";

    switch (tmp_baudrate) {
        case 1000000:
            baudrate = CANBDR_1000;
            break;
        case 800000:
            baudrate = CANBDR_800;
            break;
        case 500000:
            baudrate = CANBDR_500;
            break;
        case 250000:
            baudrate = CANBDR_250;
            break;
        case 125000:
            baudrate = CANBDR_125;
            break;
        case 100000:
            baudrate = CANBDR_100;
            break;
        case 50000:
            baudrate = CANBDR_50;
            break;
        case 20000:
            baudrate = CANBDR_20;
            break;
        case 10000:
            baudrate = CANBDR_10;
            break;
        default:
            log(warning, "unsupported baudrate %d, switching to 1000 kBit/s\n", tmp_baudrate);
            break;
    }

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

//! second stage init
void slcanmaster::init() {
    auto tmp = get_as<YAML::Node>(node, "trigger", YAML::Node());
    trg = make_shared<triggerable>(tmp, bind(&slcanmaster::tick, this));
}

//! module trigger callback
void slcanmaster::tick() {
    int retval = 0;
    can_message_t msg = { 0 };
    can::frame_t frame;

    // get frames to send
    for (const auto& kv : slave_streams) {
        ssize_t rd = kv.second->read((char *)&frame, sizeof(frame));

        if (rd == 0) {
            continue; // next slave 
        }

        msg.id = frame.hdr;
        msg.xtd = 0;
        msg.rtr = frame.rtr;
        msg.dlc = frame.dlc;
        if (frame.dlc > 0) {
            memcpy(msg.data, frame.data, frame.dlc);
        }

        do {
            retval = can_write(handle, &msg, 0u);
        } while (retval == CANERR_TX_BUSY);

        if (retval < CANERR_NOERROR) {
            if (retval < -100) {
                int _errno = -1 * (retval + 100);
                log(warning, "message could not be send: %s\n", strerror(_errno));
            } else {
                log(warning, "message could not be sent: %d\n", retval);
            }
        }
    }
}

//! State transition from SAFEOP to PREOP
void slcanmaster::set_state_safeop_2_preop() {
    stop();

    for (const auto& kv : slave_streams) {
        auto s = kv.second;
        s->trigger_dev->remove_trigger(trg);
    }

    remove_device(send_trigger);
    send_trigger->remove_trigger(trg);
    send_trigger = nullptr;

    trg->release();

    slave_streams.clear();
}

//! State transition from PREOP to INIT
void slcanmaster::set_state_preop_2_init() {
    (void)can_reset(handle);
    (void)can_exit(handle);
}

//! State transition from INIT to PREOP
void slcanmaster::set_state_init_2_preop() {
    int result;
    can_bitrate_t bitrate;
    can_sio_param_t port;

    port.name = (char*)tty_name.c_str();
    port.attr.protocol = CANSIO_CANABLE; //LAWICEL;
    port.attr.baudrate = CANSIO_BD115200;
    port.attr.bytesize = CANSIO_8DATABITS;
    port.attr.parity = CANSIO_NOPARITY;
    port.attr.stopbits = CANSIO_1STOPBIT;

    log(info, "SerialCAN version: %s\n", can_version());

    if ((handle = can_init(CAN_BOARD(CANLIB_SERIALCAN, CANDEV_SERIAL), CANMODE_DEFAULT, (const void*)&port)) < 0) {
        throw runtime_error(string_printf("interface could not be initialized: %d\n", handle));
    }

    bitrate.index = CANBTR_INDEX_250K;
    if ((result = can_start(handle, &bitrate)) < 0) {
        throw runtime_error(string_printf("interface could not be started: %d", result));
    }
}

//! State transition from PREOP to SAFEOP
void slcanmaster::set_state_preop_2_safeop() {
    for (const auto& stream_name : slave_stream_names) {
        const auto& s = robotkernel::get_device<stream>(stream_name);

        if (!s) {
            throw runtime_error(string_printf(" module not found %s\n", stream_name.c_str()));
        }

        s->trigger_dev->add_trigger(trg);
        slave_streams[stream_name] = s;
    }

    send_trigger = make_shared<trigger>(name, "send");
    send_trigger->add_trigger(trg);
    add_device(send_trigger);

    trg->aquire();
    start();
}

//! run receiver thread 
void slcanmaster::run() {
    int retval = 0;
    can_message_t msg = {};

    while (running()) {
        retval = can_read(handle, &msg, 1000);

        if (retval == CANERR_NOERROR) {
            can::frame_t frame;
            frame.hdr = msg.id;
            frame.rtr = msg.rtr;
            frame.dlc = msg.dlc;
            if (msg.dlc > 0) {
                memcpy(frame.data, msg.data, msg.dlc);
            }

            // process received frame
            for (const auto& kv : slave_streams) {
                if (kv.second->write((char *)&frame, sizeof(frame)))
                    break; // frames should only be processed once
            }
        } else {
            log(warning, "read message returned %i\n", retval);
        }
    }
}

