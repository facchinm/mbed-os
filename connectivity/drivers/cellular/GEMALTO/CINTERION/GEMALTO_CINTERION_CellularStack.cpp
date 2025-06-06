/*
 * Copyright (c) 2018, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>
#include "GEMALTO_CINTERION_CellularStack.h"
#include "GEMALTO_CINTERION.h"
#include "CellularLog.h"
#include "rtos.h"

using namespace std::chrono_literals;

// defines as per ELS61-E2_ATC_V01.000 and BGS2-W_ATC_V00.100
#define UDP_PACKET_SIZE 1460
#define FAILURE_TIMEOUT 30s // failure timeout on modem side

using namespace mbed;

GEMALTO_CINTERION_CellularStack::GEMALTO_CINTERION_CellularStack(ATHandler &atHandler, const char *apn, const char *user, const char *password,
                                                                 int cid, nsapi_ip_stack_t stack_type, AT_CellularDevice &device) :
    AT_CellularStack(atHandler, cid, stack_type, device), _apn(apn), _user(user), _password(password)
{
}

GEMALTO_CINTERION_CellularStack::~GEMALTO_CINTERION_CellularStack()
{
    _at.set_urc_handler("^SIS:", nullptr);
    _at.set_urc_handler("^SISW:", nullptr);
    _at.set_urc_handler("^SISR:", nullptr);
}

void GEMALTO_CINTERION_CellularStack::urc_sis()
{
    int sock_id = _at.read_int();
    int urc_code = _at.read_int();
    CellularSocket *sock = find_socket(sock_id);
    if (sock) {
        // Currently only UDP is supported so there is need to handle only some error codes here,
        // and others are detected on sendto/recvfrom responses.
        if (urc_code == 5) { // The service is ready to use (ELS61 and EMS31).
            if (sock->_cb) {
                sock->started = true;
                sock->tx_ready = true;
                sock->_cb(sock->_data);
            }
        }
        if (urc_code == 0) {
            int urc_info_id = _at.read_int();
            if (urc_info_id == 48) {
                tr_info("Socket closed %d", sock_id);
                sock->closed = true;
                if (sock->_cb) {
                    sock->_cb(sock->_data);
                }

            }
        }
    }
}

void GEMALTO_CINTERION_CellularStack::urc_sisw()
{
    int sock_id = _at.read_int();
    int urc_code = _at.read_int();
    sisw_urc_handler(sock_id, urc_code);
}

void GEMALTO_CINTERION_CellularStack::sisw_urc_handler(int sock_id, int urc_code)
{
    CellularSocket *sock = find_socket(sock_id);
    if (sock) {
        if (urc_code == 1) { // ready
            if (sock->_cb) {
                sock->tx_ready = true;
                if (sock->proto == NSAPI_TCP || GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleBGS2) {
                    sock->started = true;
                }
                sock->_cb(sock->_data);
            }
        }
    }
}

void GEMALTO_CINTERION_CellularStack::urc_sisr()
{
    int sock_id = _at.read_int();
    int urc_code = _at.read_int();
    sisr_urc_handler(sock_id, urc_code);
}

void GEMALTO_CINTERION_CellularStack::urc_sysstart()
{
    // close sockets if open
    _at.lock();
    for (int i = 0; i < _device.get_property(AT_CellularDevice::PROPERTY_SOCKET_COUNT); i++) {
        _at.clear_error();
        socket_close_impl(i);
    }
    _at.clear_error();
    _at.unlock();
}

void GEMALTO_CINTERION_CellularStack::sisr_urc_handler(int sock_id, int urc_code)
{
    CellularSocket *sock = find_socket(sock_id);
    if (sock) {
        if (urc_code > 0) { // data available
            if (sock->_cb) {
                sock->pending_bytes = urc_code;
                sock->_cb(sock->_data);
            }
        }
    }
}


void GEMALTO_CINTERION_CellularStack::lock() {
    _at.lock();
}

void GEMALTO_CINTERION_CellularStack::unlock() {
    _at.unlock();
}

void GEMALTO_CINTERION_CellularStack::urc_gnss() {
    char gnss_string[100] = {'$', 'G'};
    if (_gnss_cb) {
        _at.set_delimiter('\n');
        _at.read_string(&gnss_string[2], 98);
        _at.set_default_delimiter();
        _gnss_cb(gnss_string);
    }
}

void GEMALTO_CINTERION_CellularStack::beginGNSS(mbed::Callback<void(char*)> gnss_cb) {
    _at.lock();
    _gnss_cb = gnss_cb;
    _at.at_cmd_discard("^SGPSC", "=", "%s%d", "Engine/StartMode", 0);
    _at.at_cmd_discard("^SGPSC", "=", "%s%d", "Engine", 0);
    _at.at_cmd_discard("^SGPSC", "=", "%s%s", "Nmea/Urc", "off");
    _at.at_cmd_discard("^SPIO", "=", "%d", 1);
    _at.at_cmd_discard("^SCPIN", "=", "%d%d%d%d", 1, 7, 1, 0);
    _at.clear_error();
    _at.unlock();
}

void GEMALTO_CINTERION_CellularStack::endGNSS() {
    _at.lock();
    _at.at_cmd_discard("^SSIO", "=", "%d%d", 7, 0);
    _gnss_cb = nullptr;
    _at.clear_error();
    _at.unlock();
}

void GEMALTO_CINTERION_CellularStack::enableCmux()
{
    _at.at_cmd_discard("+CMUX", "=0");
}

int GEMALTO_CINTERION_CellularStack::startGNSS() {
    _at.lock();
    _engine = false;
    _at.at_cmd_discard("^SSIO", "=", "%d%d", 7, 1);
    _at.cmd_start_stop("^SGPSC", "=", "%s%d", "Engine", 3);
    _at.resp_start("^SGPSC: \"Engine\",");

    char respEng[2];
    int resp_len = _at.read_string(respEng, sizeof(respEng));
    if (strcmp(respEng, "3") != 0) {
        _engine = false;
        _at.at_cmd_discard("^SGPSC", "=", "%s%d", "Engine", 0);
        _at.at_cmd_discard("^SGPSC", "=", "%s%s", "Nmea/Urc", "off");
        return 0;
    }
    _engine = true;
    _at.at_cmd_discard("^SGPSC", "=", "%s%s", "Nmea/Urc", "on");
    _at.clear_error();
    _at.unlock();
    return 1;
}

void GEMALTO_CINTERION_CellularStack::stopGNSS() {
    if(_engine) {
        _at.lock();
        _at.at_cmd_discard("^SGPSC", "=", "%s%s", "Nmea/Urc", "off");
        _at.at_cmd_discard("^SGPSC", "=", "%s%d", "Engine", 0);
        _at.clear_error();
        _at.unlock();
        _engine = false;
    }
}

void GEMALTO_CINTERION_CellularStack::PSMEnable() {
    if(_engine) {
        _at.lock();
        _at.at_cmd_discard("^SGPSC", "=", "%s%d", "Power/Psm", 1);
        _at.clear_error();
        _at.unlock();
    }
}

void GEMALTO_CINTERION_CellularStack::PSMDisable() {
    if(_engine) {
        _at.lock();
        _at.at_cmd_discard("^SGPSC", "=", "%s%d", "Power/Psm", 0);
        _at.clear_error();
        _at.unlock();
    }
}

nsapi_error_t GEMALTO_CINTERION_CellularStack::socket_stack_init()
{
    _at.lock();
    nsapi_error_t err = create_connection_profile(_cid);
    if (!err) {
        _at.set_urc_handler("^SIS:", mbed::Callback<void()>(this, &GEMALTO_CINTERION_CellularStack::urc_sis));
        _at.set_urc_handler("^SISW:", mbed::Callback<void()>(this, &GEMALTO_CINTERION_CellularStack::urc_sisw));
        _at.set_urc_handler("^SISR:", mbed::Callback<void()>(this, &GEMALTO_CINTERION_CellularStack::urc_sisr));
        _at.set_urc_handler("^SYSSTART", mbed::Callback<void()>(this, &GEMALTO_CINTERION_CellularStack::urc_sysstart));
        _at.set_urc_handler("^SGPSE", mbed::Callback<void()>(this, &GEMALTO_CINTERION_CellularStack::urc_gnss));
        _at.set_urc_handler("$G", mbed::Callback<void()>(this, &GEMALTO_CINTERION_CellularStack::urc_gnss));
    } else { // recovery cleanup
        // close all Internet and connection profiles
        for (int i = 0; i < _device.get_property(AT_CellularDevice::PROPERTY_SOCKET_COUNT); i++) {
            _at.clear_error();
            socket_close_impl(i);
        }
        _at.clear_error();
        close_connection_profile(_cid);
    }
    _at.unlock();
    return err;
}

nsapi_error_t GEMALTO_CINTERION_CellularStack::socket_close_impl(int sock_id)
{
    tr_debug("Cinterion close %d", sock_id);

    _at.set_at_timeout(FAILURE_TIMEOUT);

    _at.at_cmd_discard("^SISC", "=", "%d", sock_id);

    _at.clear_error(); // clear SISS even though SISC fails

    _at.restore_at_timeout();

    return _at.get_last_error();
}

#ifdef MBED_CONF_CELLULAR_OFFLOAD_DNS_QUERIES
nsapi_error_t GEMALTO_CINTERION_CellularStack::gethostbyname(const char *host, SocketAddress *address,
                                                        nsapi_version_t version, const char *interface_name)
{
    (void) interface_name;
    MBED_ASSERT(host);
    MBED_ASSERT(address);

    _at.lock();

    if (_dns_callback) {
        _at.unlock();
        return NSAPI_ERROR_BUSY;
    }

    if (!address->set_ip_address(host)) {
        // _at.set_at_timeout(1min);
        _at.cmd_start_stop("^SISX", "=", "%s%d%s", "HostByName", _cid, host);
        _at.resp_start("^SISX: \"HostByName\",");
        char ipAddress[NSAPI_IP_SIZE];
        int size = _at.read_string(ipAddress, sizeof(ipAddress));
        _at.resp_stop();
        if (size > 0) {
            //Valid string received
            tr_info("Read %d bytes. Valid string: %s\n", size, ipAddress);
            // _at.restore_at_timeout();
            if (!address->set_ip_address(ipAddress)) {
                _at.unlock();
                return NSAPI_ERROR_DNS_FAILURE;
            }
        } else {
            //Null string received
            tr_info("Read %d bytes. Null string\n", size);
            return NSAPI_ERROR_NO_ADDRESS;
        }
    }

    return _at.unlock_return_error();
}
#endif

nsapi_error_t GEMALTO_CINTERION_CellularStack::socket_open_defer(CellularSocket *socket, const SocketAddress *address)
{
    int retry_open = 1;
retry_open:
    // setup internet session profile
    int internet_service_id = find_socket_index(socket);
    bool foundSrvType = false;
    bool foundConIdType = false;

    _at.cmd_start_stop("^SISS", "?");
    _at.resp_start("^SISS:");

    /*
     * Profile is a list of tag-value map:
     * ^SISS: <srvProfileId>, <srvParmTag>, <srvParmValue>
     * [^SISS: ...]
     */
    while (_at.info_resp()) {
        int id = _at.read_int();
        if (id == internet_service_id) {
            char paramTag[16];
            int paramTagLen = _at.read_string(paramTag, sizeof(paramTag));
            if (paramTagLen > 0) {
                char paramValue[100 + 1]; // APN may be up to 100 chars
                int paramValueLen = _at.read_string(paramValue, sizeof(paramValue));
                if (paramValueLen >= 0) {
                    if (strcmp(paramTag, "srvType") == 0) {
                        if (strcmp(paramValue, "Socket") == 0) {
                            foundSrvType = true;
                        }
                    }
                    if (strcmp(paramTag, "address") == 0) {
                        if (strncmp(paramValue, "sock", sizeof("sock")) == 0) {
                            foundSrvType = true;
                        }
                    }
                    if (strcmp(paramTag, "conId") == 0) {
                        char buf[10];
                        std::sprintf(buf, "%d", _cid);
                        if (strcmp(paramValue, buf) == 0) {
                            foundConIdType = true;
                        }
                    }
                }
            }
        }
    }
    _at.resp_stop();

    if (!foundSrvType) {
        _at.at_cmd_discard("^SISS", "=", "%d%s%s", internet_service_id, "srvType", "Socket");
    }

    if (!foundConIdType) {
        _at.at_cmd_discard("^SISS", "=", "%d%s%d", internet_service_id, "conId", _cid);
    }

    // host address (IPv4) and local+remote port is needed only for BGS2 which does not support UDP server socket
    char sock_addr[sizeof("sockudp://") - 1 + NSAPI_IPv6_SIZE + sizeof("[]:12345;port=12345") - 1 + 1];

    if (socket->proto == NSAPI_UDP) {
        if (GEMALTO_CINTERION::get_module() != GEMALTO_CINTERION::ModuleBGS2) {
            std::sprintf(sock_addr, "sockudp://%s:%u", address ? address->get_ip_address() : "", socket->localAddress.get_port());
        } else {
            std::sprintf(sock_addr, "sockudp://%s:%u;port=%u", address->get_ip_address(), address->get_port(), socket->localAddress.get_port());
        }
    } else {
        if (address->get_ip_version() == NSAPI_IPv4) {
            std::sprintf(sock_addr, "socktcp://%s:%u", address->get_ip_address(), address->get_port());
        } else {
            std::sprintf(sock_addr, "socktcp://[%s]:%u", address->get_ip_address(), address->get_port());
        }
    }

    _at.cmd_start("AT^SISS=");
    _at.write_int(internet_service_id);
    _at.write_string("address", false);
    _at.write_string(sock_addr);
    _at.cmd_stop_read_resp();

    _at.at_cmd_discard("^SISO", "=", "%d", internet_service_id);

    if (_at.get_last_error()) {
        tr_error("Socket %d open failed!", internet_service_id);
        _at.clear_error();
        socket_close_impl(internet_service_id); // socket may already be open on modem if app and modem are not in sync, as a recovery, try to close the socket so open succeeds the next time
        if (retry_open--) {
            goto retry_open;
        }
        return NSAPI_ERROR_NO_SOCKET;
    }

    socket->id = internet_service_id;
    tr_debug("Cinterion open %d (err %d)", socket->id, _at.get_last_error());

    return _at.get_last_error();
}

// To open socket:
// 1. Select URC mode or polling mode with AT^SCFG
// 2. create a GPRS connection profile with AT^SICS (must have PDP)
// 3. create service profile with AT^SISS and map connectionID to serviceID
// 4. open internet session with AT^SISO (ELS61 tries to attach to a packet domain)
nsapi_error_t GEMALTO_CINTERION_CellularStack::create_socket_impl(CellularSocket *socket)
{
    if (socket->proto == NSAPI_UDP) {
        if (GEMALTO_CINTERION::get_module() != GEMALTO_CINTERION::ModuleBGS2) {
            return socket_open_defer(socket);
        }
    }

    return _at.get_last_error();
}

nsapi_size_or_error_t GEMALTO_CINTERION_CellularStack::socket_sendto_impl(CellularSocket *socket,
                                                                          const SocketAddress &address, const void *data, nsapi_size_t size)
{
    if (socket->proto == NSAPI_UDP) {
        const int ip_version = address.get_ip_version();
        if (_stack_type != IPV4V6_STACK &&
                ((ip_version == NSAPI_IPv4 && _stack_type != IPV4_STACK) ||
                 (ip_version == NSAPI_IPv6 && _stack_type != IPV6_STACK))) {
            tr_warn("No IP route for %s", address.get_ip_address());
            return NSAPI_ERROR_NO_SOCKET;
        }
    }

    if (socket->proto == NSAPI_UDP && GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleBGS2) {
        tr_debug("Send addr %s, prev addr %s", address.get_ip_address(), socket->remoteAddress.get_ip_address());
        if (address != socket->remoteAddress) {
            if (socket->started) {
                socket_close_impl(socket->id);
                _at.clear_error();
            }

            if (create_socket_impl(socket) != NSAPI_ERROR_OK) {
                tr_error("Failed to create socket %d", socket->id);
                return NSAPI_ERROR_NO_SOCKET;
            }
            if (socket_open_defer(socket, &address) != NSAPI_ERROR_OK) {
                tr_error("Failed to open socket %d", socket->id);
                return NSAPI_ERROR_NO_SOCKET;
            }
            socket->remoteAddress = address;
            _at.resp_start("^SISW:");
            int sock_id = _at.read_int();
            MBED_ASSERT(sock_id == socket->id);
            int urc_code = _at.read_int();
            tr_debug("TX ready: socket=%d, urc=%d (err=%d)", sock_id, urc_code, _at.get_last_error());
            (void)sock_id;
            (void)urc_code;
            socket->started = true;
            socket->tx_ready = true;
        }
    }
    if (!socket->started || !socket->tx_ready) {
        tr_debug("Socket %d send would block (started %d, tx %d)", socket->id, socket->started, socket->tx_ready);
        return NSAPI_ERROR_WOULD_BLOCK;
    }

    if (size > UDP_PACKET_SIZE) {
        tr_error("sendto size %d (max %d)", size, UDP_PACKET_SIZE);
        return NSAPI_ERROR_PARAMETER;
    }

    _at.set_at_timeout(FAILURE_TIMEOUT);

    if (GEMALTO_CINTERION::get_module() != GEMALTO_CINTERION::ModuleBGS2) {
        // UDP requires Udp_RemClient
        if (socket->proto == NSAPI_UDP) {
            char socket_address[NSAPI_IPv6_SIZE + sizeof("[]:12345") - 1 + 1];
            if (address.get_ip_version() == NSAPI_IPv4) {
                std::sprintf(socket_address, "%s:%u", address.get_ip_address(), address.get_port());
            } else {
                std::sprintf(socket_address, "[%s]:%u", address.get_ip_address(), address.get_port());
            }
            _at.cmd_start_stop("^SISW", "=", "%d%d%d%s", socket->id, size, 0, socket_address);
        } else {
            _at.cmd_start_stop("^SISW", "=", "%d%d%d", socket->id, size, 0);
        }
    } else {
        _at.cmd_start_stop("^SISW", "=", "%d%d", socket->id, size);
    }

sisw_retry:
    _at.resp_start("^SISW:");
    if (!_at.info_resp()) {
        tr_error("Socket %d send failure", socket->id);
        _at.restore_at_timeout();
        return NSAPI_ERROR_DEVICE_ERROR;
    }
    int socket_id = _at.read_int();
    if (socket_id != socket->id) {
        // We might have read the SISW URC so let's try to handle it
        const int urc_code = _at.read_int();
        const int extra = _at.read_int();
        if (urc_code != -1 && extra == -1) {
            sisw_urc_handler(socket_id, urc_code);
            goto sisw_retry;
        }
        _at.restore_at_timeout();
        tr_error("Socket id %d != %d", socket_id, socket->id);
        return NSAPI_ERROR_DEVICE_ERROR;
    }
    int accept_len = _at.read_int();
    if (accept_len == -1) {
        tr_error("Socket %d send failed", socket->id);
        _at.restore_at_timeout();
        return NSAPI_ERROR_DEVICE_ERROR;
    }
    _at.skip_param(); // unackData

    _at.write_bytes((uint8_t *)data, accept_len);
    _at.resp_stop();
    _at.restore_at_timeout();

    if (_at.get_last_error() == NSAPI_ERROR_OK) {
        socket->tx_ready = false;
    }

    return (_at.get_last_error() == NSAPI_ERROR_OK) ? accept_len : NSAPI_ERROR_DEVICE_ERROR;
}

#define DISABLE_URCs _at.at_cmd_discard("^SCFG", "=", "%s%s","Tcp/WithURCs","off")
#define RESTORE_URCs_AND_RETURN(ret) do { _at.at_cmd_discard("^SCFG", "=", "%s%s","Tcp/WithURCs","on"); return ret; } while(0)

nsapi_size_or_error_t GEMALTO_CINTERION_CellularStack::socket_recvfrom_impl(CellularSocket *socket, SocketAddress *address,
                                                                            void *buffer, nsapi_size_t size)
{
    // AT_CellularStack::recvfrom(...) will make sure that we do have a socket
    // open on the modem, assert here to catch a programming error
    MBED_ASSERT(socket->id != -1);

    DISABLE_URCs;

    // we must use this flag, otherwise ^SISR URC can come while we are reading response and there is
    // no way to detect if that is really an URC or response
    if (!socket->pending_bytes) {
        _at.process_oob(); // check for ^SISR URC
        if (!socket->pending_bytes) {
            tr_debug("Socket %d recv would block", socket->id);
            RESTORE_URCs_AND_RETURN(NSAPI_ERROR_WOULD_BLOCK);
        }
    }

    if (size > UDP_PACKET_SIZE) {
        size = UDP_PACKET_SIZE;
    }

    tr_info("requesting %d bytes\n", size);
    _at.cmd_start_stop("^SISR", "=", "%d%d", socket->id, size);

sisr_retry:
    _at.resp_start("^SISR:");
    if (!_at.info_resp()) {
        tr_error("Socket %d not responding", socket->id);
        RESTORE_URCs_AND_RETURN(NSAPI_ERROR_DEVICE_ERROR);
    }

    int socket_id = _at.read_int();
    if (socket_id != socket->id) {
        const int urc_code = _at.read_int();
        const int extra = _at.read_int(); // should be -1 if URC
        if (urc_code != -1 && extra == -1) {
            sisr_urc_handler(socket_id, urc_code);
            goto sisr_retry;
        }
        tr_error("Socket recvfrom id %d != %d", socket_id, socket->id);
        RESTORE_URCs_AND_RETURN(NSAPI_ERROR_DEVICE_ERROR);
    }

    nsapi_size_or_error_t len = _at.read_int();
    if (len == 0) {
        tr_warn("Socket %d no data", socket->id);
        _at.resp_stop();
        RESTORE_URCs_AND_RETURN(NSAPI_ERROR_WOULD_BLOCK);
    }
    if (len == -1) {
        if (GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleTX62 && _at.get_last_read_error() == -2) {
            _at.process_oob();
            tr_error("Socket %d recvfrom finished!", socket->id);
            socket->pending_bytes = 0;
            RESTORE_URCs_AND_RETURN(NSAPI_ERROR_OK);
        }
        tr_error("Socket %d recvfrom failed!", socket->id);
        RESTORE_URCs_AND_RETURN(NSAPI_ERROR_DEVICE_ERROR);
    }
    if (len >= (nsapi_size_or_error_t)size) {
        len = (nsapi_size_or_error_t)size;
    }

    // UDP Udp_RemClient
    if (socket->proto == NSAPI_UDP && GEMALTO_CINTERION::get_module() != GEMALTO_CINTERION::ModuleBGS2) {

        size_t ip_address_len = NSAPI_IPv6_SIZE + sizeof("[]:12345") - 1 + 1;
        char ip_address[ip_address_len];

        if (GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleTX62) {
            // Local buffer for parsing Udp_RemClient for TX62
            uint8_t at_buf[ip_address_len];
            size_t ip_len = 0;

            // Skip <remainUdpPacketLength>
            nsapi_size_or_error_t rem_len = _at.read_int();

            // Wait for full <Udp_RemClient> in the _at buffer
            do {
                int len = _at.read_bytes(at_buf + ip_len, 1);
                if (len <= 0) {
                    tr_error("Socket %d recvfrom addr (len %d)", socket->id, ip_len);
                    RESTORE_URCs_AND_RETURN(NSAPI_ERROR_DEVICE_ERROR);
                }
                ip_len += len;
            } while (ip_len < ip_address_len && at_buf[ip_len - 2] != '\r' && at_buf[ip_len - 1] != '\n');

            // if (ip_len < sizeof("0.0.0.0:0")) {
            if (ip_len < sizeof("[]:0")) {
                tr_error("Socket %d has no address", socket->id);
                goto sisr_retry;
            }

            // at_buf contains remote client IP information
            // in the format "<ip address>:<port>"\r\n.

            // Terminate the C string at the closing quotation mark
            at_buf[ip_len - 3] = '\0';
            // Skip the opening quotation mark
            memcpy(ip_address, at_buf + 1, ip_len - 4);
            tr_info("ip_address %s (%d)", ip_address, ip_len - 4);
        } else {
            int ip_len = _at.read_string(ip_address, sizeof(ip_address));
            if (ip_len <= 0) {
                tr_error("Socket %d recvfrom addr (len %d)", socket->id, ip_len);
                RESTORE_URCs_AND_RETURN(NSAPI_ERROR_DEVICE_ERROR);
            }
        }

        if (address) {
            char *ip_start = ip_address;
            char *ip_stop;
            char *port_start;
            if (_stack_type == IPV6_STACK) {
                ip_start++; // skip '['
                ip_stop = strchr(ip_address, ']');
                if (ip_stop) {
                    port_start = strchr(ip_stop, ':');
                }
            } else {
                ip_stop = strchr(ip_address, ':');
                port_start = ip_stop;
            }
            if (ip_stop && port_start) {
                char tmp_ch = *ip_stop;
                *ip_stop = '\0'; // split IP and port
                address->set_ip_address(ip_start);
                port_start++; // skip ':'
                int port = std::strtol(port_start, NULL, 10);
                address->set_port(port);
                *ip_stop = tmp_ch; // restore original IP string
            }
        }
    } else {
        if (address) {
            *address = socket->remoteAddress;
        }
    }

    nsapi_size_or_error_t recv_len = _at.read_bytes((uint8_t *)buffer, len);

    if (recv_len < len) {
        goto sisr_retry;
    }

    _at.resp_stop();

    RESTORE_URCs_AND_RETURN((_at.get_last_error() == NSAPI_ERROR_OK) ? (recv_len ? recv_len : NSAPI_ERROR_WOULD_BLOCK) : NSAPI_ERROR_DEVICE_ERROR);
}

// setup internet connection profile for sockets
nsapi_error_t GEMALTO_CINTERION_CellularStack::create_connection_profile(int connection_profile_id)
{
    if (GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleEMS31) {
        // EMS31 connection has only DNS settings and there is no need to modify those here for now
        return NSAPI_ERROR_OK;
    }

    if (GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleTX62) {
        _at.cmd_start_stop("^SICA", "=", "%d%d", 1, _cid);
        tr_debug("Cinterion profile %d, %s (err %d)", connection_profile_id, (_stack_type == IPV4_STACK) ? "IPv4" : "IPv6", _at.get_last_error());
        return _at.get_last_error();
    }

    char conParamType[sizeof("GPRS0") + 1];
    std::sprintf(conParamType, "GPRS%d", (_stack_type == IPV4_STACK) ? 0 : 6);

    _at.cmd_start_stop("^SICS", "?");
    bool found_connection = false;
    _at.resp_start("^SICS:");
    while (_at.info_resp()) {
        int id = _at.read_int();
        if (id == connection_profile_id) {
            char paramTag[16];
            int paramTagLen = _at.read_string(paramTag, sizeof(paramTag));
            if (paramTagLen > 0) {
                char paramValue[100 + 1]; // APN may be up to 100 chars
                int paramValueLen = _at.read_string(paramValue, sizeof(paramValue));
                if (paramValueLen >= 0) {
                    if (strcmp(paramTag, "conType") == 0) {
                        if (strcmp(paramValue, conParamType) == 0) {
                            found_connection = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    _at.resp_stop();

    // connection profile is bound to a PDP context and it can not be changed
    if (!found_connection) {
        _at.at_cmd_discard("^SICS", "=", "%d%s%s", connection_profile_id, "conType", conParamType);

        if (_apn && strlen(_apn) > 0) {
            _at.at_cmd_discard("^SICS", "=", "%d%s%s", connection_profile_id, "apn", _apn);
        }

        if (_user && strlen(_user) > 0) {
            _at.at_cmd_discard("^SICS", "=", "%d%s%s", connection_profile_id, "user", _user);
        }

        if (_password && strlen(_password) > 0) {
            _at.at_cmd_discard("^SICS", "=", "%d%s%s", connection_profile_id, "passwd", _password);
        }

        // set maximum inactivity timeout
        _at.at_cmd_discard("^SICS", "=", "%d%s%d", connection_profile_id, "inactTO", 0xffff);

        // use URC mode ON
        _at.at_cmd_discard("^SCFG", "=", "%s%s", "Tcp/withURCs", "on");
    }

    tr_debug("Cinterion profile %d, %s (err %d)", connection_profile_id, (_stack_type == IPV4_STACK) ? "IPv4" : "IPv6", _at.get_last_error());
    return _at.get_last_error();
}

void GEMALTO_CINTERION_CellularStack::close_connection_profile(int connection_profile_id)
{
    if (GEMALTO_CINTERION::get_module() == GEMALTO_CINTERION::ModuleEMS31) {
        return;
    }

    // To clear connection profile need to detach from packet data.
    // After detach modem sends PDP disconnected event to network class,
    // which propagates network disconnected to upper layer to start reconnecting.
    _at.at_cmd_discard("+CGATT", "=0");
    _at.clear_error();

    _at.at_cmd_discard("^SICS", "=", "%d%s%s", connection_profile_id, "conType", "none");

    _at.clear_error();
}

nsapi_error_t GEMALTO_CINTERION_CellularStack::socket_connect(nsapi_socket_t handle, const SocketAddress &address)
{
    int err = NSAPI_ERROR_DEVICE_ERROR;

    struct CellularSocket *socket = (struct CellularSocket *)handle;
    if (!socket) {
        return err;
    }

    _at.lock();
    err = create_socket_impl(socket);
    if (err != NSAPI_ERROR_OK) {
        _at.unlock();
        return err;
    }
    err = socket_open_defer(socket, &address);
    _at.unlock();

    if (err == NSAPI_ERROR_OK) {
        socket->remoteAddress = address;
        socket->connected = true;
    }

    return err;
}
