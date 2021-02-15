/* mbed Microcontroller Library
 * Copyright (c) 2018-2021 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2021 Arduino SA
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

#if DEVICE_USBDEVICE

#include "USBPhyHw.h"
#include "pinmap.h"
#include "pico.h"
#include "rp2040_usb.h"
#include "pico/fix/rp2040_usb_device_enumeration.h"

/*------------------------------------------------------------------*/
/* Low level controller
 *------------------------------------------------------------------*/

#define MAX_PACKET_SIZE_EP0     (64)

#define usb_hw_set hw_set_alias(usb_hw)
#define usb_hw_clear hw_clear_alias(usb_hw)

// Init these in dcd_init
static uint8_t assigned_address;
static uint8_t *next_buffer_ptr;

// Endpoints 0-15, direction 0 for out and 1 for in.
static struct hw_endpoint hw_endpoints[16][2] = {0};

typedef enum
{
  TUSB_DIR_OUT = 0,
  TUSB_DIR_IN  = 1,

  TUSB_DIR_IN_MASK = 0x80
}tusb_dir_t;

// Get direction from Endpoint address
static inline tusb_dir_t tu_edpt_dir(uint8_t addr)
{
  return (addr & TUSB_DIR_IN_MASK) ? TUSB_DIR_IN : TUSB_DIR_OUT;
}

// Get Endpoint number from address
static inline uint8_t tu_edpt_number(uint8_t addr)
{
  return (uint8_t)(addr & (~TUSB_DIR_IN_MASK));
}

static inline uint8_t tu_edpt_addr(uint8_t num, uint8_t dir)
{
  return (uint8_t)(num | (dir ? TUSB_DIR_IN_MASK : 0));
}

static inline struct hw_endpoint *hw_endpoint_get_by_num(uint8_t num, uint8_t in)
{
    return &hw_endpoints[num][in];
}

static struct hw_endpoint *hw_endpoint_get_by_addr(uint8_t ep_addr)
{
    uint8_t num = tu_edpt_number(ep_addr);
    uint8_t in = (ep_addr & TUSB_DIR_IN_MASK) ? 1 : 0;
    return hw_endpoint_get_by_num(num, in);
}

static void _hw_endpoint_alloc(struct hw_endpoint *ep)
{
    uint size = 64;
    if (ep->wMaxPacketSize > 64)
    {
        size = ep->wMaxPacketSize;
    }

    // Assumes single buffered for now
    ep->hw_data_buf = next_buffer_ptr;
    next_buffer_ptr += size;
    // Bits 0-5 are ignored by the controller so make sure these are 0
    if ((uintptr_t)next_buffer_ptr & 0b111111u)
    {
        // Round up to the next 64
        uint32_t fixptr = (uintptr_t)next_buffer_ptr;
        fixptr &= ~0b111111u;
        fixptr += 64;
        pico_info("Rounding non 64 byte boundary buffer up from %x to %x\n", (uintptr_t)next_buffer_ptr, fixptr);
        next_buffer_ptr = (uint8_t*)fixptr;
    }
    assert(((uintptr_t)next_buffer_ptr & 0b111111u) == 0);
    uint dpram_offset = hw_data_offset(ep->hw_data_buf);
    assert(hw_data_offset(next_buffer_ptr) <= USB_DPRAM_MAX);

    pico_info("Alloced %d bytes at offset 0x%x (0x%p) for ep %d %s\n",
                size,
                dpram_offset,
                ep->hw_data_buf,
                ep->num,
                ep_dir_string[ep->in]);

    // Fill in endpoint control register with buffer offset
    uint32_t reg =  EP_CTRL_ENABLE_BITS
                  | EP_CTRL_INTERRUPT_PER_BUFFER
                  | (ep->transfer_type << EP_CTRL_BUFFER_TYPE_LSB)
                  | dpram_offset;

    *ep->endpoint_control = reg;
}

static void _hw_endpoint_init(struct hw_endpoint *ep, uint8_t ep_addr, uint wMaxPacketSize, uint8_t transfer_type)
{
    uint8_t num = tu_edpt_number(ep_addr);
    bool in = ep_addr & TUSB_DIR_IN_MASK;
    ep->ep_addr = ep_addr;
    ep->in = in;
    // For device, IN is a tx transfer and OUT is an rx transfer
    ep->rx = in == false;
    ep->num = num;
    // Response to a setup packet on EP0 starts with pid of 1
    ep->next_pid = num == 0 ? 1u : 0u;

    // Add some checks around the max packet size
    if (transfer_type == TUSB_XFER_ISOCHRONOUS)
    {
        if (wMaxPacketSize > USB_MAX_ISO_PACKET_SIZE)
        {
            panic("Isochronous wMaxPacketSize %d too large", wMaxPacketSize);
        }
    }
    else
    {
        if (wMaxPacketSize > USB_MAX_PACKET_SIZE)
        {
            panic("Isochronous wMaxPacketSize %d too large", wMaxPacketSize);
        }
    }

    ep->wMaxPacketSize = wMaxPacketSize;
    ep->transfer_type = transfer_type;

    // Every endpoint has a buffer control register in dpram
    if (ep->in)
    {
        ep->buffer_control = &usb_dpram->ep_buf_ctrl[num].in;
    }
    else
    {
        ep->buffer_control = &usb_dpram->ep_buf_ctrl[num].out;
    }

    // Clear existing buffer control state
    *ep->buffer_control = 0;

    if (ep->num == 0)
    {
        // EP0 has no endpoint control register because
        // the buffer offsets are fixed
        ep->endpoint_control = NULL;

        // Buffer offset is fixed
        ep->hw_data_buf = (uint8_t*)&usb_dpram->ep0_buf_a[0];
    }
    else
    {
        // Set the endpoint control register (starts at EP1, hence num-1)
        if (in)
        {
            ep->endpoint_control = &usb_dpram->ep_ctrl[num-1].in;
        }
        else
        {
            ep->endpoint_control = &usb_dpram->ep_ctrl[num-1].out;
        }

        // Now alloc a buffer and fill in endpoint control register
        _hw_endpoint_alloc(ep);
    }

    ep->configured = true;
}

static void hw_endpoint_init(uint8_t ep_addr, uint wMaxPacketSize, uint8_t bmAttributes)
{
    struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);
    _hw_endpoint_init(ep, ep_addr, wMaxPacketSize, bmAttributes);
}

static void hw_endpoint_xfer(uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes, bool start)
{
    struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);
    _hw_endpoint_xfer(ep, buffer, total_bytes, start);
}

static void hw_handle_buff_status(void)
{
    uint32_t remaining_buffers = usb_hw->buf_status;
    pico_trace("buf_status 0x%08x\n", remaining_buffers);
    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_MAX_ENDPOINTS * 2; i++)
    {
        if (remaining_buffers & bit)
        {
            uint __unused which = (usb_hw->buf_cpu_should_handle & bit) ? 1 : 0;
            // Should be single buffered
            assert(which == 0);
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            struct hw_endpoint *ep = hw_endpoint_get_by_num(i >> 1u, !(i & 1u));
            // Continue xfer
            bool done = _hw_endpoint_xfer_continue(ep);
            if (done)
            {
                // Notify
                dcd_event_xfer_complete(0, ep->ep_addr, ep->len, XFER_RESULT_SUCCESS, true);
                hw_endpoint_reset_transfer(ep);
            }
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

static void reset_ep0(void)
{
    // If we have finished this transfer on EP0 set pid back to 1 for next
    // setup transfer. Also clear a stall in case
    uint8_t addrs[] = {0x0, 0x80};
    for (uint i = 0 ; i < count_of(addrs); i++)
    {
        struct hw_endpoint *ep = hw_endpoint_get_by_addr(addrs[i]);
        ep->next_pid = 1u;
        ep->stalled  = 0;
    }
}

static void ep0_0len_status(void)
{
    // Send 0len complete response on EP0 IN
    reset_ep0();
    hw_endpoint_xfer(0x80, NULL, 0, true);
}

static void _hw_endpoint_stall(struct hw_endpoint *ep)
{
    assert(!ep->stalled);
    if (ep->num == 0)
    {
        // A stall on EP0 has to be armed so it can be cleared on the next setup packet
        usb_hw_set->ep_stall_arm = ep->in ? USB_EP_STALL_ARM_EP0_IN_BITS : USB_EP_STALL_ARM_EP0_OUT_BITS;
    }
    _hw_endpoint_buffer_control_set_mask32(ep, USB_BUF_CTRL_STALL);
    ep->stalled = true;
}

static void hw_endpoint_stall(uint8_t ep_addr)
{
    struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);
    _hw_endpoint_stall(ep);
}

static void _hw_endpoint_clear_stall(struct hw_endpoint *ep)
{
    if (ep->num == 0)
    {
        // Probably already been cleared but no harm
        usb_hw_clear->ep_stall_arm = ep->in ? USB_EP_STALL_ARM_EP0_IN_BITS : USB_EP_STALL_ARM_EP0_OUT_BITS;
    }
    _hw_endpoint_buffer_control_clear_mask32(ep, USB_BUF_CTRL_STALL);
    ep->stalled = false;
}

static void hw_endpoint_clear_stall(uint8_t ep_addr)
{
    struct hw_endpoint *ep = hw_endpoint_get_by_addr(ep_addr);
    _hw_endpoint_clear_stall(ep);
}

static void dcd_rp2040_irq(void)
{
}

#define USB_INTS_ERROR_BITS ( \
    USB_INTS_ERROR_DATA_SEQ_BITS      |  \
    USB_INTS_ERROR_BIT_STUFF_BITS     |  \
    USB_INTS_ERROR_CRC_BITS           |  \
    USB_INTS_ERROR_RX_OVERFLOW_BITS   |  \
    USB_INTS_ERROR_RX_TIMEOUT_BITS)

/*------------------------------------------------------------------*/
/* DCD Endpoint port
 *------------------------------------------------------------------*/

void dcd_edpt0_status_complete(uint8_t rhport, tusb_control_request_t const * request)
{
    pico_trace("dcd_edpt0_status_complete %d\n", rhport);
    assert(rhport == 0);

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
        request->bRequest == TUSB_REQ_SET_ADDRESS)
    {
        pico_trace("Set HW address %d\n", assigned_address);
        usb_hw->dev_addr_ctrl = assigned_address;
    }

    reset_ep0();
}

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
    pico_info("dcd_edpt_open %d %02x\n", rhport, desc_edpt->bEndpointAddress);
    assert(rhport == 0);
    hw_endpoint_init(desc_edpt->bEndpointAddress, desc_edpt->wMaxPacketSize.size, desc_edpt->bmAttributes.xfer);
    return true;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
    assert(rhport == 0);
    // True means start new xfer
    hw_endpoint_xfer(ep_addr, buffer, total_bytes, true);
    return true;
}

static USBPhyHw *instance;

USBPhy *get_usb_phy()
{
    static USBPhyHw usbphy;
    return &usbphy;
}

USBPhyHw::USBPhyHw(): events(NULL), sof_enabled(false)
{

}

USBPhyHw::~USBPhyHw()
{

}

void USBPhyHw::init(USBPhyEvents *events)
{
    if (this->events == NULL) {
        sleep_manager_lock_deep_sleep();
    }
    this->events = events;

    // Reset hardware to default state
    rp2040_usb_init();

    irq_set_exclusive_handler(USBCTRL_IRQ, _usbisr);
    memset(hw_endpoints, 0, sizeof(hw_endpoints));
    assigned_address = 0;
    next_buffer_ptr = &usb_dpram->epx_data[0];

    // EP0 always exists so init it now
    // EP0 OUT
    hw_endpoint_init(0x0, 64, 0);
    // EP0 IN
    hw_endpoint_init(0x80, 64, 0);

    // Initializes the USB peripheral for device mode and enables it.
    // Don't need to enable the pull up here. Force VBUS
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable individual controller IRQS here. Processor interrupt enable will be used
    // for the global interrupt enable...
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS; 
    usb_hw->inte     = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS;
}

void USBPhyHw::deinit()
{
    if (events != NULL) {
        sleep_manager_unlock_deep_sleep();
    }
    events = NULL;
}

bool USBPhyHw::powered()
{
    return true;
}

void USBPhyHw::connect()
{
    // connect by enabling internal pull-up resistor on D+/D-
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
    irq_set_enabled(USBCTRL_IRQ, true);
}

void USBPhyHw::disconnect()
{
    // disconnect by disabling internal pull-up resistor on D+/D-
    usb_hw_clear->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
    irq_set_enabled(USBCTRL_IRQ, false);
}

void USBPhyHw::configure()
{
    // Not needed
}

void USBPhyHw::unconfigure()
{
    // Not needed
}

void USBPhyHw::sof_enable()
{
    sof_enabled = true;
}

void USBPhyHw::sof_disable()
{
    sof_enabled = false;
}

void USBPhyHw::set_address(uint8_t address)
{
    // Can't set device address in hardware until status xfer has complete
    assigned_address = address;

    ep0_0len_status();
}

void USBPhyHw::remote_wakeup()
{

}

const usb_ep_table_t *USBPhyHw::endpoint_table()
{
    static const usb_ep_table_t table = {
        16 * 64;
        {
            {USB_EP_ATTR_ALLOW_CTRL | USB_EP_ATTR_DIR_IN_AND_OUT, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
            {USB_EP_ATTR_ALLOW_ALL, 0, 0},
        }
    };
    return &table;
}

uint32_t USBPhyHw::ep0_set_max_packet(uint32_t max_packet)
{
    // FUTURE - set endpoint 0 size and return this size
    return MAX_PACKET_SIZE_EP0;
}

// read setup packet
void USBPhyHw::ep0_setup_read_result(uint8_t *buffer, uint32_t size)
{
    if (size > MAX_PACKET_SIZE_SETUP) {
        size = MAX_PACKET_SIZE_SETUP;
    }
    memcpy(buffer,  hpcd.Setup, size);
    memset(hpcd.Setup, 0, MAX_PACKET_SIZE_SETUP);
}

void USBPhyHw::ep0_read(uint8_t *data, uint32_t size)
{
    HAL_StatusTypeDef ret;
    epComplete[EP_TO_IDX(0x00)] = 2;
    ret = HAL_PCD_EP_Receive(&hpcd, 0x00, data, size > MAX_PACKET_SIZE_EP0 ? MAX_PACKET_SIZE_EP0 : size);
    MBED_ASSERT(ret != HAL_BUSY);
}

uint32_t USBPhyHw::ep0_read_result()
{
    epComplete[EP_TO_IDX(0x00)] = 0;
    return HAL_PCD_EP_GetRxCount(&hpcd, 0);
}

void USBPhyHw::ep0_write(uint8_t *buffer, uint32_t size)
{
    /*  check that endpoint maximum size is not exceeding TX fifo */
    MBED_ASSERT(hpcd.IN_ep[0].maxpacket >= size);
    endpoint_write(0x80, buffer, size);
}

void USBPhyHw::ep0_stall()
{
    endpoint_stall(0x80);
    endpoint_stall(0x00);
}

bool USBPhyHw::endpoint_add(usb_ep_t endpoint, uint32_t max_packet, usb_ep_type_t type)
{
    hw_endpoint_init(endpoint, max_packet, type);
}

void USBPhyHw::endpoint_remove(usb_ep_t endpoint)
{
}

void USBPhyHw::endpoint_stall(usb_ep_t endpoint)
{
    hw_endpoint_stall(endpoint);
}

void USBPhyHw::endpoint_unstall(usb_ep_t endpoint)
{
    hw_endpoint_clear_stall(endpoint);
}

bool USBPhyHw::endpoint_read(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    hw_endpoint_xfer(endpoint, data, size, true);
}

uint32_t USBPhyHw::endpoint_read_result(usb_ep_t endpoint)
{
    if (epComplete[EP_TO_IDX(endpoint)] == 0) {
        /*  no reception possible !!! */
        return 0;
    } else if ((epComplete[EP_TO_IDX(endpoint)] != 1)) {
        return 0;
    }
    epComplete[EP_TO_IDX(endpoint)] = 0;
    return HAL_PCD_EP_GetRxCount(&hpcd, endpoint);;
}

bool USBPhyHw::endpoint_write(usb_ep_t endpoint, uint8_t *data, uint32_t size)
{
    hw_endpoint_xfer(endpoint, data, size, true);
}

void USBPhyHw::endpoint_abort(usb_ep_t endpoint)
{
    HAL_StatusTypeDef ret = HAL_PCD_EP_Close(&hpcd, endpoint); // fix me: implementation not correct
    MBED_ASSERT(ret == HAL_OK);
}

void USBPhyHw::process()
{
    HAL_PCD_IRQHandler(&instance->hpcd);
    // Re-enable interrupt
    NVIC_ClearPendingIRQ(USBHAL_IRQn);
    NVIC_EnableIRQ(USBHAL_IRQn);
}

void USBPhyHw::_usbisr(void)
{
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;

    if (status & USB_INTS_SETUP_REQ_BITS)
    {
        handled |= USB_INTS_SETUP_REQ_BITS;
        uint8_t const *setup = (uint8_t const *)&usb_dpram->setup_packet;
        // Clear stall bits and reset pid
        reset_ep0();
        // Pass setup packet to tiny usb
        dcd_event_setup_received(0, setup, true);
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    }

    if (status & USB_INTS_BUFF_STATUS_BITS)
    {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        hw_handle_buff_status();
    }

    if (status & USB_INTS_BUS_RESET_BITS)
    {
        pico_trace("BUS RESET (addr %d -> %d)\n", assigned_address, 0);
        assigned_address = 0;
        usb_hw->dev_addr_ctrl = assigned_address;
        handled |= USB_INTS_BUS_RESET_BITS;
        dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, true);
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        rp2040_usb_device_enumeration_fix();
    }

    if (status ^ handled)
    {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }
}

#endif /* DEVICE_USBDEVICE */
