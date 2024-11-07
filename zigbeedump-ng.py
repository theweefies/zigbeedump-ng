#!/usr/bin/env python3

import signal
import usb1
import time
import sys
import gps
import queue
import struct
import argparse
from threading import Thread
from datetime import datetime

from ieee802154_crc import ieee802154_crc16
from ppi import calculate_ppi_header_length, write_ppi_headers

BUF_SIZE   = 256
TIMEOUT    = 200  # milliseconds

VENDOR_ID  = 0x0451
PRODUCT_ID = 0x16ae

DARK_RED = '\x1b[31m'
DEFAULT = "\033[0m"
CURSOR_TO_TOP = '\x1b[H'
CLEAR_SCREEN_CURSOR_TO_TOP = '\x1b[2J\x1b[H'
CURSOR_HIDE = '\x1b[?25l'
CURSOR_SHOW = '\x1b[?25h'

signal_exit = False
gpsd = False
outfile = None

packet_queue = queue.Queue()

cur_channel = 11
num_channels = None
channels_list = []
cur_channel_index = 0

aps = {}
stas = {}

def signal_handler(sig, frame):
    global signal_exit
    print("\nCTRL+C received, stopping...")
    signal_exit = True

# Register the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

class GPS:
    gpsd_sock = None
    gpsd_stream = None
    latitude = None
    longitude = None
    altitude = None

    def __init__(self):
        self.gpsd_sock = gps.GPSDSocket()
        self.gpsd_stream = gps.DataStream()

    def start(self, host, port):
        self.gpsd_sock.connect(host, port)
        self.gpsd_sock.watch()
        for packet in self.gpsd_sock:
            if packet:
                self.gpsd_stream.unpack(packet)
                self.latitude = self.gpsd_stream.TPV['lat']
                self.longitude = self.gpsd_stream.TPV['lon']
                self.altitude = self.gpsd_stream.TPV['alt']
    
    def get_data(self):
        return self.latitude, self.longitude, self.altitude

def hide_cursor() -> None:
    """
    Function to hide the cursor.
    """
    sys.stdout.write(CURSOR_HIDE)
    sys.stdout.flush()

def show_cursor() -> None:
    """
    Function to show the cursor.
    """
    sys.stdout.write(CURSOR_SHOW)
    sys.stdout.flush()

def display(aps: dict, stas: dict, gps_object: GPS) -> None:
    global signal_exit, cur_channel, gpsd

    start_time = time.time()

    timeout = 60
    bssid = None
    targets = None
    last_total_count = {}

    # Define maximum widths for each column
    max_widths = {
        'BSSID': 19, 'PWR': 5, 'RXQ': 5, 'Beacons': 10, '#Data': 7,
        '#/s': 5, 'CH': 4, 'MB': 4, 'ENC': 5, 'CIPHER': 8, 'AUTH': 6, 'ESSID': 20
    }

    # Define the header and row formats using the maximum widths
    header_format = " ".join([f"{{:<{max_widths[col]}}}" for col in max_widths])
    header = header_format.format(*max_widths.keys())

    # Define maximum widths for each column
    max_widths = {
        'BSSID': 19, 'STA': 19, 'PWR': 5, 'Rate': 8, 'Lost': 7,
        'Frames': 8, 'Probe': 20
    }

    footer_format = " ".join([f"{{:<{max_widths[col]}}}" for col in max_widths])
    footer = footer_format.format(*max_widths.keys())

    sys.stdout.write(CLEAR_SCREEN_CURSOR_TO_TOP) # initial clear screen and move cursor to top left

    while not signal_exit:

        sys.stdout.write(CLEAR_SCREEN_CURSOR_TO_TOP)

        display_body = ""

        current_time = time.time()
        elapsed_time = int(current_time - start_time)
        elapsed_str = f"{elapsed_time // 60} min" if elapsed_time >= 60 else f"{elapsed_time} s"
        current_date_time = time.strftime('%Y-%m-%d %H:%M', time.localtime())

        status_line = f"cc2355 ][ Elapsed: {elapsed_str} ][ {current_date_time} ][ CH {cur_channel if cur_channel else ''} "

        if gpsd:
            latitude, longitude, _ = gps_object.get_data()
            if latitude and longitude:
                status_line += f"][ GPS: {latitude:.4f}, {longitude:.4f}"

        display_body += CURSOR_TO_TOP
        display_body += " " + status_line + '\n\n' 
        # Print the header
        display_body += " " + header + "\n\n"
        
        for bssid, ap in aps.items():
            # Packet count management
            total_packets = ap.total
            previous_total = last_total_count.get(bssid, total_packets)
            packets_per_second = total_packets - previous_total
            ap.pps = str(packets_per_second)
            last_total_count[bssid] = total_packets
            
            # If bssid filtering is applied, and our bssid does not match we 
            # won't display the device
            if bssid and bssid != bssid:
                continue

            ap_line = str(ap) + '\n'

            # timeout management
            if (current_time - ap.last_seen) < timeout:
                if targets:
                    for target in targets:
                        if target == bssid:
                            ap_line = DARK_RED + str(ap) + DEFAULT + '\n'

                display_body += ap_line

            # if timeout is set to zero, we never remove devices from the display
            elif timeout == 0:
                if targets:
                    for target in targets:
                        if target == bssid:
                            ap_line = DARK_RED + str(ap) + DEFAULT + '\n'

                display_body += ap_line

            # still want another new line even if timeout is reached for cur ap
            display_body += "\n"
            
            # Print the footer
            display_body += " " + footer + "\n\n"

            for sta_mac, sta in stas.items():

                sta_line = str(sta) + '\n'

                # timeout management
                if (current_time - sta.last_seen) < timeout:
                    if targets:
                        for target in targets:
                            if target == sta_mac:
                                sta_line = DARK_RED + str(sta) + DEFAULT + '\n'
                    
                    display_body += sta_line

                # if timeout is set to zero, we never remove devices from the display
                elif timeout == 0:
                    # Even if the timeout is set to never happen, we want to clear the probe field
                    if (current_time - sta.last_seen) < 60:
                        sta.probe = ""
                    if targets:
                        for target in targets:
                            if target == sta_mac:
                                sta_line = DARK_RED + str(sta) + DEFAULT + '\n'
                    
                    display_body += sta_line
                else:
                    sta.probe = ""

            # still want another new line even if timeout is reached for cur sta
            display_body += "\n"

        sys.stdout.write(display_body)
        sys.stdout.flush()
        time.sleep(1)

class USBPacketParser:
    USB_HEADER_SIZE = 3        # USBHeaderType size (1 byte type + 2 bytes length)
    DATA_HEADER_SIZE = 5      # USBDataHeaderType size (3 + 4 bytes timestamp + 1 byte WPAN length)
    TIMESTAMP_DIVISOR = 32     # Factor to convert timestamp ticks to microseconds
    DATA_HEADER_TOO_SHORT = "Data too short for USB header"
    DATA_LEN_MISMATCH     = "Data length does not match usb_len"
    TICK_MSG              = "Tick Header Message"
    USB_HDR_UNKNOWN       = "Unknown USB header type"
    TICK_MSG_TOO_SHORT    = "Data too short for tick header"
    ERR_MSG = {-1: DATA_HEADER_TOO_SHORT, -2:DATA_LEN_MISMATCH, -3:TICK_MSG, -4:USB_HDR_UNKNOWN, -5:TICK_MSG_TOO_SHORT}

    def __init__(self, data, timestamp_epoch=None):
        self.raw_data = data
        self.type = None
        self.usb_len = None
        self.timestamp = None
        self.wpan_len = None
        self.packet_data = None
        self.timestamp_epoch = int(timestamp_epoch or datetime.now().timestamp())
        self.timestamp_offset = 0
        self.timestamp_tick = 0
        self.ts_sec = 0
        self.ts_usec = 0
        self.res = self.parse_usb_header()
        
    def parse_usb_header(self):
        # Parse the USB header (first 4 bytes)
        if len(self.raw_data) < self.USB_HEADER_SIZE:
            return -1

        # Unpack USB header: type (1 byte), usb_len (2 bytes, little-endian)
        self.type, self.usb_len = struct.unpack_from("<BH", self.raw_data)
        
        # Verify we have enough data for the rest of the packet
        if len(self.raw_data) < self.USB_HEADER_SIZE + self.usb_len:
            return -2

        # Process data depending on type
        if self.type == 0:
            self.res = self.parse_data_header()
            return self.res
        elif self.type == 1:
            self.res = self.parse_tick_header()
            return self.res
        else:
            return -4

    def parse_data_header(self):
        # Check if we have enough data for USBDataHeaderType
        if len(self.raw_data) < self.USB_HEADER_SIZE + self.DATA_HEADER_SIZE:
            return -2

        # Unpack data header: timestamp (4 bytes), wpan_len (1 byte)
        self.timestamp, self.wpan_len = struct.unpack("<IB", self.raw_data[self.USB_HEADER_SIZE:self.USB_HEADER_SIZE + self.DATA_HEADER_SIZE])

        # Calculate adjusted timestamp
        timestamp_us = (self.timestamp_tick + self.timestamp) / self.TIMESTAMP_DIVISOR

        if self.timestamp_offset == 0:
            self.timestamp_offset = timestamp_us
        timestamp_us -= self.timestamp_offset

        # Convert to seconds and microseconds for timestamp display
        self.ts_sec = int(timestamp_us / 1_000_000)
        self.ts_usec = int(timestamp_us % 1_000_000)

        # Extract WPAN packet data
        data_offset = self.USB_HEADER_SIZE + self.DATA_HEADER_SIZE
        self.packet_data = self.raw_data[data_offset:data_offset + self.wpan_len]

        return 1

    def parse_tick_header(self):
        # Parse tick header (only 1 byte after the USB header)
        tick_offset = self.USB_HEADER_SIZE
        if len(self.raw_data) <= tick_offset:
            return -5

        # Tick header contains only a single byte for the tick counter
        tick_value, = struct.unpack_from("<B", self.raw_data, tick_offset)
        # Handle tick rollover
        if tick_value == 0x00:
            self.timestamp_tick += 0xFFFFFFFF
        else:
            self.timestamp_tick += tick_value

        return -3

    def get_packet_info(self):
        # Display parsed packet information
        info = {
            "type": self.type,
            "usb_len": self.usb_len,
            "timestamp": self.ts_sec + self.timestamp_epoch if self.type == 0 else None,
            "microseconds": self.ts_usec if self.type == 0 else None,
            "wpan_len": self.wpan_len if self.type == 0 else None,
            "packet_data": self.packet_data if self.type == 0 else None,
            "tick": self.timestamp_tick if self.type == 1 else None,
        }
        return info

pcap_hdr = struct.pack(
    '<IHHIIII',      # Format for little-endian byte order: magic number, version, zone, etc.
    0xA1B2C3D4,      # magic_number
    2,               # version_major
    4,               # version_minor
    0,               # thiszone (GMT)
    0,               # sigfigs (standard zero)
    128,             # snaplen (max 802.15.4 packet length)
    192              # network (or 195 for IEEE 802.15.4 depending on choice)
)

def close_usb_sniffer(handle: usb1.USBDeviceHandle):
    try:
        # Stop sniffing
        #handle.controlWrite(0x40, 209, 0, 0, b'', TIMEOUT)
        # Power off radio, wIndex = 0
        #handle.controlWrite(0x40, 197, 0, 0, b'', TIMEOUT)
        # Release interface and close the handle
        #handle.releaseInterface(0)
        handle.close()
        
        # Exit libusb
        #handle.getContext().exit()
        
    except usb1.USBError as e:
        print("Error closing USB sniffer:", e)

def init_usb_sniffer(channel):
    usb_buf = bytearray(BUF_SIZE)

    # Initialize libusb
    context = usb1.USBContext()
    context.setDebug(3)

    handle = None

    # Check if the device is the CC2531 USB Dongle (Vendor ID: 0x0451, Product ID: 0x16ae)
    handle = context.openByVendorIDAndProductID(VENDOR_ID, PRODUCT_ID)
    if handle is None:
        print("Device not found")
        return None, None
    
    # Detach kernel driver if necessary
    if handle.kernelDriverActive(0):
        try:
            handle.detachKernelDriver(0)
        except usb1.USBError as e:
            print("ERROR: Could not detach kernel driver from CC2531 USB Dongle.")
            handle.close()
            return None, None

    # Set configuration and claim interface
    try:
        handle.setConfiguration(1)
        handle.claimInterface(0)
    except usb1.USBError as e:
        print("--> unable to set configuration or claim interface for device:", e)
        handle.close()
        return None, None

    # Perform control transfers to initialize device
    try:
        # Get identity from firmware command
        handle.controlRead(0xC0, 192, 0, 0, BUF_SIZE, TIMEOUT)

        # Power on radio, wIndex = 4
        handle.controlWrite(0x40, 197, 0, 4, b'', TIMEOUT)

        # Wait until powered up
        while True:
            status = handle.controlRead(0xC0, 198, 0, 0, 1, TIMEOUT)
            if status[0] == 0x04:
                break
            time.sleep(0.01)  # 10 ms

        # Unknown command
        handle.controlWrite(0x40, 201, 0, 0, b'', TIMEOUT)

        # Set channel command
        usb_buf[0] = channel
        handle.controlWrite(0x40, 210, 0, 0, usb_buf[:1], TIMEOUT)
        usb_buf[0] = 0x00
        handle.controlWrite(0x40, 210, 0, 1, usb_buf[:1], TIMEOUT)

        # Start sniffing
        handle.controlWrite(0x40, 208, 0, 0, b'', TIMEOUT)

    except usb1.USBError as e:
        print("ERROR during USB control transfer:", e)
        handle.close()
        return None, None

    return handle, context

def packet_handler(buf: bytes, gpsd_object: GPS):
    global outfile

    usb = USBPacketParser(buf)
    if usb.res < 0:
        msg = usb.ERR_MSG.get(usb.res,"")
        return
    
    incl_len = orig_len = usb.wpan_len
    if not usb.wpan_len or usb.wpan_len == 0:
        return

    pcaprec_hdr = struct.pack("<IIII", usb.ts_sec + usb.timestamp_epoch, usb.ts_usec, incl_len, orig_len)

    # GPS coordinates setup
    gpsLat, gpsLon, gpsAlt = 0.0, 0.0, 0.0
    if gpsd_object:
        gpsLat, gpsLon, gpsAlt = gpsd_object.get_data()

    # Calculate PPI header length (placeholder for actual function)
    ppi_header_len = calculate_ppi_header_length(gpsLat, gpsLon, gpsAlt)
    incl_len = orig_len = usb.wpan_len + ppi_header_len
    pcaprec_hdr = struct.pack("<IIII", usb.ts_sec + usb.timestamp_epoch, usb.ts_usec, incl_len, orig_len)

    f_out = open(outfile, 'ab')
    # Write the packet header to the pcap file
    f_out.write(pcaprec_hdr)

    # Extract RSSI and LQI from the packet
    rssi_index = -2
    rssi_unsigned = usb.packet_data[rssi_index]
    lqi_unsigned = usb.packet_data[rssi_index + 1]

    # Convert RSSI and LQI
    rssi_signed = ((rssi_unsigned + (1 << 7)) % (1 << 8)) - (1 << 7) - 73
    lqi_signed = ((lqi_unsigned + (1 << 7)) % (1 << 8)) - (1 << 7) - 73
    print(f"RSSI: {rssi_signed}")

    # Write PPI headers (placeholder for actual function) 4th is channel
    write_ppi_headers(f_out, 0, 1, cur_channel, rssi_signed, lqi_signed, gpsLat, gpsLon, gpsAlt)

    # Write the actual packet data to the file
    f_out.write(usb.packet_data)

    # Compute FCS for the packet data and write it to the file
    fcs = ieee802154_crc16(usb.packet_data, 0, len(usb.packet_data) - 2)
    le_fcs = struct.pack("<H", fcs)  # htole16 equivalent
    f_out.write(le_fcs)
    f_out.flush()
    f_out.close()

def process_packets(gpsd_object: GPS):
    global signal_exit

    while not signal_exit:
        try:
            packet = packet_queue.get(timeout=1)
            packet_handler(packet, gpsd_object)
            packet_queue.task_done()
        except queue.Empty:
            continue

# Callback function to handle completed transfers
def transfer_callback(transfer):
    if transfer.getStatus() == usb1.TRANSFER_COMPLETED:
        if not signal_exit:
            data = transfer.getBuffer()[:transfer.getActualLength()]
            #print(data)
            packet_queue.put(data)
            
            # Re-submit the transfer for continuous reading
            transfer.submit()
    elif transfer.getStatus() == usb1.TRANSFER_CANCELLED:
        return
    else:
        if not signal_exit:
            transfer.submit()

def receive_and_process_data(usb_ctx: usb1.USBContext, handle: usb1.USBDeviceHandle, scan_interval):
    global signal_exit, channels_list, cur_channel_index, cur_channel, num_channels

    start_time = datetime.now()

    transfer = handle.getTransfer()

    # Set up the transfer for bulk reading
    transfer.setBulk(
        endpoint=0x83,
        buffer_or_len=BUF_SIZE,
        callback=transfer_callback,
        timeout=TIMEOUT
    )
    
    # Submit the transfer
    transfer.submit()

    try:
        while not signal_exit:
            try:
                usb_ctx.handleEvents()  # Poll events to handle transfer completions
                
            except usb1.USBErrorTimeout:
                transfer.submit()
            except usb1.USBError as e:
                if not signal_exit:
                    print(f"USB Error: {e}")
                signal_exit = True
                break
            
            # Check if it’s time to change the channel
            elapsed_seconds = (datetime.now() - start_time).total_seconds()
            if num_channels > 1 and elapsed_seconds >= scan_interval:
                cur_channel_index = (cur_channel_index + 1) % num_channels
                cur_channel = channels_list[cur_channel_index]

                print(f"Changing channel to: {cur_channel}")

                handle.controlWrite(0x40, 210, 0, 0, bytearray([cur_channel]), TIMEOUT)
                handle.controlWrite(0x40, 210, 0, 1, b'\x00', TIMEOUT)

                start_time = datetime.now()
    except KeyboardInterrupt:
        signal_exit = True
    finally:
        handle.releaseInterface(0)
        handle.close()

def main(args):
    global outfile, channels_list, num_channels, gpsd, aps, stas

    gpsd     = args.gpsd
    outfile  = args.outfile
    interval = args.interval
    gpsd_object = None

    usb_handle, usb_ctx = init_usb_sniffer(cur_channel)

    if not usb_handle:
        print('Cannot initialize USB sniffer device')
        sys.exit(0)

    with open(args.outfile, 'wb') as f_out:
        f_out.write(pcap_hdr)
        f_out.flush()
        f_out.close()

    for x in range(11,27):
       channels_list.append(x)
    #channels_list = [14,18]
    num_channels = len(channels_list)

    hide_cursor()

    """display_thread = Thread(target=display, args=(aps, stas,))
    display_thread.daemon = True
    display_thread.start()"""

    # Run packet processing in a background thread
    packet_processor = Thread(target=process_packets, args=(gpsd_object,))
    packet_processor.daemon = True
    packet_processor.start()

    if gpsd:
        gps_object = GPS()
        gps_thread = Thread(target=gps_object.start, args=('127.0.0.1', 2947,))
        gps_thread.daemon = True
        gps_thread.start()

    receive_and_process_data(usb_ctx, usb_handle, interval)

    packet_processor.join()

    show_cursor()

if __name__ == "__main__":

    lc_argparse = argparse.ArgumentParser(prog='GreyStream', description='Decrypt packets in real-time and write out to file.')
    lc_argparse.version = '1.0'
    lc_argparse.add_argument('-c', '--channel', required=False,                  type=int, help="Enter a channel number for which to tune the interface.")
    lc_argparse.add_argument('-b', '--bssid',   required=False,                  type=str, help="Enter an AP BSSID address for which to attempt decryption.",               default=5)
    lc_argparse.add_argument('-i', '--interval',required=False, action='store',  type=int, help="Pass this switch to use gpsd and encode coordinates in pcap ppi headers.", default=False)
    lc_argparse.add_argument('-g', '--gpsd',    required=False, action='store_true',       help="Pass this switch to use gpsd and encode coordinates in pcap ppi headers.", default=False)
    lc_argparse.add_argument('-o', '--outfile', required=False, action='store', type=str,  help='Enter a filename to which packets will be written.',                       default="output.pcap")
    lc_argparse.add_argument('-v', '--version', action='version')

    args = lc_argparse.parse_args()

    main(args)
