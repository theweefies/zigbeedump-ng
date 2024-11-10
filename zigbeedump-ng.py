#!/usr/bin/env python3

import signal
import usb1
import pkgutil
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

IEEE802154_DAT  = 0x01
IEEE802154_ACK  = 0x02
IEEE802154_CMD  = 0x03
IEEE802154_RES  = 0x04
IEEE802154_MUP  = 0x05
IEEE802154_FRG  = 0x06
IEEE802154_EXT  = 0x07

signal_exit = False
gpsd = False
outfile = None
oui_table = None

packet_queue = queue.Queue()

cur_channel = 11
num_channels = None
channels_list = []
cur_channel_index = 0

aps = {}
stas = {}

last_total_count = {}

class Count:
    def __init__(self):
        self.beacons = 0
        self.data    = 0

class AP:
    def __init__(self, bssid, rssi, channel):
        self.bssid       = bssid
        self.manu        = ""
        self.rssi        = rssi
        self.clients     = set([])
        self.cur_channel = channel
        self.rate        = 1
        self.enc         = ""
        self.total       = 0
        self.beacons     = 0
        self.data        = 0
        self.pps         = 0

    def __str__(self):
        return f"{self.bssid:<23}{self.rssi:>5}{self.beacons:>10}{self.data:>8} {self.pps:>3} {self.cur_channel:>4} {self.rate:>4} {self.enc:>5} {self.manu:>20}"
    

class STA:
    def __init__(self, mac, bssid=None, rssi=""):
        self.mac = mac
        self.bssid = bssid
        self.rssi = rssi
        self.rate = 1
        self.lost = 0
        self.total = 0
        self.probe = ""

    def __str__(self):
        if not self.bssid:
            self.bssid = '(Not associated)'
        return f"{self.bssid:<23}{self.mac:>23} {self.rssi:>5}{self.rate:>5}{self.lost:>7}   {self.total:>8}    {self.probe:<20}"

def load_oui(oui_file: str, running_pyz: bool) -> dict:
    """
    Function to load an OUI dictionary from an OUI file.
    The OUI dictionary will store the first 6 characters of the OUI as the key and the manufacturer name as the value.
    """
    oui_table = {}

    try:
        lines = None
        if running_pyz:
            # Load from within the .pyz archive
            print('[+] Loading OUI from archive')
            data = pkgutil.get_data(__name__, oui_file)  # Load the file from the archive
            if data is None:
                raise FileNotFoundError(f"[!] {oui_file} not found in the .pyz archive")
            lines = data.decode('utf-8').splitlines()
        else:
            # Load from the file system
            print(f'[+] Loading OUI from file system: {oui_file}')
            with open(oui_file, 'r') as f_oui:
                lines = f_oui.readlines()

        current_oui = None
        current_manufacturer = None

        # Process the OUI file contents line by line
        for line in lines:
            # Ignore empty lines or lines with just spaces
            if line.strip() == '':
                continue
            
            # Check if the line contains OUI information (format: xx-xx-xx (hex))
            if '(hex)' in line:
                # Extract the first 6 characters (OUI) and remove any spaces/hyphens
                current_oui = line.split()[0].replace('-', '').lower()
                current_manufacturer = line.split('\t')[-1].strip()  # Extract the manufacturer name
            
            # Check if the line contains additional manufacturer info (such as address)
            elif current_oui is not None and current_manufacturer is not None:
                # Continue reading manufacturer address lines and skip them (or store if needed)
                if line.startswith('\t'):
                    continue
                else:
                    # Store the OUI and manufacturer in the dictionary
                    oui_table[current_oui] = current_manufacturer
                    current_oui = None
                    current_manufacturer = None

    except FileNotFoundError:
        print(f'[!] File Not Found: {oui_file}')
        return None

    return oui_table


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
    global signal_exit, cur_channel, gpsd, last_total_count

    start_time = time.time()

    timeout = 60
    bssid = None
    targets = None

    # Define maximum widths for each column
    max_widths = {
        'BSSID': 23, 'PWR': 5, 'Beacons': 9, '#Data': 4,
        '#/s': 5, 'CH': 4, 'MB': 4, 'ENC': 5, 'MANUFACTURER':20
    }

    # Define the header and row formats using the maximum widths
    header_format = " ".join([f"{{:<{max_widths[col]}}}" for col in max_widths])
    header = header_format.format(*max_widths.keys())

    # Define maximum widths for each column
    max_widths = {
        'BSSID': 23, 'STA': 23, 'PWR': 5, 'Rate': 8, 'Lost': 7,
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
            # if bssid and bssid != bssid:
            #    continue

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
        
        if stas:
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
    if not usb.wpan_len or usb.wpan_len < 4:
        return

    f_out = open(outfile, 'ab')

    # GPS coordinates setup
    gpsLat, gpsLon, gpsAlt = 0.0, 0.0, 0.0
    if gpsd_object:
        gpsLat, gpsLon, gpsAlt = gpsd_object.get_data()

    # Calculate PPI header length (placeholder for actual function)
    ppi_header_len = calculate_ppi_header_length(gpsLat, gpsLon, gpsAlt)
    #print('PPI Header length:', ppi_header_len)
    #print('wpan_len:', usb.wpan_len)
    #print('usb packet data len:', len(usb.packet_data))
    incl_len = orig_len = usb.wpan_len + ppi_header_len
    pcaprec_hdr = struct.pack("<IIII", usb.ts_sec + usb.timestamp_epoch, usb.ts_usec, incl_len, orig_len)

    # Extract RSSI and LQI from the packet
    rssi_index = -2
    rssi_unsigned = usb.packet_data[rssi_index]
    lqi_unsigned = usb.packet_data[rssi_index + 1]

    # Convert RSSI and LQI
    rssi_signed = ((rssi_unsigned + (1 << 7)) % (1 << 8)) - (1 << 7) - 73
    lqi_signed = ((lqi_unsigned + (1 << 7)) % (1 << 8)) - (1 << 7) - 73
    if 127 < rssi_signed: rssi_signed = 127
    if rssi_signed < -128: rssi_signed = -128
    if 127 < lqi_signed: lqi_signed = 127
    if lqi_signed < -128: lqi_signed = -128
    # print(f"RSSI: {rssi_signed}")

    # Write PPI headers (placeholder for actual function) 4th is channel
    ppi = write_ppi_headers(0, 1, cur_channel, rssi_signed, lqi_signed, gpsLat, gpsLon, gpsAlt)

    # Compute FCS for the packet data and write it to the file
    fcs = ieee802154_crc16(usb.packet_data, 0, len(usb.packet_data) - 2)
    le_fcs = struct.pack("<H", fcs)  # htole16 equivalent

    # create full packet
    full_pkt = pcaprec_hdr + ppi + usb.packet_data[:-2] + le_fcs

    # write out packet to file
    f_out.write(full_pkt)

    f_out.flush()
    f_out.close()

    parse_packet(usb.packet_data[:-2], rssi_signed)

def get_manu(mac: str):
    global oui_table

    manu = ""
    oui = mac.replace(':','').upper()
    manu = oui_table.get(oui, "")

    return manu

def parse_packet(pkt_data: bytes, rssi: int):
    global aps, stas, oui_table

    rate = 1

    def reverse_bytes(byte_string):
        return byte_string[::-1]
    
    def bytes_to_mac(bytes: bytes) -> str:
        return ':'.join('{:02x}'.format(b) for b in bytes)

    pkt_len = len(pkt_data)
    if pkt_len < 3:
        return
    
    index = 0
    frame_control = struct.unpack_from('<H', pkt_data, index)[0]
    index += 2

    frame_type = frame_control & 0x0007
    security_enable = bool((frame_control & 0x0008) >> 3)
    frame_pending = bool((frame_control & 0x0010) >> 4)
    ack_request = bool((frame_control & 0x0020) >> 5)
    pan_id_compression = bool((frame_control & 0x0040) >> 6)
    seqno_suppression = bool((frame_control & 0x0100) >> 8)
    ie_present = bool((frame_control & 0x0200) >> 9)
    dst_addr_mode = (frame_control & 0x0c00) >> 10
    frame_version = (frame_control & 0x3000) >> 12
    src_addr_mode = (frame_control & 0xc000) >> 14

    seqno = 0
    if not seqno_suppression:
        seqno = pkt_data[index]
        index += 1

    dst_pan = src_pan = dst16 = src16 = dst64 = src64 = b''
    sta_cur = None
    ap_cur  = None

    # Handling addressing fields based on the address mode
    if dst_addr_mode:
        dst_pan_present = True
        if dst_addr_mode == 0x2:  # Short address
            if len(pkt_data) < index + 4:
                return None
            dst_pan = reverse_bytes(pkt_data[index:index+2])
            if src_addr_mode != 0:
                dst16 = reverse_bytes(pkt_data[index+2:index+4])
                index += 4
            else:
                dst16 = dst_pan
                index += 2
        elif dst_addr_mode == 0x3:  # Extended address
            if len(pkt_data) < index + 10:
                return None
            dst_pan = reverse_bytes(pkt_data[index:index+2])
            dst64 = reverse_bytes(pkt_data[index+2:index+8])
            dst64 = bytes_to_mac(dst64)
            index += 10

    if src_addr_mode:
        src_pan_present = not pan_id_compression
        if src_addr_mode == 0x2:  # Short address
            if src_pan_present:
                if len(pkt_data) < index + 2:
                    return None
                src_pan = reverse_bytes(pkt_data[index:index+2])
                index += 2
            if len(pkt_data) < index + 2:
                return None
            src16 = reverse_bytes(pkt_data[index:index+2])
            index += 2
        elif src_addr_mode == 0x3:  # Extended address
            if src_pan_present:
                if len(pkt_data) < index + 2:
                    return None
                src_pan = reverse_bytes(pkt_data[index:index+2])
                index += 2
            if len(pkt_data) < index + 8:
                return None
            src64 = reverse_bytes(pkt_data[index:index+8])
            src64 = bytes_to_mac(src64)
            index += 8

    match frame_type:
        case 0x00: # Beacon

            if len(pkt_data) < index + 2:
                return None
            superframe_spec = pkt_data[index : index + 2]
            index += 2
            if len(pkt_data) < index + 1:
                return None
            gts = pkt_data[index : index + 1]
            index += 1
            if len(pkt_data) < index + 1:
                return None
            pending_addrs = pkt_data[index : index + 1]
            index += 1
            data = pkt_data[index:]

            if src64:
                ap_cur = aps.get(src64)
                if not ap_cur:
                    aps[src64] = AP(src64, rssi, cur_channel)
                    ap_cur = aps[src64]
                
                if dst64:
                    ap_cur.clients.add(dst64)
                    sta_cur = stas.get(dst64)  
                    if not sta_cur:  
                        stas[dst64] = STA(dst64)
                        sta_cur = stas[dst64]
                    sta_cur.bssid = src64

            if ap_cur:
                ap_cur.manu = get_manu(ap_cur.bssid)
                ap_cur.total += 1
                ap_cur.last_seen = time.time()
                ap_cur.beacons += 1

            if sta_cur:
                sta_cur.total += 1
                sta_cur.last_seen = time.time()

        case 0x03: # Command

            if len(pkt_data) < index + 1:
                return None
            command_id = pkt_data[index]
            index += 1
            if len(pkt_data) < index + 4:
                return None
            frame_counter = pkt_data[index:index+4]
            index += 4
            if len(pkt_data) < index + 1:
                return None
            key_sequence_counter = pkt_data[index:index+1]
            index += 1
            mic_len = 8
            mic = pkt_data[-mic_len:]
            payload = pkt_data[index:-mic_len]
            payload_start = index

        case (0x01, 0x07): # Data/Extended
            if frame_version == 0x00: # 802.15.4-2003
                if seqno_suppression:
                    return None # sequence number suppression is invalid for 802.15.4-2003 and 2006
                if len(pkt_data) < index + 8:
                    return None
                
                if pkt_data[index : index + 2] == b'\x48\x02': # Zigbee Network layer data
                    frame_control = struct.unpack_from('<H', pkt_data, index)[0]
                    index += 2
                    frame_type = frame_control & 0x0003
                    protocol_versio= frame_control & 0x003C
                    discover_route = frame_control & 0x00C0
                    multicast = frame_control & 0x0100
                    security_enable = frame_control & 0x0200
                    source_route = frame_control & 0x0400
                    destination = frame_control & 0x0800
                    extended_source= frame_control & 0x1000
                    end_dev_initiator = frame_control & 0x2000

                    dst = pkt_data[index + 1: index + 2] + pkt_data[index : index + 1]
                    index += 2
                    src = pkt_data[index + 1: index + 2] + pkt_data[index : index + 1]
                    index += 2

                    radius = pkt_data[index]
                    seqno = pkt_data[index + 1]
                    index += 2

                    security_control = pkt_data[index]
                    index += 1

                    extended_source = reverse_bytes(pkt_data[index:index+8])
                    extended_source = bytes_to_mac(extended_source)
                    index += 8

                    key_sequence_no = pkt_data[index]
                    index += 1

                    mic = pkt_data[:-4]
                    pkt_data = pkt_data[index:-4]

                    payload = pkt_data
                    ap_cur = None
                    if extended_source:
                        ap_cur = aps.get(extended_source)
                        if not ap_cur:
                            aps[extended_source] = AP(extended_source, rssi, cur_channel)
                            ap_cur = aps[extended_source]
                        ap_cur.manu = get_manu(ap_cur.bssid)
                        ap_cur.data += 1
                        ap_cur.total += 1
                        ap_cur.last_seen = time.time()

                    return None

                frame_counter = pkt_data[index:index+4]
                index += 4
                if len(pkt_data) < index + 1:
                    return None
                key_sequence_counter = pkt_data[index:index+1]
                index += 1
                mic_len = 8
                mic = pkt_data[-mic_len:]
                payload = pkt_data[index:-mic_len]
                payload_start = index
                security_level = -1
                key_id_mode = -1
                frame_counter_suppression = False
                asn = -1
                key_source = b''
                key_index = -1
                security_control = 0
            else:    # v1 == 802.15.4-2006
                frame_counter_suppression=False
                key_source = b''
                key_index = b''
                security_level = 0
                security_control = 0
                key_id_mode = 0
                frame_counter = b'\x00\x00\x00\x00'

                if len(pkt_data) < index + 1:
                    return None
                security_control = pkt_data[index]
                security_level = security_control & 0x07
                key_id_mode = (security_control & 0x18) >> 3
                frame_counter_suppression = bool((security_control & 0x20) >> 5)
                asn = (security_control & 0x40) >> 6
                index += 1
                if not frame_counter_suppression:
                    if len(pkt_data) < index + 4:
                        return None
                    frame_counter = pkt_data[index:index+4]
                    index += 4

                # Parsing key identifier depending on key_id_mode
                if key_id_mode != 0:
                    if key_id_mode == 1:
                        if len(pkt_data) < index + 1:
                            return None
                        key_index = pkt_data[index]
                        index += 1
                        key_source = b''
                    elif key_id_mode in [2, 3]:
                        if len(pkt_data) < index + 4:
                            return None
                        key_source = pkt_data[index:index+4]
                        index += 4
                        if len(pkt_data) < index + 1:
                            return None
                        key_index = pkt_data[index]
                        index += 1

                # https://research.kudelskisecurity.com/2017/11/08/zigbee-security-basics-part-2/

                mic = b''
                if security_level == 0x07:
                    mic_len = 16
                elif security_level == 0x06:
                    mic_len = 8
                elif security_level == 0x05:
                    mic_len = 4
                elif security_level == 0x04:
                    mic_len = 0
                elif security_level == 0x03:
                    mic_len = 16
                elif security_level == 0x02:
                    mic_len = 8
                elif security_level == 0x01:
                    mic_len = 4
                elif security_level == 0x00:
                    mic_len = 0
                else:
                    
                    return None

                if mic_len > 0:
                    mic = pkt_data[-mic_len:]
                    payload = pkt_data[index:-mic_len]
                    payload_start = index
                else:
                    payload = pkt_data[index:]
                    payload_start = index

        case _:
            return

    
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

                # print(f"Changing channel to: {cur_channel}")

                handle.controlWrite(0x40, 210, 0, 0, bytearray([cur_channel]), TIMEOUT)
                handle.controlWrite(0x40, 210, 0, 1, b'\x00', TIMEOUT)

                start_time = datetime.now()
    except KeyboardInterrupt:
        signal_exit = True
    finally:
        handle.releaseInterface(0)
        handle.close()

def main(args):
    global outfile, oui_table, channels_list, num_channels, gpsd, aps, stas

    gpsd     = args.gpsd
    outfile  = args.outfile
    interval = args.interval
    gpsd_object = None
    oui_table = load_oui('oui.txt', False)

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

    if gpsd:
        gpsd_object = GPS()
        gps_thread = Thread(target=gpsd_object.start, args=('127.0.0.1', 2947,))
        gps_thread.daemon = True
        gps_thread.start()

    display_thread = Thread(target=display, args=(aps, stas, gpsd_object,))
    display_thread.daemon = True
    display_thread.start()

    # Run packet processing in a background thread
    packet_processor = Thread(target=process_packets, args=(gpsd_object,))
    packet_processor.daemon = True
    packet_processor.start()

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
