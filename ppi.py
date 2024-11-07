
import ctypes
import struct

# Constants based on provided C definitions
PPI_80211_COMMON = 2
PPI_GEOTAG = 30002
PPI_DLT = 195

# Constants for PPI header and field header lengths
PPI_HDRLEN = 8
PPI_FIELD_HDRLEN = 4

def float_to_fixed37(value: float) -> int:
    return int((value + 180) * 10_000_000)

def float_to_fixed64(value: float) -> int:
    return int((value + 180_000.0) * 10_000)

def calculate_ppi_header_length(gpsLat: float, gpsLon: float, gpsAlt: float) -> int:
    # Start with the base length of the PPI header
    ppi_total_len = PPI_HDRLEN

    # Add length of the 802.11-Common PPI data header
    # 20 bytes is the fixed length for 802.11-Common data fields (24 bytes total)
    ppi_total_len += PPI_FIELD_HDRLEN + 20

    # Check if GPS data is available
    if gpsLat != 0 and gpsLon != 0:
        # Add the PPI-GEOLOCATION field headers
        ppi_total_len += PPI_FIELD_HDRLEN                     # Geolocation field header
        ppi_total_len += 2 + 1 + 1 + 4  # Geo Tag header [rev (1 byte), padding (1 byte), len (2 bytes), field mask (4 bytes)]
        ppi_total_len += 2 * 4          # Lat and Lon (each 4 bytes)

        # Add Altitude field if gpsAlt is not zero
        if gpsAlt != 0:
            ppi_total_len += 4          # Altitude field length (4 bytes)

    return ppi_total_len

def write_ppi_headers(tsf_timer, data_rate, freq, rssi, noise, gps_lat, gps_lon, gps_alt):

    # Construct the PPI data fields
    ppi_data = b''

    # Construct 802.11-Common PPI data header
    ppi_radio_tap = b''
    ppi_radio_tap += struct.pack('<HH', 2, 20) # 2 is the 802.11-Common tag, 20 is the length of the data
    # TSF Timer, Flags, Rate, Channel-Freq, Channel-Flags, FHSS-Hopset, FHSS-Pattern, dBm-Antsignal, dBm-Antnoise
    ppi_radio_tap += struct.pack('<QHHHHHbb', tsf_timer, 0, data_rate, freq, 0x0080, 0, rssi, noise)
    ppi_data += ppi_radio_tap

    # Construct the PPI-GEOLOCATION data fields
    gps_data = b''
    gps_field_mask = 0

    if(gps_lat and gps_lon):
        gps_data += struct.pack('<I', float_to_fixed37(gps_lat))
        gps_data += struct.pack('<I', float_to_fixed37(gps_lon))
        gps_field_mask |= 0b00000110 # Lat/Long fields present
        # Don't care about altitude if no coords
        if(gps_alt):
            gps_data += struct.pack('<I', float_to_fixed64(gps_alt))
            gps_field_mask |= 0b00001000 # Altitude field present

    if(len(gps_data)):
        gps_header = struct.pack('<BBHI', 2, 0, 8+len(gps_data), gps_field_mask) # base_geotag_header
        gps_data = gps_header + gps_data
        # print(gpsData)
        ppi_data += struct.pack('<HH', PPI_GEOTAG, len(gps_data)) # 30002 is the PPI Vendor Tag for PPI-GEOLOCATION
        ppi_data += gps_data

    # Construct the PPI packet header
    ppi_len = 8 + len(ppi_data)
    ppi_ph = struct.pack('<BBHI', 0, 0, ppi_len, PPI_DLT) # version, flags, length, encapsulated DLT

    # print('ppi header length after composing: ', len(ppi_ph + ppi_data))

    return ppi_ph + ppi_data
