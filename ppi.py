
import ctypes
import struct

# Constants based on provided C definitions
PPI_80211_COMMON = 2
PPI_GEOTAG = 30002

# Define the PPI Header Structure
class ppi_hdr(ctypes.Structure):
    _fields_ = [
        ('pph_version', ctypes.c_uint8),
        ('pph_flags', ctypes.c_uint8),
        ('pph_len', ctypes.c_uint16),
        ('pph_dlt', ctypes.c_uint32)
    ]

# Define the PPI Field Header Structure
class ppi_fieldhdr(ctypes.Structure):
    _fields_ = [
        ('pfh_type', ctypes.c_uint16),
        ('pfh_datalen', ctypes.c_uint16)
    ]

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

def write_ppi_headers(file, tsfTimer, dataRate, freq, rssi, noise, gpsLat, gpsLon, gpsAlt):
    # Constants for PPI
  
    PPI_80211_COMMON = 2
    PPI_GEOTAG = 30002
    ppi_total_len = PPI_HDRLEN  # Start with PPI header length

    # Initialize PPI header
    pph = ppi_hdr()
    pph.pph_version = 0
    pph.pph_flags = 0
    pph.pph_len = PPI_HDRLEN  # To be updated
    pph.pph_dlt = 195         # DLT for HCI UART

    # Write the PPI header to the file
    file.write(struct.pack('<BBHI', pph.pph_version, pph.pph_flags, pph.pph_len, pph.pph_dlt))

    # Prepare and write the 802.11-Common PPI data header
    pfh = ppi_fieldhdr()
    pfh.pfh_type = PPI_80211_COMMON
    pfh.pfh_datalen = 20

    # Write the field header and its data
    file.write(struct.pack('<HH', pfh.pfh_type, pfh.pfh_datalen))
    file.write(struct.pack('<QH', tsfTimer, 0))         # TSF Timer + Flags (2 bytes zeroed)
    file.write(struct.pack('<H', dataRate))             # Data Rate
    file.write(struct.pack('<H', freq))                 # Frequency
    file.write(struct.pack('<H', 0x0080))               # Channel Flags (2GHz Spectrum)
    file.write(struct.pack('<BB', 0, 0))                # FHSS Hopset and FHSS Pattern
    file.write(struct.pack('<bb', rssi, noise))         # RSSI and Noise

    # Update total length to include 802.11-Common header
    ppi_total_len += PPI_FIELD_HDRLEN + pfh.pfh_datalen

    # Check if GPS data is available
    if gpsLat != 0 and gpsLon != 0:
        gpsFieldMask = 0
        fixedLat = float_to_fixed37(gpsLat)
        fixedLon = float_to_fixed37(gpsLon)
        gpsFieldMask |= 0b00000110  # Lat/Long fields present

        if gpsAlt != 0:
            fixedAlt = float_to_fixed64(gpsAlt)
            gpsFieldMask |= 0b00001000  # Altitude field present

        # Prepare and write PPI-GEOLOCATION data fields
        pfh.pfh_type = PPI_GEOTAG
        geoFhLen = 8 + 4 + 4  # Geotag Header + Lat + Lon
        if gpsAlt != 0:
            geoFhLen += 4  # Add Altitude field length

        pfh.pfh_datalen = geoFhLen
        file.write(struct.pack('<HH', pfh.pfh_type, pfh.pfh_datalen))

        # Write geotag header
        geoTagRev = 2
        geoTagPad = 0
        geoTagHeaderLen = 8 + 4 + 4  # Geotag Header + Lat + Lon
        if gpsAlt != 0:
            geoTagHeaderLen += 4  # Include Altitude

        file.write(struct.pack('<BBHI', geoTagRev, geoTagPad, geoTagHeaderLen, gpsFieldMask))
        file.write(struct.pack('<II', fixedLat, fixedLon))  # Lat/Lon in fixed-point

        if gpsAlt != 0:
            file.write(struct.pack('<I', fixedAlt))  # Altitude in fixed-point

        # Update total length to include PPI-GEOLOCATION data
        ppi_total_len += PPI_FIELD_HDRLEN + pfh.pfh_datalen

    # Update the PPI header length
    pph.pph_len = ppi_total_len

    # Seek back to write the updated PPI header length
    file.seek(-ppi_total_len, 1)    # Move back by total length
    file.seek(2, 1)                 # Move forward by 2 bytes (skip version and flags)
    file.write(struct.pack('<H', pph.pph_len))  # Update length in the header

    # Move to end of file for next writes
    file.seek(0, 2)