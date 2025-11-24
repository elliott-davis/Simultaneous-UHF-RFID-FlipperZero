
from __future__ import print_function
from datetime import datetime
import serial
import mercury
import argparse
import glob
import sys
import logging
import string
import struct
import time

# Setup logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("uhf_reader.log")
    ]
)

# Defaults
DEFAULT_READER_PORT = 'tmr:///dev/serial0'
DEFAULT_FLIPPER_PORT = "/dev/ttySOFT0"
DEFAULT_BAUDRATE = 9600

# Global variables
reader_virtual_serial = DEFAULT_READER_PORT
serial_port = DEFAULT_FLIPPER_PORT
baud_rate = DEFAULT_BAUDRATE

reader_power = 1500
connected = False
reader = "undefined"

#Lists for the epcs, reserved, and user memory
epc_list = []
res_list = []
user_mem_list = []


#Extracts the string for the EPC value
def parse_epc_string(s, epc_length=24):
    epc_start = 0 
    prefix = s[:epc_start].strip()
    old_epc = s[epc_start:epc_start + epc_length]
    new_epc = s[epc_start + epc_length:].strip()
    return prefix, old_epc, new_epc[0:epc_length]

#Extracts the user memory value
def parse_mem_string(s, epc_length=40):
    epc_start = 0
    prefix = s[:epc_start].strip()
    old_epc = s[epc_start:epc_start + 24]
    new_epc = s[24:].strip()
    return prefix, old_epc, new_epc


#Parses the reserved memory string 
def parse_res_string(s, epc_length=40):
    epc_start = 0
    prefix = s[:epc_start].strip()
    old_epc = s[epc_start:epc_start + 24]
    new_epc = s[24:].strip()
    return prefix, old_epc, new_epc

#Parses the TID string 
def parse_tid_string(s, epc_length=40):
    epc_start = 0
    prefix = s[:epc_start].strip()
    old_epc = s[epc_start:epc_start + 24]
    new_epc = s[24:].strip()
    return prefix, old_epc, new_epc


#Connects to the reader 
def reader_connect():
    global reader
    logging.info(f"Attempting to connect to reader at {reader_virtual_serial}")
    if(reader == "undefined"):
        try:
            reader = mercury.Reader(reader_virtual_serial, baudrate=115200)
            reader.set_region("NA")
            # Configure read plan to also read TID, User, and Reserved memory banks during inventory
            # This is MUCH more reliable than separate read_tag_mem calls
            reader.set_read_plan([1], "GEN2", read_power=reader_power, bank=["tid", "user", "reserved"])
            logging.info("Successfully connected to reader (with TID/User/Reserved bank reads enabled)")
            return True
        except Exception as e:
            logging.error(f"Error Connecting to reader: {e}", exc_info=True)
            return False
    else:
        logging.info("Reader already defined")
        return True


#Formats the string to begin with a b
def extract_content_from_format(input_string):
    if input_string.startswith("b'") and input_string.endswith("'") and len(input_string) > 3:
        return input_string[2:-1]
    else:
        return ""

def clean_hex_input(value):
    if not value:
        return ""

    trimmed_value = value.strip()

    if trimmed_value.startswith("b'") and trimmed_value.endswith("'") and len(trimmed_value) > 2:
        trimmed_value = trimmed_value[2:-1]

    if trimmed_value.endswith('b') and len(trimmed_value) % 2 != 0:
        trimmed_value = trimmed_value[:-1]

    return trimmed_value

def to_readonly_bytes(value):
    if value is None:
        return None
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, memoryview):
        return value.tobytes()
    return value

def normalize_epc_bytes(epc_value):
    if isinstance(epc_value, bytearray):
        return to_readonly_bytes(epc_value)

    if isinstance(epc_value, bytes):
        try:
            ascii_candidate = epc_value.decode('ascii').strip()
            if (
                ascii_candidate
                and len(ascii_candidate) % 2 == 0
                and all(char in string.hexdigits for char in ascii_candidate)
            ):
                return to_readonly_bytes(bytearray.fromhex(ascii_candidate))
        except UnicodeDecodeError:
            pass
        return to_readonly_bytes(epc_value)

    if isinstance(epc_value, str):
        cleaned = clean_hex_input(epc_value)
        try:
            return to_readonly_bytes(bytearray.fromhex(cleaned))
        except ValueError:
            logging.error(f"Failed to normalize EPC string: {epc_value}")
            return None

    logging.error(f"Unsupported EPC type for normalization: {type(epc_value)}")
    return None

def strip_pc_prefix(epc_bytes):
    if epc_bytes is None:
        return None
    if len(epc_bytes) <= 2:
        return b""
    return epc_bytes[2:]

def audit_visible_epcs(label=""):
    """Utility to log currently visible EPCs for debugging write issues."""
    if reader == "undefined":
        logging.debug("Reader undefined; skipping EPC audit")
        return []

    try:
        tags = reader.read(timeout=500)
        epc_payloads = []
        for tag in tags:
            normalized = normalize_epc_bytes(tag.epc)
            if normalized:
                epc_payloads.append(to_readonly_bytes(normalized))
        logging.info(f"{label} Visible tags: {len(epc_payloads)}")
        for idx, epc in enumerate(epc_payloads, 1):
            logging.info(f"{label} Tag {idx}: {epc.hex().upper()} ({len(epc)} bytes)")
        return epc_payloads
    except Exception as e:
        logging.warning(f"{label} Failed to audit EPCs: {e}")
        return []

def reset_pc_to_match_epc(match_epc_bytes):
    """Force the PC word to reflect the currently reported EPC length."""
    if reader == "undefined" or not match_epc_bytes:
        return False

    current_words = len(match_epc_bytes) // 2
    if current_words == 0:
        logging.warning("reset_pc_to_match_epc: zero-length EPC")
        return False

    pc_val = (current_words << 11)
    pc_bytes = struct.pack('>H', pc_val)
    payload = bytearray(pc_bytes + match_epc_bytes)
    filter_dict = {'epc': match_epc_bytes.hex().encode('ascii')}

    logging.info(
        f"Resetting PC word to match current EPC length {current_words} words "
        f"(PC=0x{pc_val:04X})"
    )

    try:
        if reader.write_tag_mem(1, 0x01, payload, filter_dict):
            return True
    except Exception as e:
        logging.warning(f"reset_pc_to_match_epc (targeted) failed: {e}")

    logging.info("Retrying PC reset with wildcard filter")
    wildcard_filter = {'epc': b''}
    try:
        return reader.write_tag_mem(1, 0x01, payload, wildcard_filter)
    except Exception as e:
        logging.warning(f"reset_pc_to_match_epc (wildcard) failed: {e}")
        return False


def describe_epc_layout(match_epc_bytes, label="", attempt_repair=False):
    """Read PC word + current EPC payload for the supplied tag filter."""
    if reader == "undefined":
        logging.debug("Reader undefined; cannot describe EPC layout")
        return None
    if match_epc_bytes is None:
        logging.debug("describe_epc_layout called without match_epc_bytes")
        return None

    filter_dict = {'epc': match_epc_bytes.hex().encode('ascii')}

    try:
        pc_bytes = reader.read_tag_mem(1, 0x01, 2, filter_dict)
        if not pc_bytes or len(pc_bytes) < 2:
            logging.warning(f"{label} Unable to read PC word for tag {match_epc_bytes.hex().upper()}")
            return None

        pc_val = struct.unpack('>H', to_readonly_bytes(pc_bytes))[0]
        length_words = (pc_val >> 11) & 0x1F
        payload_len_bytes = min(length_words * 2, 64)
        payload_bytes = None
        if payload_len_bytes:
            payload_bytes = reader.read_tag_mem(1, 0x02, payload_len_bytes, filter_dict)

        info = {
            "pc_val": pc_val,
            "length_words": length_words,
            "payload_len_bytes": payload_len_bytes,
            "payload_hex": payload_bytes.hex().upper() if payload_bytes else None,
        }

        logging.info(
            f"{label} Tag {match_epc_bytes.hex().upper()} PC=0x{pc_val:04X} "
            f"len_words={length_words} payload={info['payload_hex']}"
        )
        return info
    except Exception as e:
        msg = str(e).lower()
        logging.warning(f"{label} Failed to inspect EPC layout: {e}")
        if attempt_repair and "bad pc" in msg:
            if reset_pc_to_match_epc(match_epc_bytes):
                logging.info(f"{label} PC repair successful; retrying layout inspection")
                return describe_epc_layout(match_epc_bytes, label, attempt_repair=False)
        return None

def auto_detect_current_epc(max_attempts=5, timeout=500):
    if reader == "undefined":
        logging.warning("Reader is not initialized; cannot auto-detect EPC")
        return None

    for attempt in range(max_attempts):
        try:
            tags = reader.read(timeout=timeout)
        except Exception as e:
            logging.warning(f"Auto-detect attempt {attempt + 1} failed: {e}")
            continue

        if not tags:
            continue

        if len(tags) > 1:
            logging.warning("Multiple tags detected during auto-detect; unable to determine unique target")
            return None

        resolved = normalize_epc_bytes(tags[0].epc)
        if resolved is None:
            logging.error("Auto-detect could not normalize EPC bytes")
            return None

        # Mercury API tag.epc is usually just the EPC payload. 
        # We should NOT strip bytes unless we are sure they are PC.
        # Assuming tag.epc is the full EPC.
        payload = resolved

        logging.info(
            f"Auto-detected current tag EPC: {payload.hex().upper()} "
            f"({len(payload)} bytes)"
        )
        return to_readonly_bytes(payload)

    logging.warning("Auto-detect timed out without seeing a single tag")
    return None

def verify_epc_write(expected_epc_bytes, attempts=5, delay=0.3):
    """Re-read the EPC bank to ensure the data on-tag matches what we wrote."""
    if reader == "undefined":
        logging.warning("Reader is not initialized; cannot verify EPC write")
        return False

    if not expected_epc_bytes:
        logging.error("Verification requested without expected EPC bytes")
        return False

    read_len = len(expected_epc_bytes)
    expected_hex = expected_epc_bytes.hex().upper()
    filter_dict = {'epc': expected_epc_bytes.hex().encode('ascii')}
    
    logging.debug(f"Verifying EPC write. Expected: {expected_hex} ({read_len} bytes)")

    for attempt in range(attempts):
        # Method 1: Try to read with filter matching new EPC
        try:
            read_back = reader.read_tag_mem(1, 0x02, read_len, filter_dict)
            if read_back is None:
                logging.debug(f"EPC verification attempt {attempt + 1}: no tag matched new EPC filter")
            else:
                normalized = to_readonly_bytes(read_back)
                if normalized == expected_epc_bytes:
                    logging.info(f"EPC write verified on attempt {attempt + 1}")
                    return True
                logging.debug(
                    f"EPC verification attempt {attempt + 1} mismatch: "
                    f"expected {expected_hex} got {normalized.hex().upper()}"
                )
        except Exception as e:
            logging.warning(f"EPC verification attempt {attempt + 1} failed: {e}")
        
        # Method 2: Fallback - do a fresh inventory and check if new EPC appears
        try:
            tags = reader.read(timeout=300)
            for tag in tags:
                tag_epc = normalize_epc_bytes(tag.epc)
                if tag_epc:
                    tag_hex = tag_epc.hex().upper()
                    logging.debug(f"Verification inventory found tag: {tag_hex}")
                    if tag_epc == expected_epc_bytes:
                        logging.info(f"EPC write verified via inventory on attempt {attempt + 1}")
                        return True
        except Exception as e:
            logging.debug(f"Verification inventory failed: {e}")
            
        time.sleep(delay)

    logging.warning("Unable to verify EPC write after multiple attempts")
    audit_visible_epcs("[Verify Failure]")
    return False

def try_write_epc_bytes(new_epc_bytes, target_epc):
    if not new_epc_bytes:
        logging.error("No EPC data provided for write operation")
        return False

    if len(new_epc_bytes) % 2 != 0:
        logging.error("EPC data length must be a multiple of 2 bytes")
        return False

    # EPC bank layout: word 0 = CRC (read-only), word 1 = PC, word 2+ = EPC payload
    # We construct a payload that includes the PC word to ensure the length is correctly set.
    
    epc_len_words = len(new_epc_bytes) // 2
    # Standard PC word calculation: length in words shifted left by 11 bits
    pc_val = (epc_len_words << 11)
    
    desired_words = len(new_epc_bytes) // 2

    # First try the high-level write helper (targeted if possible)
    # CRITICAL: reader.write() expects HEX STRINGS, not raw bytes!
    # The mercury API uses TMR_hexToBytes() internally which parses ASCII hex chars.
    new_epc_hex = new_epc_bytes.hex().upper()
    target_epc_hex = target_epc.hex().upper() if target_epc else None
    
    logging.debug(f"reader.write params: epc_code={new_epc_hex}, epc_target={target_epc_hex}")
    
    try:
        if reader.write(epc_code=new_epc_hex, epc_target=target_epc_hex):
            logging.info(
                f"reader.write ({'targeted' if target_epc else 'untargeted'}) succeeded"
            )
            audit_visible_epcs("[Post reader.write]")
            return True
    except Exception as e:
        logging.warning(f"reader.write ({'targeted' if target_epc else 'untargeted'}) failed: {e}")

    if target_epc is None:
        try:
            if reader.write(epc_code=new_epc_hex, epc_target=None):
                logging.info("reader.write (untargeted fallback) succeeded")
                audit_visible_epcs("[Post reader.write fallback]")
                return True
        except Exception as e:
            logging.warning(f"reader.write (untargeted) failed: {e}")

    try:
        layout = None
        if target_epc:
            layout = describe_epc_layout(target_epc, "[Pre-Write] Layout check", attempt_repair=True)
            if layout:
                current_words = layout.get("length_words", 0)
                if current_words and desired_words > current_words:
                    logging.warning(
                        f"Target tag currently advertises {current_words} words "
                        f"but requested EPC needs {desired_words} words"
                    )

        pc_bytes = struct.pack('>H', pc_val)
        full_payload = bytearray(pc_bytes + new_epc_bytes)
        
        logging.debug(
            f"Writing PC+EPC starting at word offset 1. "
            f"PC: {pc_bytes.hex().upper()}, EPC: {new_epc_bytes.hex().upper()}"
        )
        
        # Strategy 1: Targeted write using Gen2 Select (Dict)
        if target_epc:
            filter_hex_bytes = target_epc.hex().encode('ascii')
            filter_dict = {'epc': filter_hex_bytes}
            for i in range(3):
                try:
                    logging.debug(f"Using Filter (Dict): {filter_dict} (Attempt {i+1})")
                    if reader.write_tag_mem(1, 0x01, full_payload, filter_dict):
                        return True
                    time.sleep(0.1)
                except Exception as e:
                    logging.warning(f"write_tag_mem (targeted dict) failed: {e}")
                    break

        # Strategy 2: Untargeted write using no filter (writes to first responding tag)
        logging.info("Attempting untargeted write_tag_mem (no filter)")
        try:
            # Pass None for filter to write to first responding tag
            if reader.write_tag_mem(1, 0x01, full_payload, None):
                logging.info("write_tag_mem (untargeted) succeeded")
                return True
        except Exception as e:
            logging.warning(f"write_tag_mem (untargeted) failed: {e}")
    
    except AttributeError:
        logging.debug("write_tag_mem not available; falling back to reader.write (already attempted)")
    except Exception as e:
        logging.warning(f"write_tag_mem (PC+EPC) setup failed: {e}")

    logging.error("All EPC write strategies failed")
    return False

#Format the file to length X
def format_byte_array_to_X_bytes(byte_array, x):
    input_length = len(byte_array)
    
    if input_length == x:
        return byte_array.hex()
    
    elif input_length < x:
        padding_length = x - input_length
        padded_byte_array = byte_array + b'\x00' * padding_length
        return padded_byte_array.hex()
    
    else:
        truncated_byte_array = byte_array[:x]
        return truncated_byte_array.hex()
    

#Function to handle the serial commands 
def handle_command(command, ser):
    global connected
    global epc_list
    global res_list
    global user_mem_list
    
    logging.debug(f"Processing command: {command}")

    #Handle the write command for EPCs
    if command == "WRITE":
       
        if connected:
            # The Flipper sends the target (old) EPC on one line, and the new EPC on the next line.
            try:
                raw_old_epc = ser.readline().decode().strip()
                raw_new_epc = ser.readline().decode().strip()
                
                logging.debug(f"WRITE Target EPC: {raw_old_epc}")
                logging.debug(f"WRITE New EPC: {raw_new_epc}")
                
                new_epc_str = clean_hex_input(raw_new_epc)
                old_epc_str = clean_hex_input(raw_old_epc)
                
                if not new_epc_str:
                    logging.error("New EPC is empty")
                    ser.write(b"EVBAD\n")
                    return

                try:
                    new_epc = to_readonly_bytes(bytearray.fromhex(new_epc_str))
                except ValueError as e:
                    logging.error(f"New EPC hex conversion error: {e}")
                    ser.write(b"EVBAD\n")
                    return
                logging.info(f"New EPC request: {new_epc.hex().upper()} ({len(new_epc)} bytes)")

                supplied_target = None
                if old_epc_str:
                    try:
                        supplied_target = to_readonly_bytes(bytearray.fromhex(old_epc_str))
                    except ValueError:
                        logging.warning("Target EPC provided but not valid hex; ignoring it")

                use_supplied_target = supplied_target is not None and old_epc_str.lower() != new_epc_str.lower()

                if use_supplied_target:
                    target_epc = supplied_target
                    logging.debug("Using supplied target EPC filter for write operation")
                else:
                    if supplied_target is None and old_epc_str:
                        logging.info("Target EPC was invalid; attempting to auto-detect the current tag")
                    elif old_epc_str:
                        logging.info("Target EPC matches desired EPC; attempting to detect the current tag automatically")
                    else:
                        logging.info("No target EPC supplied; attempting to auto-detect the current tag")

                    target_epc = auto_detect_current_epc()
                    if target_epc is None:
                        logging.info("Auto-detect unavailable; proceeding without an EPC filter (writes the first responsive tag)")
                    else:
                        target_epc = to_readonly_bytes(target_epc)
                        logging.info(
                            f"Auto-detected tag length {len(target_epc)} bytes "
                            f"EPC: {target_epc.hex().upper()}"
                        )

                write_attempts = []
                if target_epc is not None:
                    describe_epc_layout(target_epc, "[Pre-Write] Current tag snapshot", attempt_repair=True)
                    write_attempts.append(("targeted", target_epc))
                    if not use_supplied_target:
                        write_attempts.append(("untargeted", None))
                else:
                    write_attempts.append(("untargeted", None))

                success = False
                for mode, candidate in write_attempts:
                    logging.info(f"Attempting single-shot EPC write ({mode})")
                    if try_write_epc_bytes(new_epc, candidate):
                        success = True
                        break

                if success:
                    if verify_epc_write(new_epc):
                        ser.write(b"EVOK\n")
                        logging.info("Write EPC successful and verified")
                    else:
                        ser.write(b"EVBAD\n")
                        logging.warning("Write EPC completed but verification failed")
                else:
                    ser.write(b"EVBAD\n")
                    logging.warning("Write EPC failed or cancelled")

            except Exception as e:
                logging.error(f"Exception during WRITE: {e}", exc_info=True)
                ser.write(b"EVBAD\n")
        else:
            logging.warning("WRITE command received but reader not connected")
    
    #Handle the write TID command
    elif command == "WRITETID":
       
        if connected:
            try:
                old_epc_str = ser.readline().decode().strip()
                new_data_str = ser.readline().decode().strip()
                logging.debug(f"WRITETID Target: {old_epc_str}, Data: {new_data_str}")
                
                if not new_data_str:
                     logging.error("New TID data is empty")
                     ser.write(b"TVBAD\n")
                     return

                if old_epc_str.endswith('b') and len(old_epc_str) % 2 != 0: old_epc_str = old_epc_str[:-1]
                if new_data_str.endswith('b') and len(new_data_str) % 2 != 0: new_data_str = new_data_str[:-1]

                # WRITETID logic for magic/cloneable TID tags
                old_epc_hex_bytes = old_epc_str.encode('ascii')
                filter_dict = {'epc': old_epc_hex_bytes}
                full_tid_data = bytearray.fromhex(new_data_str)
                
                logging.info(f"TID write requested: {len(full_tid_data)} bytes")
                
                success = False
                
                # Strategy 1: Try writing first 12 bytes (most common critical portion) at offset 0
                # Magic tags often only have 8-12 bytes of writable TID
                tid_12 = full_tid_data[:12] if len(full_tid_data) >= 12 else full_tid_data
                logging.info(f"Strategy 1: Writing first 12 bytes at offset 0: {tid_12.hex().upper()}")
                try:
                    success = reader.write_tag_mem(2, 0x00, tid_12, filter_dict)
                    if success:
                        logging.info("TID write succeeded with 12 bytes at offset 0")
                except Exception as e:
                    logging.warning(f"Strategy 1 failed: {e}")
                
                # Strategy 2: Try writing first 8 bytes at offset 0
                if not success:
                    tid_8 = full_tid_data[:8] if len(full_tid_data) >= 8 else full_tid_data
                    logging.info(f"Strategy 2: Writing first 8 bytes at offset 0: {tid_8.hex().upper()}")
                    try:
                        success = reader.write_tag_mem(2, 0x00, tid_8, filter_dict)
                        if success:
                            logging.info("TID write succeeded with 8 bytes at offset 0")
                    except Exception as e:
                        logging.warning(f"Strategy 2 failed: {e}")
                
                # Strategy 3: Try writing at word offset 2 (byte 4) - skip manufacturer header
                # Some magic tags have read-only first 4 bytes
                if not success and len(full_tid_data) > 4:
                    tid_after_header = full_tid_data[4:16] if len(full_tid_data) >= 16 else full_tid_data[4:]
                    logging.info(f"Strategy 3: Writing {len(tid_after_header)} bytes at offset 4 (skip header): {tid_after_header.hex().upper()}")
                    try:
                        success = reader.write_tag_mem(2, 0x04, tid_after_header, filter_dict)
                        if success:
                            logging.info("TID write succeeded at offset 4")
                    except Exception as e:
                        logging.warning(f"Strategy 3 failed: {e}")
                
                # Strategy 4: Try without filter (single tag in field)
                if not success:
                    logging.info("Strategy 4: Trying without EPC filter...")
                    for size in [12, 8, 4]:
                        tid_chunk = full_tid_data[:size] if len(full_tid_data) >= size else full_tid_data
                        logging.info(f"Strategy 4: Writing {size} bytes without filter: {tid_chunk.hex().upper()}")
                        try:
                            success = reader.write_tag_mem(2, 0x00, tid_chunk, None)
                            if success:
                                logging.info(f"TID write succeeded with {size} bytes (no filter)")
                                break
                        except Exception as e:
                            logging.warning(f"Strategy 4 ({size} bytes) failed: {e}")

                if success:
                    ser.write(b"TVOK\n")
                    logging.info("Write TID successful")
                else:
                    ser.write(b"TVBAD\n")
                    logging.warning("Write TID failed - all strategies exhausted")

            except Exception as e:
                logging.error(f"Exception during WRITETID: {e}", exc_info=True)
                ser.write(b"TVBAD\n")

    #Handle the Write User Memory Command
    elif command == "WRITEUSR":
        if connected:
            try:
                old_epc_str = ser.readline().decode().strip()
                new_data_str = ser.readline().decode().strip()
                logging.debug(f"WRITEUSR Target: {old_epc_str}, Data: {new_data_str}")

                if not new_data_str:
                     logging.error("New User Memory data is empty")
                     ser.write(b"UVBAD\n")
                     return

                # WRITEUSR logic
                if old_epc_str.endswith('b') and len(old_epc_str) % 2 != 0: old_epc_str = old_epc_str[:-1]
                if new_data_str.endswith('b') and len(new_data_str) % 2 != 0: new_data_str = new_data_str[:-1]
                
                old_epc_hex_bytes = old_epc_str.encode('ascii')
                filter_dict = {'epc': old_epc_hex_bytes}
                new_data = bytearray.fromhex(new_data_str)
            
                logging.info("Attempting single-shot User Memory write")
                try:
                    # write_tag_mem(bank, addr, data, filter)
                    success = reader.write_tag_mem(3, 0x00, new_data, filter_dict)
                except Exception as e:
                    logging.error(f"Reader raised during User Memory write: {e}")
                    success = False

                if success:
                    ser.write(b"UVOK\n")
                    logging.info("Write User Memory successful")
                else:
                    ser.write(b"UVBAD\n")
                    logging.warning("Write User Memory failed or cancelled")

            except Exception as e:
                logging.error(f"Exception during WRITEUSR: {e}", exc_info=True)
                ser.write(b"UVBAD\n")

        else:
            logging.warning("WRITEUSR command received but reader not connected")
    
    #Handle the write Reserved Memory Command
    elif command == "WRITERES":
       
        if connected:
            try:
                old_epc_str = ser.readline().decode().strip()
                new_data_str = ser.readline().decode().strip()
                logging.debug(f"WRITERES Target: {old_epc_str}, Data: {new_data_str}")
                
                if not new_data_str:
                     logging.error("New Reserved Memory data is empty")
                     ser.write(b"RVBAD\n")
                     return

                # WRITERES logic
                if old_epc_str.endswith('b') and len(old_epc_str) % 2 != 0: old_epc_str = old_epc_str[:-1]
                if new_data_str.endswith('b') and len(new_data_str) % 2 != 0: new_data_str = new_data_str[:-1]

                old_epc_hex_bytes = old_epc_str.encode('ascii')
                filter_dict = {'epc': old_epc_hex_bytes}
                new_data = bytearray.fromhex(new_data_str)
                
                logging.info("Attempting single-shot Reserved Memory write (bank 0)")
                try:
                    # write_tag_mem(bank, addr, data, filter)
                    success = reader.write_tag_mem(0, 0x00, new_data, filter_dict)
                except Exception as e:
                    logging.error(f"Reader raised during Reserved Memory write: {e}")
                    success = False

                if success:
                    ser.write(b"RVOK\n")
                    logging.info("Write Reserved Memory successful")
                else:
                    ser.write(b"RVBAD\n")
                    logging.warning("Write Reserved Memory failed or cancelled")

            except Exception as e:
                logging.error(f"Exception during WRITERES: {e}", exc_info=True)
                ser.write(b"RVBAD\n")

        else:
            logging.warning("WRITERES command received but reader not connected")
    
    #Handle the Read Command
    elif command == "READ":
        if connected:
            try:
                # Read tags with memory banks included (configured in read_plan with bank parameter)
                logging.debug("Starting read operation (with embedded memory bank reads)...")
                epc_list = []
                tid_list = []
                res_list = []
                user_mem_list = []
                
                # Store raw tag objects so we can extract memory data
                raw_tag_data = []

                while len(epc_list) == 0:
                    if ser.inWaiting() > 0:
                        logging.info("Incoming command detected, stopping read loop")
                        break

                    raw_tags = reader.read(timeout=500)
                    for tag in raw_tags:
                        normalized = normalize_epc_bytes(tag.epc)
                        payload = normalized
                        if payload:
                            epc_list.append(to_readonly_bytes(payload))
                            raw_tag_data.append(tag)
                
                logging.info(f"Read {len(epc_list)} tags")
                
                # Extract memory data from tag objects (read during inventory - much more reliable!)
                for idx, tag in enumerate(raw_tag_data):
                    epc = epc_list[idx]
                    logging.debug(f"Processing memory for EPC: {epc.hex().upper()}")
                    
                    # Get TID from tag object (read during inventory)
                    input_byte_array = None
                    if hasattr(tag, 'tid_mem_data') and tag.tid_mem_data is not None and len(tag.tid_mem_data) > 0:
                        input_byte_array = bytes(tag.tid_mem_data)
                        logging.info(f"TID from inventory: {input_byte_array.hex().upper()}")
                    else:
                        logging.warning(f"TID not available from inventory for {epc.hex().upper()}")
                        # Fallback: try direct read (may fail due to timing)
                        try:
                            input_byte_array = reader.read_tag_mem(2, 0x00, 20, None)
                            if input_byte_array:
                                logging.info(f"TID from fallback read: {input_byte_array.hex().upper()}")
                        except Exception as e:
                            logging.warning(f"TID fallback read failed: {e}")
                    
                    # Get Reserved memory from tag object
                    input_reserved_arr = None
                    if hasattr(tag, 'reserved_mem_data') and tag.reserved_mem_data is not None and len(tag.reserved_mem_data) > 0:
                        input_reserved_arr = bytes(tag.reserved_mem_data)
                        logging.info(f"Reserved from inventory: {input_reserved_arr.hex().upper()}")
                    else:
                        logging.warning(f"Reserved not available from inventory for {epc.hex().upper()}")
                        try:
                            input_reserved_arr = reader.read_tag_mem(0, 0x00, 8, None)
                            if input_reserved_arr:
                                logging.info(f"Reserved from fallback read: {input_reserved_arr.hex().upper()}")
                        except Exception as e:
                            logging.warning(f"Reserved fallback read failed: {e}")
                    
                    # Get User memory from tag object
                    input_user_mem_arr = None
                    if hasattr(tag, 'user_mem_data') and tag.user_mem_data is not None and len(tag.user_mem_data) > 0:
                        input_user_mem_arr = bytes(tag.user_mem_data)
                        logging.info(f"User memory from inventory: {input_user_mem_arr.hex().upper()}")
                    else:
                        logging.warning(f"User memory not available from inventory for {epc.hex().upper()}")
                        try:
                            input_user_mem_arr = reader.read_tag_mem(3, 0x00, 64, None)
                            if input_user_mem_arr:
                                logging.info(f"User memory from fallback read: {input_user_mem_arr.hex().upper()}")
                        except Exception as e:
                            logging.warning(f"User memory fallback read failed: {e}")
                    
                    # Log a summary of all memory banks for this tag
                    logging.info(f"=== MEMORY SUMMARY for EPC {epc.hex().upper()} ===")
                    logging.info(f"  TID (Bank 2):      {input_byte_array.hex().upper() if input_byte_array else 'FAILED TO READ'}")
                    logging.info(f"  Reserved (Bank 0): {input_reserved_arr.hex().upper() if input_reserved_arr else 'FAILED TO READ'}")
                    logging.info(f"  User (Bank 3):     {input_user_mem_arr.hex().upper() if input_user_mem_arr else 'FAILED TO READ'}")
                    logging.info(f"===============================================")

                    # Format and store results (fill with zeros if read failed)
                    if input_byte_array is not None:
                        formatted_byte_array_str = format_byte_array_to_X_bytes(input_byte_array, 20)
                        tid_list.append(formatted_byte_array_str)
                    else:
                        tid_list.append("0000000000000000000000000000000000000000")
                    
                    if input_reserved_arr is not None:
                        formatted_byte_array_res_str = format_byte_array_to_X_bytes(input_reserved_arr, 8)
                        res_list.append(formatted_byte_array_res_str)
                    else:
                        res_list.append("0000000000000000")
                                                      
                    if input_user_mem_arr is not None:
                        formatted_byte_array_mem_str = format_byte_array_to_X_bytes(input_user_mem_arr, 16)
                        user_mem_list.append(formatted_byte_array_mem_str)
                    else:
                        user_mem_list.append("00000000000000000000000000000000")
                
                
                ser.write(b"TID\n") 
                number_of_tags_TID = (str(len(tid_list)) + "\n").encode()
                ser.write(number_of_tags_TID)   
                
                for tid in tid_list:
                    tid = tid + "\n"
                    logging.debug(f"Sending TID: {tid.strip()}")
                    tid = tid.encode()
                    ser.write(tid)
                ser.write(b"end\n") 

            except Exception as e:
                logging.error(f"Error Reading: {e}", exc_info=True)
        else:
            logging.warning("READ command received but reader not connected")
        
        logging.debug("Read operation finished")

    #Handle the power command
    elif command == "POWER":
        value = ser.readline().decode().strip()
        logging.debug(f"POWER command value: {value}")
        if connected:
            try:
                # Preserve the bank reads when changing power
                reader.set_read_plan([1], "GEN2", read_power=int(value), bank=["tid", "user", "reserved"])
                logging.info(f"Power set to {value} (with TID/User/Reserved bank reads)")
            except Exception as e:
                logging.error(f"Error setting reader power: {e}", exc_info=True)
        else:
            logging.warning("POWER command received but reader not connected")

    #Send the EPCS to the flipper app   
    elif command == "EPCS":
        logging.debug("Sending EPCs to Flipper")
        ser.write(b"TR\n")
        number_of_tags_read = (str(len(epc_list)) + "\n").encode()
        ser.write(number_of_tags_read)
        epc_values = [epc.hex().upper() for epc in epc_list]
        
        for epc in epc_values:
            payload = (epc + "b\n").encode()
            ser.write(payload)
        ser.write(b"end\n")   
        epc_list = []
    
    #Send the reserved memory to the flipper app
    elif command == "RES":
        logging.debug("Sending Reserved Memory to Flipper")
        ser.write(b"RES\n")
        number_of_tags_read = (str(len(res_list)) + "\n").encode()
        ser.write(number_of_tags_read)
        res_values = [str(res) for res in res_list]
        
        for res in res_values:
            res = res + "\n"
            res = res.encode()
            ser.write(res)
        
        ser.write(b"end\n")   
        res_list = []

    #Send the user memory to the flipper app
    elif command == "MEM":
        logging.debug("Sending User Memory to Flipper")
        ser.write(b"MEM\n")
        number_of_tags_read = (str(len(user_mem_list)) + "\n").encode()
        ser.write(number_of_tags_read)
        mem_values = [str(mem) for mem in user_mem_list]
        
        for mem in mem_values:
            mem = mem + "\n"
            mem = mem.encode()
            ser.write(mem)
            
        ser.write(b"end\n")   
        user_mem_list = []
    #This is just a place holder for now... 
    elif command == "External": 
        logging.debug("got a external")
    
    #This is just a place holder for now... 
    elif command == "Internal":
        logging.debug("got a internal")
    
    #Connect to the reader
    elif command == "C": 
        connected = reader_connect()
        if connected:
            ser.write(b"CONOK\n")
        else:
            ser.write(b"CONBAD\n") # Added NAK for failure
    
    #Disconnect from the reader
    elif command == "D":
        connected = False
        ser.write(b"DISOK\n")
        logging.info("Disconnected from reader")

    # Handle the done command to advance Flipper state machine
    elif command == "done":
        logging.debug("Received done from Flipper, sending acknowledgement")
        ser.write(b"\n")

    else:
        ser.write(b"unknown command\n")
        logging.warning(f"Unknown command: {command}")
        
        


def parse_arguments():
    parser = argparse.ArgumentParser(description='UHF RFID Reader Bridge')
    parser.add_argument('--reader-port', help='Serial port for RFID Reader (e.g. /dev/ttyUSB0)')
    parser.add_argument('--flipper-port', help='Serial port for Flipper Zero (e.g. /dev/serial0)')
    parser.add_argument('--baudrate', type=int, help='Baud rate for Flipper connection')
    parser.add_argument('--usb', action='store_true', help='Use USB reader mode (auto-detects USB port, uses /dev/serial0 for Flipper)')
    return parser.parse_args()

#Main function for the program that handles the M6E Nano Reader and communicates back with the flipper zero
def main():
    global reader_virtual_serial, serial_port, baud_rate

    args = parse_arguments()

    if args.usb:
        # Try to find USB device
        usb_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if usb_ports:
            logging.info(f"Found USB device: {usb_ports[0]}")
            reader_virtual_serial = f"tmr://{usb_ports[0]}"
            if not args.flipper_port:
                serial_port = "/dev/serial0"
            if not args.baudrate:
                baud_rate = DEFAULT_BAUDRATE 
        else:
            logging.error("Error: No USB serial device found for --usb mode")
            sys.exit(1)
    
    if args.reader_port:
        reader_virtual_serial = f"tmr://{args.reader_port}" if not args.reader_port.startswith("tmr://") else args.reader_port
    if args.flipper_port:
        serial_port = args.flipper_port
    if args.baudrate:
        baud_rate = args.baudrate

    logging.info(f"Configuration:")
    logging.info(f"  Reader Port: {reader_virtual_serial}")
    logging.info(f"  Flipper Port: {serial_port}")
    logging.info(f"  Baud Rate: {baud_rate}")

    try:
        #Setup serial connection
        logging.info(f"Opening serial port {serial_port} at {baud_rate} baud")
        ser = serial.Serial(serial_port, baudrate=baud_rate, timeout=1)

        #Flush any existing input buffer
        ser.flushInput()
        logging.info("Serial port opened and flushed")

        while True:
            
            #Attempt to read a line of data from the RX buffer
            if ser.inWaiting() > 0:
                try:
                    
                    #Attempt to decode the incoming data
                    raw_line = ser.readline()
                    try:
                        incoming_data = raw_line.decode('utf-8').rstrip()
                        if incoming_data:
                            handle_command(incoming_data, ser)
                    except UnicodeDecodeError:
                        logging.error(f"Error: Received data could not be decoded to UTF-8: {raw_line}")
                        continue
                        
                except Exception as e:
                    logging.error(f"Error in main loop: {e}", exc_info=True)
                
    except Exception as e:
        logging.error(f"An error occurred in main setup: {str(e)}", exc_info=True)
        
    finally:
        if 'ser' in locals() or 'ser' in globals():
            ser.close()
            logging.info("Serial connection closed.")
            
            
if __name__ == "__main__":
    main()
