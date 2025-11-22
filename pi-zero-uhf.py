
from __future__ import print_function
from datetime import datetime
import serial
import mercury
import argparse
import glob
import sys
import logging

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
            reader.set_read_plan([1], "GEN2", read_power=reader_power)
            logging.info("Successfully connected to reader")
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
                old_epc_str = ser.readline().decode().strip()
                new_epc_str = ser.readline().decode().strip()
                
                logging.debug(f"WRITE Target EPC: {old_epc_str}")
                logging.debug(f"WRITE New EPC: {new_epc_str}")
                
                # Basic validation or cleanup if needed
                if not new_epc_str:
                    logging.error("New EPC is empty")
                    ser.write(b"EVBAD\n")
                    return

                # Strip trailing 'b' if present (common artifact from Flipper storage/transmission)
                if old_epc_str.endswith('b') and len(old_epc_str) % 2 != 0:
                    old_epc_str = old_epc_str[:-1]
                if new_epc_str.endswith('b') and len(new_epc_str) % 2 != 0:
                    new_epc_str = new_epc_str[:-1]

                try:
                    # Convert hex strings to raw bytes
                    # epc_code (new EPC) should definitely be raw bytes
                    new_epc = bytearray.fromhex(new_epc_str)
                    
                    # epc_target (old EPC) handling - preserving existing behavior of encode() 
                    # unless it causes issues, but logically it should probably be raw bytes too.
                    # However, for safety with existing code conventions elsewhere:
                    # We will try to use raw bytes for target if possible, as that's standard for Mercury API
                    old_epc = bytearray.fromhex(old_epc_str)
                except ValueError as e:
                     logging.error(f"Hex conversion error: {e}")
                     ser.write(b"EVBAD\n")
                     return
                
                # Loop to continuously attempt writing until success or cancellation
                success = False
                logging.info("Starting continuous write loop for EPC...")
                while True:
                    if ser.inWaiting() > 0:
                        logging.info("Write cancelled by user")
                        break
                    
                    try:
                        if reader.write(epc_code=new_epc, epc_target=old_epc):
                            success = True
                            break
                    except Exception as e:
                        # Keep retrying if tag not found
                        pass

                if success:
                    ser.write(b"EVOK\n")
                    logging.info("Write EPC successful")
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

                old_epc = bytearray.fromhex(old_epc_str)
                new_data = bytearray.fromhex(new_data_str)

                # Loop to continuously attempt writing until success or cancellation
                success = False
                logging.info("Starting continuous write loop for TID...")
                while True:
                    if ser.inWaiting() > 0:
                        logging.info("Write cancelled by user")
                        break
                    
                    try:
                        if reader.write_tag_mem(2, 0x00, new_data, old_epc):
                            success = True
                            break
                    except Exception as e:
                        pass

                if success:
                    ser.write(b"TVOK\n")
                    logging.info("Write TID successful")
                else:
                    ser.write(b"TVBAD\n")
                    logging.warning("Write TID failed or cancelled")

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

                if old_epc_str.endswith('b') and len(old_epc_str) % 2 != 0: old_epc_str = old_epc_str[:-1]
                if new_data_str.endswith('b') and len(new_data_str) % 2 != 0: new_data_str = new_data_str[:-1]
                
                old_epc = bytearray.fromhex(old_epc_str)
                new_data = bytearray.fromhex(new_data_str)
            
                # Loop to continuously attempt writing until success or cancellation
                success = False
                logging.info("Starting continuous write loop for User Memory...")
                while True:
                    if ser.inWaiting() > 0:
                        logging.info("Write cancelled by user")
                        break
                    
                    try:
                        if reader.write_tag_mem(3, 0x00, new_data, old_epc):
                            success = True
                            break
                    except Exception as e:
                        pass

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

                if old_epc_str.endswith('b') and len(old_epc_str) % 2 != 0: old_epc_str = old_epc_str[:-1]
                if new_data_str.endswith('b') and len(new_data_str) % 2 != 0: new_data_str = new_data_str[:-1]

                old_epc = bytearray.fromhex(old_epc_str)
                new_data = bytearray.fromhex(new_data_str)
                
                # Loop to continuously attempt writing until success or cancellation
                success = False
                logging.info("Starting continuous write loop for Reserved Memory...")
                while True:
                    if ser.inWaiting() > 0:
                        logging.info("Write cancelled by user")
                        break
                    
                    try:
                        if reader.write_tag_mem(1, 0x00, new_data, old_epc):
                            success = True
                            break
                    except Exception as e:
                        pass

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
                # Added timeout to read() to ensure it doesn't block indefinitely or return too quickly without scanning
                logging.debug("Starting read operation...")
                epc_list = []

                while len(epc_list) == 0:
                    if ser.inWaiting() > 0:
                        logging.info("Incoming command detected, stopping read loop")
                        break

                    raw_tags = reader.read(timeout=500)
                    epc_list = list(map(lambda tag: tag.epc, raw_tags))

                logging.info(f"Read {len(epc_list)} tags")
                
                tid_list = []
                res_list = []
                user_mem_list = []
                for epc in epc_list:
                     logging.debug(f"Reading memory for EPC: {epc}")
                     
                     # Read TID (Bank 2)
                     try:
                         input_byte_array = reader.read_tag_mem(2, 0x00, 20, epc)
                     except Exception as e:
                         logging.warning(f"Failed to read TID for {epc}: {e}")
                         input_byte_array = None

                     # Read Reserved (Bank 0)
                     try:
                         input_reserved_arr = reader.read_tag_mem(0, 0x00, 8, epc)
                     except Exception as e:
                         logging.warning(f"Failed to read Reserved memory for {epc}: {e}")
                         input_reserved_arr = None

                     # Read User (Bank 3)
                     try:
                         input_user_mem_arr = reader.read_tag_mem(3, 0x00, 16, epc)
                     except Exception as e:
                         logging.warning(f"Failed to read User memory for {epc}: {e}")
                         input_user_mem_arr = None
                     

                     #Filling in any missing values if the Tag is locked
                     if input_byte_array != None:
                        formatted_byte_array_str = format_byte_array_to_X_bytes(input_byte_array, 20)
                        tid_list.append(formatted_byte_array_str)
                     else:
                        formatted_byte_array_str = "0000000000000000000000000000000000000000" 
                        tid_list.append(formatted_byte_array_str)
                     if input_reserved_arr != None:
                        formatted_byte_array_res_str = format_byte_array_to_X_bytes(input_reserved_arr, 8)
                        res_list.append(formatted_byte_array_res_str)
                     else:
                        formatted_byte_array_res_str =  "0000000000000000"
                        res_list.append(formatted_byte_array_res_str)
                                                      
                     if input_user_mem_arr != None:
                        formatted_byte_array_mem_str = format_byte_array_to_X_bytes(input_user_mem_arr, 16)
                        user_mem_list.append(formatted_byte_array_mem_str)
                     else:
                        formatted_byte_array_mem_str =  "00000000000000000000000000000000"
                        user_mem_list.append(formatted_byte_array_mem_str)
                
                
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
                reader.set_read_plan([1], "GEN2", read_power=int(value))
                logging.info(f"Power set to {value}")
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
        epc_values = [str(epc) for epc in epc_list]
        
        for epc in epc_values:
            epc = extract_content_from_format(epc)
            epc = epc + "b\n"
            epc = epc.encode()
            ser.write(epc)
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
