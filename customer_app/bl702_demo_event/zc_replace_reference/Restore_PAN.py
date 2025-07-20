# $language = "python"
# $interface = "1.0"


import os
import time


ieee_address = None
extended_pan_id = None
pan_id = None
radio_channel = None
global_key = None
global_fc = None  
network_key = None
network_fc = None  
network_sequence_number = None
tc_link_key_number = 0  
tc_link_key_list = []


global_var_dict = {
    'ieee_address': ieee_address,
    'extended_pan_id': extended_pan_id,
    'pan_id': pan_id,
    'radio_channel': radio_channel,
    'global_key': global_key,
    'global_fc': global_fc,
    'network_key': network_key,
    'network_fc': network_fc,
    'network_sequence_number': network_sequence_number,
    'tc_link_key_number': tc_link_key_number
}

def parse_line_to_global_vars(line, global_var_dict):
    line = line.strip()
    if ':' in line:
        key, value = line.split(':', 1)
        key = key.strip().lower()  
        value = value.strip()
        
        if key == 'tc_link_key_number':
            global_var_dict[key] = int(value)  
        elif key in global_var_dict:
            global_var_dict[key] = value  
        elif key.startswith('tc_link_key'):
            index = int(key.split('[')[1].split(']')[0])
#            cleaned_key = value.replace(" ", "").replace("0x", "")
            cleaned_key = value.strip()
            tc_link_key_list.insert(index, cleaned_key)  
        else:
            pass


def Main():

    objTab = crt.GetScriptTab()

    file_path = 'nwkInfo.txt'  
    with open(file_path, 'r') as file:
        for line in file:
            parse_line_to_global_vars(line, global_var_dict)

    if all(global_var_dict.values()): 
        cmd_reset_stack = "zb_pan_reset \r\n"
        cmd_write_role = "zb_set_role 0 \r\n"
        cmd_write_addr = "zb_set_tc_addr " + str(global_var_dict["ieee_address"]) +"\r\n"
        cmd_write_extPanid = "zb_set_extern_panid " + " " + str(global_var_dict["extended_pan_id"]) + "\r\n"
        cmd_write_panid = "zb_set_panid " + " " + str(global_var_dict["pan_id"]) + "\r\n"
        cmd_write_channel = "zb_set_channel " + " " +  str(global_var_dict["radio_channel"])  + "\r\n"
        cmd_write_tc = "zb_set_tc_link_key " + str(global_var_dict["global_key"]) + "\r\n"
        cmd_write_nwk_key = "zb_set_nwk_key " + str(global_var_dict["network_fc"]) + " " +  str(global_var_dict["network_sequence_number"]) + " " + str(global_var_dict["network_key"]) + "\r\n"

        objTab.Screen.Send(cmd_reset_stack)
        time.sleep(1)

        objTab.Screen.Send(cmd_write_addr)
        time.sleep(1)

        objTab.Screen.Send(cmd_write_role)
        time.sleep(1)

        objTab.Screen.Send(cmd_write_channel)
        time.sleep(1)

        objTab.Screen.Send(cmd_write_panid)
        time.sleep(1)

        objTab.Screen.Send(cmd_write_extPanid)
        time.sleep(1)

        objTab.Screen.Send(cmd_write_nwk_key)
        time.sleep(1)

        objTab.Screen.Send("zb_pan_startup \r\n")

        response = objTab.Screen.WaitForStrings(["Start as ZC: ", "zb_startup_complete_cb:"], 5)
      
        if response:
            # send trust center link key until ZC startup success
            objTab.Screen.Send(cmd_write_tc)
            time.sleep(1)

            # send link key table entry one by one
            for i in range(global_var_dict['tc_link_key_number']):
                cmd_write_link_key_table = "zb_set_lk_table " + str(tc_link_key_list[i]) + "\r\n"
                objTab.Screen.Send(cmd_write_link_key_table)
                objTab.Screen.Send("\r\n")
                objTab.Screen.Send("\r\n")
                time.sleep(1) 

            objTab.Screen.Send("zb_dump_pan_info \r\n")

        else:
            objTab.Screen.Send("ZC_CLONE_FAILED \r\n")
            time.sleep(1)
                 

    return

Main()