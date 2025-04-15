#!/bin/python
import sys, json, pexpect, os
from typing import List
from termcolor import colored

bash = pexpect.spawn("/bin/bash")

def run_cmd(description : str, cmd : str, expectations : List[str], timeout : int = 30, use_os : bool = False):
    print(colored(description, "cyan"))
    print(cmd)
    if use_os:
        os.system(cmd)
        return
    bash.sendline(cmd)
    if expectations != None:
        return bash.expect(expectations, timeout=timeout)

argv = sys.argv[1:]

config_file = argv[0]
root_dir = argv[1]
build_dir = root_dir + "/build"
open_file = argv[2]
open_file_standalone = open_file[open_file.rindex("/")+1:]


f = open(config_file)
config = json.load(f)
build_conf = config['build']
target_conf = config['target']

#identify the file to debug
lines = []
print(colored(f"Including the following CMake files:", "cyan"))
for cmake_file in build_conf['cmake_files']:
    f = open(cmake_file)
    lines.extend(f.readlines())
    print(colored(cmake_file, "cyan"))

dbg_file = ""
for line in lines:
    if line.lower().startswith("add_executable(") and line.find(open_file_standalone) != -1:
        dbg_file = line[15:line.find(" ")]

if dbg_file == "":
    print(colored(f"{open_file_standalone} is not part of an executable.", 'red'))
    exit()
print(colored(f"Binary to debug: {dbg_file}", "cyan"))

# Source ROS environment
run_cmd("Sourcing ROS Foxy", f"cd {build_dir} && source /opt/ros/foxy/setup.bash", None)

# Run CMake
response = run_cmd("Running CMake",
          f"cmake .. {build_conf['cmake_args']}",
          ["-- Configuring incomplete, errors occurred!",
           "-- Generating done"])
if response == 0:
    print(colored("Fail", 'red'))
    print(bash.before)
    bash.close()
    exit()
elif response == 1:
    print(colored("Success", 'red'))

# Run Make
response = run_cmd("Running make", f"make {build_conf['make_args']}",
           ["Built target", 
            "compilation terminated.", "Error 1", "Error 2"])
if response == 0:
    print(colored("Success", 'red'))
else:
    print(colored("Fail", 'red'))
    print(bash.before)
    bash.close()
    exit()


#transfer newly built files to target
output_dir = root_dir + "/" + build_conf['output_directory']
run_cmd("Transferring compiled files to target", 
          (f"sshpass -p \"{target_conf['password']}\" "
           f"scp -rp {output_dir}/* "
           f"{target_conf['user']}@{target_conf['host']}:{target_conf['debug_directory']}"), None, use_os=True)


#start gdbserver on target
run_cmd("Starting gdbserver on target", 
          (f"sshpass -p \"{target_conf['password']}\" "
           f"ssh {target_conf['user']}@{target_conf['host']} "
           #f"\"cd {target_conf['debug_directory']}; "
           f"'gdbserver localhost:{target_conf['debug_port']} {target_conf['debug_directory']}/{dbg_file} "
           f"</dev/null &>/dev/null &'"),
           None, use_os=True)
#bash.interact()