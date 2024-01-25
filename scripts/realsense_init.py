# autor:yeahtoon
# belong to WHU-ROBOCON sensor group

# import modules
import subprocess
import os
import time
import threading
import psutil
import signal



# define common viriable
program_path = "../build/theProgram"
program_name = "theProgram"
last_time = time.time()
check_string = "PL normal"
sem = threading.Semaphore(0)


# open c++ program and retarget the stdoutput pipe.
# realsense_cpp_program = subprocess.Popen("../virtual_client/build/virtual_client", text=True)
# while(1):
#     output, _ = realsense_cpp_program.communicate()
#     print(output)
#     time.sleep(1)

# check program exist 
def check_programExist(program_name):
    if(os.system("ps -A | grep " + program_name)):
        return 0
    else:
        return 1

# start the program and retarget the standard-ouput-pipe
def open_CppProgram(path):
    CppProgram = subprocess.Popen(path, stdout = subprocess.PIPE, text = True)
    return CppProgram


# get pid from process name
def get_pid(process_name):
    pid = None
    for process in psutil.process_iter(['pid', 'name']):
        if process.info['name'] == process_name:
            pid = process.info['pid']
            break
    return pid

# check if program stucked
def check_ProgramStucked():
    
    while(True):
        # calculate availiable time and decide whether shutdown the process
        availiable_time = time.time() - last_time
        if(availiable_time > 5):    # the program is stucked

            print(availiable_time)

            # without sem ,this stucked
            sem.acquire()

            # kill the program
            pid = get_pid(program_name)
            os.kill(pid, signal.SIGKILL)

        # delay
        time.sleep(1)

        print(availiable_time)


# deal with defunct program
def handle_func(*args):
    os.waitpid(-1, os.WNOHANG)



# main function
def main():

    # check the program stucked or not
    threading.Thread(target = check_ProgramStucked).start()

    # main circle
    while(True):

        # extern the global viriable
        global last_time 

        #deal with defunct process
        try:
            os.waitpid(-1, os.WNOHANG)
        except:
            pass

        # check the existence of realsense's cpp-program
        # if exist : check the reply of child program and update reply time
        if(check_programExist(program_name)):

            # get the stdout and judge whether the program is stucked
            while True:
                output = the_program.stdout.readline()   # communicate with the program, program would send back imformation if being healthy, or be stucked here

                # read over and break the circle
                if not output:
                    break

                #update last time
                if(check_string in output):
                    last_time = time.time() 

        # if not exist : restart the program
        else:  
            # open the program
            the_program = open_CppProgram(program_path)

            # release sem
            sem.release()

            # update last time
            last_time = time.time() 

            print("create new process\n\n\n\n")


        # delay 1 second
        time.sleep(1)


main()
