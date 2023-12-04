import subprocess
import signal

# Define the file paths of your scripts
main_script_path = 'Child_Scripts\Main_writetocsv.py'
live_plot_script_path = 'Child_Scripts\liveplot.py'

# Define the file name
file_name = 'C:/Users/taylo/OneDrive/Documents/Desktop/GradSchool/Lab/tMotor Controllers/Data/230713TESTING7.csv'

#Define the Amplitude of the sine wave sent to the motor
amplitude = 1.0

# Execute the scripts simultaneously using subprocess
main_process = subprocess.Popen(['python', main_script_path, '--file_name', file_name, '--amplitude', str(amplitude)])
live_plot_process = subprocess.Popen(['python', live_plot_script_path, '--file_name', file_name])

# Catch the termination signal and terminate the child processes
def signal_handler(sig, frame):
    main_process.terminate()
    live_plot_process.terminate()

signal.signal(signal.SIGINT, signal_handler)

# Wait for both processes to finish
main_process.wait()
live_plot_process.wait()

# Show the final plot
subprocess.run(['python', live_plot_script_path, '--file_name', file_name])