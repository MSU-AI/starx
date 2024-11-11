""""
This file just runs both leg collections at once.
The 3 second delay between scripts is because we experienced some I2C errors when trying to
instantaneously access both I2C busses on the Jetson. 
"""

import subprocess
import time

DELAY_SECONDS = 3

right = 'right.py'
left = 'left.py'

print(f"Running {right}")
subprocess.run(["python",right])

print(f"Waiting {DELAY_SECONDS} seconds...")
time.sleep(DELAY_SECONDS)

print(f"Running {left}")
subprocess.run(["python",left])

print("Run complete.")