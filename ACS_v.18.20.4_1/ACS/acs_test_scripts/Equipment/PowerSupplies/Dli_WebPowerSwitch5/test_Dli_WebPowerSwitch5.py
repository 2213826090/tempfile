"""
Created on 29 mai 2013

:author: lbavois
"""

from time import sleep
import dlipower as dli


if __name__ == '__main__':

    # Set up the PowerSwitch with default parameters present in default config file path
    # CONFIG_FILE = os.path.expanduser('~/.dlipower.conf')

    switch = dli.PowerSwitch("admin", "1234", "192.168.0.100", 20, 3, 3)

    # Verify we can talk to the switch
    if not switch.verify():
        print("ERROR : Can't talk to the switch")
    else:
        print("SUCCESS : Can talk to the switch")

        loop_number = 1

        while True:

            print("START TEST LOOP NUMBER = " + str(loop_number))

            # Get status for all outlet on the power supply
            switch.printstatus()

            outlet = 1

            while outlet <= 8:

                # Switch on OUTLET
                if not switch.on(outlet):
                    # if return value is False, we succeed to turn on the outlet
                    print("SUCCESS : Outlet " + str(outlet) + " switched on")
                else:
                    print("FAILURE : Outlet " + str(outlet) + " switched on")

                sleep(2)

                # Get status for all outlet on the power supply
                switch.printstatus()

                outlet += 1

            outlet = 1

            while outlet <= 8:

                # Switch off OUTLET
                if not switch.off(outlet):
                    # if return value is False, we succeed to turn off the outlet
                    print("SUCCESS : Outlet " + str(outlet) + " switched off")
                else:
                    print("FAILURE : Outlet " + str(outlet) + " switched off")

                sleep(2)

                # Get status for all outlet on the power supply
                switch.printstatus()

                outlet += 1

            loop_number += 1
