"""
Template file to create a mission class
"""

# import what you need from within the package

import logging

from ..cv.template import TemplateCV


class TemplateMission:
    def __init__(self, **kwargs):
        """
        Init of the class,
        setup here everything that will be needed for the run fonction
        kwargs is a dict containing the key arguments you give to the mission
        """
        logging.info("Template mission init")

    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        Return True if everything went well, False otherwise (ideally)
        """
        logging.info("Template mission run")
        return True

    def terminate(self):
        """
        Here should be all the code required to terminate the mission.
        This could be cleanup, saving data, closing files, etc.
        Return True if everything went well, False otherwise (ideally)
        """
        logging.info("Template mission terminate")
        return True


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template"
    # You can also import it in a mission file outside of the package
    
    # Create a mission object with arguments
    mission = TemplateMission(arg1="value1", arg2="value2")
    # Run the mission
    mission.run()
    # Terminate the mission
    mission.terminate()

