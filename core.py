import time
import numpy as np
from sage.base_app import BaseApp


from .gaitphase import GaitPhase
from .Rotation import Rotation as R
from .JointAngles import JointAngles


def get_rotation(data, node_num):
    return R.from_quat(
        [
            data[node_num]["Quat1"],
            data[node_num]["Quat2"],
            data[node_num]["Quat3"],
            data[node_num]["Quat4"],
        ],
        scalar_first=True,
    )


class Core(BaseApp):
    ###########################################################
    # INITIALIZE APP
    ###########################################################
    def __init__(self, my_sage):
        BaseApp.__init__(self, my_sage, __file__)

        self.DATARATE = self.info["datarate"]
        self.pulse_length = self.info["pulse_length"]
        self.min_threshold = float(self.config["min_threshold"])
        self.max_threshold = float(self.config["max_threshold"])

        self.FEEDBACK_DELAY = float(self.config["feedback_delay"])

        self.NodeNum_foot = self.info["sensors"].index("foot")
        self.NodeNum_shank = self.info["sensors"].index("shank")
        self.NodeNum_thigh = self.info["sensors"].index("thigh")
        self.NodeNum_pelvis = self.info["sensors"].index("pelvis")

        self.NodeNum_feedback_min = self.info["feedback"].index("feedback_min")
        self.NodeNum_feedback_max = self.info["feedback"].index("feedback_max")

        self.iteration = 0
        self.joint_angles = JointAngles(self.config["which_leg"] == "Right Leg")
        self.gait_phase = GaitPhase(self.DATARATE)

        self.foot_quat = None
        self.pelvis_quat = None
        self.thigh_quat = None
        self.shank_quat = None

        self.min_feedback_state = 0
        self.max_feedback_state = 0

    ###########################################################
    # CHECK NODE CONNECTIONS
    # Make sure all the nodes needed for sensing and feedback
    # are present before starting the app.
    #
    # If you do not need to check for feedback nodes, you can
    # comment or delete this function. The BaseApp will ensure
    # the correct number of sensing nodes are present and
    # throw an exception if they are not.
    ###########################################################
    def check_status(self):
        sensors_count = self.get_sensors_count()
        feedback_count = self.get_feedback_count()
        err_msg = ""
        if sensors_count < len(self.info["sensors"]):
            err_msg += "App requires {} sensors but only {} are connected".format(
                len(self.info["sensors"]), sensors_count
            )
        if self.config["feedback_enabled"] and feedback_count < len(
            self.info["feedback"]
        ):
            err_msg += "App require {} feedback but only {} are connected".format(
                len(self.info["feedback"]), feedback_count
            )
        if err_msg != "":
            return False, err_msg
        return True, "Now running Walking One Leg Flexion App"

    #############################################################
    # UPON STARTING THE APP
    # If you have anything that needs to happen before the app starts
    # collecting data, you can uncomment the following lines
    # and add the code in there. This function will be called before the
    # run_in_loop() function below.
    #############################################################
    # def on_start_event(self, start_time):
    #     print("In On Start Event: {start_time}")

    ###########################################################
    # RUN APP IN LOOP
    ###########################################################
    def run_in_loop(self):
        data = self.my_sage.get_next_data()

        # Get Quaternion Data
        self.foot_quat = get_rotation(data, self.NodeNum_foot)
        self.pelvis_quat = get_rotation(data, self.NodeNum_pelvis)
        self.thigh_quat = get_rotation(data, self.NodeNum_thigh)
        self.shank_quat = get_rotation(data, self.NodeNum_shank)

        # Calibrate to find BS_q, sensor to body segment alignment quaternions on 1st iteration
        if self.iteration == 0:
            self.joint_angles.calibrate(
                self.foot_quat, self.pelvis_quat, self.thigh_quat, self.shank_quat
            )
        # Update gait phases
        self.gait_phase.update_gaitphase(data[self.NodeNum_foot])

        # Calculate Extension angles
        self.Hip_flex = self.joint_angles.calculate_Hip_Flex(
            self.pelvis_quat, self.thigh_quat
        )
        self.Knee_flex = self.joint_angles.calculate_Knee_Flex(
            self.thigh_quat, self.shank_quat
        )
        self.Ankle_flex = self.joint_angles.calculate_Ankle_Flex(
            self.shank_quat, self.foot_quat
        )

        # Give haptic feedback (turn feedback nodes on/off)
        if self.config["feedback_enabled"]:
            self.give_feedback()
        else:
            self.min_feedback_state = 0
            self.max_feedback_state = 0

        time_now = self.iteration / self.DATARATE  # time in seconds

        my_data = {
            "time": [time_now],
            "Gait_Phase": [self.gait_phase.gaitphase.value],
            "step_count": [self.gait_phase.step_count],
            "min_threshold": [self.min_threshold],
            "max_threshold": [self.max_threshold],
            "min_feedback_state": [self.min_feedback_state],
            "max_feedback_state": [self.max_feedback_state],
            "Hip_flex": [self.Hip_flex],
            "Knee_flex": [self.Knee_flex],
            "Ankle_flex": [self.Ankle_flex],
        }

        self.my_sage.save_data(data, my_data)
        self.my_sage.send_stream_data(data, my_data)

        self.iteration += 1
        return True

    #############################################################
    # MANAGE FEEDBACK FOR APP
    #############################################################
    def toggle_feedback(self, feedbackNode=0, duration=1, feedback_state=False):
        if feedback_state:
            self.my_sage.feedback_on(feedbackNode, duration)
        else:
            self.my_sage.feedback_off(feedbackNode)

    def give_feedback(self):
        angles = {
            "Hip Flex": self.Hip_flex,
            "Knee Flex": self.Knee_flex,
            "Ankle Flex": self.Ankle_flex,
        }
        angleVal = angles[self.config["which_angle"]]

        self.min_feedback_state = int(angleVal < self.min_threshold)
        self.max_feedback_state = int(angleVal > self.max_threshold)

        self.toggle_feedback(
            self.NodeNum_feedback_min,
            duration=self.pulse_length,
            feedback_state=self.min_feedback_state,
        )
        self.toggle_feedback(
            self.NodeNum_feedback_max,
            duration=self.pulse_length,
            feedback_state=self.max_feedback_state,
        )

        # if no feedback should be given, make sure to toggle all feedback off
        # Note: This can only happen if both are 0
        if self.min_feedback_state == self.max_feedback_state:
            self.toggle_all_feedback_off()

    def toggle_all_feedback_off(self):
        self.toggle_feedback(self.NodeNum_feedback_min, feedback_state=False)
        self.toggle_feedback(self.NodeNum_feedback_max, feedback_state=False)

    #############################################################
    # UPON STOPPING THE APP
    # If you have anything that needs to happen after the app stops,
    # you can uncomment the following lines and add the code in there.
    # This function will be called after the data file is saved and
    # can be read back in for reporting purposes if needed.
    #############################################################
    # def on_stop_event(self, stop_time):
    #     print(f"In On Stop Event: {stop_time}")
