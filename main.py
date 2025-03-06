import csv
import time
from tkinter import Text, Scrollbar
import serial
import customtkinter
from customtkinter import *
from window_creation import PlanarWindowCreation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import numpy as np
import datetime
from scipy.signal import medfilt
from collections import deque
import pandas as pd

from closed_loop_ekf import measurement_update1, state_transition, jacob_state, measurement_update2

customtkinter.set_appearance_mode("system")    # Modes: system (default), light, dark
customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

ARDUINO_PORT = 'COM3'
BAUD_RATE = 115200

LEFT = -900/2 #450/2
RIGHT = 900/2 #450/2

class RootWindow:

    def __init__(self, master):
        self.master = master
        self.sub_window = None
        # left frame
        self.frame_1 = customtkinter.CTkFrame(master)

        cnt = 0
        connect_button = customtkinter.CTkButton(self.frame_1, text="Connect", command=self.connect_to_arduino)
        connect_button.grid(row=cnt, column=0, padx=25, pady=10, columnspan=1)
        self.open_exo_but = customtkinter.CTkButton(self.frame_1, text="Open Planar Window", command=self.open_planar_exo_window)
        self.open_exo_but.grid(row=cnt, column=1, columnspan=1, padx=5, pady=10)

        cnt += 1
        user_label = customtkinter.CTkLabel(self.frame_1, text='User Number:')
        user_label.grid(row=cnt, column=0, pady=10)
        self.user_entry_var = customtkinter.StringVar()
        user_entry = customtkinter.CTkEntry(self.frame_1, textvariable=self.user_entry_var)
        user_entry.grid(row=cnt, column=1, padx=10)

        cnt += 1
        trial_label = customtkinter.CTkLabel(self.frame_1, text='Trial:')
        trial_label.grid(row=cnt, column=0, padx=10, pady=10)
        self.combobox_trial = customtkinter.CTkComboBox(self.frame_1, values=['LC', 'LR', 'CL', 'CR', 'CLRC']) # LC: Left to Center, CLRC: Center to Left to Right to Center
        self.combobox_trial.grid(row=cnt, column=1, padx=10, pady=10)

        cnt += 1
        self.run_trial_but = customtkinter.CTkButton(self.frame_1, text="Run Trial",
                                                     fg_color="green")  # command=self.run_trial,
        self.run_trial_but.grid(row=cnt, column=0, columnspan=1, padx=8, pady=10)
        self.stop_but = customtkinter.CTkButton(self.frame_1, text="Stop Trial", fg_color="red")#command=self.stop_trial,
        self.stop_but.grid(padx=25, pady=10, row=cnt, column=1, columnspan=1)

        cnt += 1
        combo_loc_name = customtkinter.CTkLabel(self.frame_1, text='Initial Location:')
        combo_loc_name.grid(row=cnt, column=0, padx=10, pady=10)
        self.combobox_loc = customtkinter.CTkComboBox(self.frame_1, values=["Left", "Center", "Right"])
        self.combobox_loc.grid(row=cnt, column=1, padx=10, pady=10)

        cnt += 1
        combo_speed = customtkinter.CTkLabel(self.frame_1, text='Speed:')
        combo_speed.grid(row=cnt, column=0, padx=10, pady=10)
        self.combobox_speed = customtkinter.CTkComboBox(self.frame_1, values=["Slow", "Fast"])
        self.combobox_speed.grid(row=cnt, column=1, padx=10, pady=10)

        cnt += 1
        combo_des_name = customtkinter.CTkLabel(self.frame_1, text='Destination:')
        combo_des_name.grid(row=cnt, column=0, padx=10, pady=10)
        self.combobox_des = customtkinter.CTkComboBox(self.frame_1, values=["Left center", "Far left", "Right center", "Far right"])
        self.combobox_des.grid(row=cnt, column=1, padx=10, pady=10)

        cnt += 1
        self.des_but = customtkinter.CTkButton(self.frame_1, text="Go to Destination", command=self.go_to_destination)
        self.des_but.grid(padx=25, pady=10, row=cnt, column=0, columnspan=2)

        cnt += 1
        self.Offset_lbl = customtkinter.CTkLabel(self.frame_1, text='Real offset:')
        self.Offset_lbl.grid(row=cnt, column=0, pady=2)

        cnt += 1
        self.Est_offset_lbl= customtkinter.CTkLabel(self.frame_1, text='Estimated offset:')
        self.Est_offset_lbl.grid(row=cnt, column=0, pady=2)

        cnt += 1
        self.Est_offset_unobs_lbl = customtkinter.CTkLabel(self.frame_1, text='Estimated offset2:')
        self.Est_offset_unobs_lbl.grid(row=cnt, column=0, pady=2)

        cnt += 1
        steady_label = customtkinter.CTkLabel(self.frame_1, text='No Movement:')
        steady_label.grid(row=cnt, column=0, pady=10)
        self.steady_label_var = customtkinter.StringVar()
        steady_entry = customtkinter.CTkEntry(self.frame_1, textvariable=self.steady_label_var)
        steady_entry.grid(row=cnt, column=1, padx=10)


        cnt += 1
        self.checkbox_var = customtkinter.IntVar()
        self.checkbox = customtkinter.CTkCheckBox(self.frame_1, variable=self.checkbox_var, text="Recovery")
        self.checkbox.grid(row=cnt, column=0, columnspan=2, padx=8, pady=10)

        # button_labels1 = ["1", "2", "3", "4", "5"] #CLF: center to left, fast
        # button_functions1 = [self.func1, self.func2, self.func3, self.func4, self.func5]
        #
        # button_labels2 = ["6", "7", "8", "9", "10"]  # CLF: center to left, fast
        # button_functions2 = [self.func6, self.func7, self.func8, self.func9, self.func10]
        #
        # self.radio_var = customtkinter.StringVar(value="")
        #
        # for i, (func1, func2) in enumerate(zip(button_functions1, button_functions2)):
        #     cnt += 1
        #     radio_button1 = customtkinter.CTkRadioButton(self.frame_1, text=str(i+1), variable=self.radio_var,
        #                                                  value=str(i+1), command=func1)
        #     radio_button1.grid(row=cnt, column=0, columnspan=1, padx=8, pady=10)
        #     radio_button2 = customtkinter.CTkRadioButton(self.frame_1, text=str(i+6), variable=self.radio_var,
        #                                                  value=str(i+6), command=func2)
        #     radio_button2.grid(row=cnt, column=1, columnspan=1, padx=8, pady=10)

        # Create buttons and add them to the window
        # for label1, func1, label2, func2 in zip(button_labels1, button_functions1, button_labels2, button_functions2):
        #     cnt +=1
        #     button = customtkinter.CTkButton(self.frame_1, text=label1, fg_color="green", command=func1)
        #     button.grid(row=cnt, column=0, columnspan=1, padx=8, pady=10)
        #     button = customtkinter.CTkButton(self.frame_1, text=label2, fg_color="green", command=func2)
        #     button.grid(row=cnt, column=1, columnspan=1, padx=8, pady=10)

        # cnt += 1
        # self.clear_but = customtkinter.CTkButton(self.frame_1, text="CLEAR", fg_color="tomato",  command=self.clear_received_data)
        # self.clear_but.grid(row=cnt, column=0, columnspan=2, padx=8, pady=10)

        # cnt += 1
        # self.checkbox_var = customtkinter.IntVar()
        # self.checkbox = customtkinter.CTkCheckBox(self.frame_1, variable=self.checkbox_var, text="Recovery")
        # self.checkbox.grid(row=cnt, column=0, columnspan=2, padx=8, pady=10)

        self.frame_1.grid(padx=10, pady=10, row=0, column=1)


        # second frame===============================================================================================
        self.frame_2 = customtkinter.CTkFrame(master)

        self.fig = Figure(figsize=(8, 5), dpi=100, facecolor='#D3D3D3')
        self.ax1 = self.fig.add_subplot(221)
        self.ax2 = self.fig.add_subplot(222)
        self.ax3 = self.fig.add_subplot(223)
        self.ax4 = self.fig.add_subplot(224)
        self.ax1.grid()
        self.ax2.grid()
        self.ax3.grid()
        self.ax4.grid()
        self.ax1.set_title('Distances')
        self.ax2.set_title('Estimated distances (EKF)')
        self.ax3.set_title('Estimated distances (P Unobs EKF)')
        self.ax4.set_title('Offsets')
        self.ax3.set_xlabel('Time')
        self.ax4.set_xlabel('Time')
        self.ax1.set_ylabel('Amplitude (cm)')
        self.ax3.set_ylabel('Amplitude (cm)')
        self.fig.suptitle("Visualization")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_2)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0)  # Adjust row and column as required
        self.fig.tight_layout()
        self.frame_2.grid(padx=10, pady=10, row=0, column=0)

        # Third frame ================================================================================
        self.frame_3 = customtkinter.CTkFrame(master)

        self.text_box_lbl = customtkinter.CTkLabel(self.frame_3, text='Information:')
        self.text_box_lbl.grid(row=0, column=0, pady=2)

        # Creating a CTkText widget to display text
        self.text_box = customtkinter.CTkTextbox(self.frame_3, wrap='word', height=450, width=400,  bg_color="white", fg_color="black")  # adjust the height and width as required
        self.text_box.grid(row=1, column=0, sticky="nsew")

        self.frame_3.grid(padx=10, pady=10, row=0, column=3)

        # =======================================================
        self.frame_5 = customtkinter.CTkFrame(master)

        button_functions1 = [self.func1, self.func2, self.func3, self.func4, self.func5, self.func6, self.func7, self.func8, self.func9, self.func10, self.func11, self.func12]

        button_functions2 = [self.func13, self.func14, self.func15, self.func16, self.func17, self.func18, self.func19, self.func20, self.func21, self.func22, self.func23, self.func24]

        self.radio_var = customtkinter.StringVar(value="")

        for i, (func1, func2) in enumerate(zip(button_functions1, button_functions2)):
            cnt += 1
            radio_button1 = customtkinter.CTkRadioButton(self.frame_5, text=str(i + 1), variable=self.radio_var,
                                                         value=str(i + 1), command=func1)
            radio_button1.grid(row=cnt, column=0, columnspan=1, padx=8, pady=10)
            radio_button2 = customtkinter.CTkRadioButton(self.frame_5, text=str(i + 13), variable=self.radio_var,
                                                         value=str(i + 13), command=func2)
            radio_button2.grid(row=cnt, column=1, columnspan=1, padx=8, pady=10)

        self.frame_5.grid(padx=10, pady=10, row=0, column=2)

        # Fourth frame ===========================================================================
        self.frame_4 = customtkinter.CTkFrame(master)

        self.info_box = customtkinter.CTkTextbox(self.frame_4, wrap='word', height=300, width=700,  bg_color="white", fg_color="black")

        # To insert some default text, you can use:
        info_text = (
            "Motor (right[1] >0; left[0] < 0): \n A[low]B[low]  = -10; A[high]B[low] = -20; A[low]B[high] = +10; A[high]B[high] = +20"
            "\n Sensors: 0: middle; 1: left; right: 2 \n Encoder: left>0, right<0; \n\n offset = encoderDis - laserDis : left<0; right>0  [no human]"
            "\n\n Going left (B_low, A_low), offset <0  bottom ahead, top behind  bottom change direction (go right : B_high)"
            "\n Going left (B_low, A_low), offset>0  bottom behind, top ahead  bottom speed/catch up to left (A_high)"
            "\n Going right (B_high, A_low), offset>0  bottom ahead, top behind  bottom change direction (go left: B_low)"
            "\n Going right (B_high, A_low), offset<0  bottom behind, top ahead  bottom speed/catch up to right (A_high)"
            "\n\n commands to arduino: [sensor_num][A_status][B_status]"
        )
        self.info_box.insert('1.0', info_text, "center")
        self.info_box.grid(row=0, column=0, sticky="nsew")

        # self.frame_4.grid(padx=10, pady=10, row=1, column=0)

        #defining variables ====================================================================================

        self.read_thread = None
        self.is_reading = False

        self.current_sensor = None

        self.number1 = None #encoder
        self.number2 = None #laser
        self.number3 = None #raw HLFB
        self.number4 = None #Calculated torque in arduino
        self.offset = None

        self.trial_n = None
        self.direction = None
        self.speed = None
        self.sensor_crossed = True

        #reading torque =============
        self.pwm_threshold = 30
        self.high_duration = 0
        self.low_duration = 0
        self.prev_value_high = False

        # tutle ===================
        self.turtle_coords = None
        self.turtle_start_coords = None
        self.turtle_stop_coords = None

        self.mag_start = None
        self.mag_stop = None
        self.mag_dis_list = [24, 36, 54]#[15, 28, 45] #[22, 33, 48]#[20, 33, 51] #[7,33,68]

        # offset recovery ==============
        self.original_command = None
        self.threshold = 6
        self.in_recovery = None
        self.in_recovery_list = []

        # (Observable) EKF (1) ===============

        self.state_mean_predicted_list = []
        self.state_covariance_predicted_list = []

        self.EKF_offset = None
        self.EKF_state_mean = [35.20343111, - 1.93035287, 36.08940561, - 2.71101332]
        self.EKF_state_covariance = [[2.59486978e-03,  3.81238970e-02,  4.29827463e-03, - 3.25371979e-02],
                                     [3.81238970e-02, 1.88364257e+00, 3.46480163e-01, - 1.48139592e+00],
                                     [4.29827463e-03,  3.46480163e-01,  2.52905178e-01, - 1.15828030e-01],
                                     [-3.25371979e-02, - 1.48139592e+00, - 1.15828030e-01, 4.66623868e+00]]
        self.EKF_process_noise_value = np.diag([0.001, 0.01, 0.1, 0.1])
        self.EKF_measurement_noise_value = np.array([0.001, 0.4])


        self.tic = datetime.datetime.now()
        # self.toc = time.perf_counter()
        self.dt_seconds = None

        self.state_covariance_predicted_list.append(self.EKF_state_covariance)

        # (Partially unobservable) EKF (2) ===============================================================================

        self.state_mean_predicted_list2 = []
        self.state_covariance_predicted_list2 = []

        self.EKF_offset2= None
        self.EKF_state_mean2 = [35.20343111, - 1.93035287, 36.08940561, - 2.71101332]
        self.EKF_process_noise_value2 = np.diag([0.001, 0.01, 0.1, 0.1])
        self.EKF_measurement_noise_value2 = np.array([0.01])
        self.EKF_state_covariance2 = [[1.38616555e-02, 9.24779277e-02, 5.92543577e-04, 7.99880673e-02],
                                     [9.24779277e-02, 3.04822594e+00, -7.45887716e-02, 2.59527560e+00],
                                     [5.92543577e-04, -7.45887716e-02, 3.86470301e-02, -3.81288224e-02],
                                     [7.99880673e-02, 2.59527560e+00, -3.81288224e-02, 2.82050237e+00]]

        self.state_covariance_predicted_list2.append(self.EKF_state_covariance2)

        # Plotting ===============================================================================
        self.number1_values = []
        self.number2_values = []
        self.number3_values = []
        self.number4_values = []
        self.torque_values = []
        self.offset_values = []

        self.mag_force_values = []
        self.turtle_coords_values = []

        self.number1_estimated_values = []
        self.number2_estimated_values = []
        self.number1_estimated_values2 = []
        self.number2_estimated_values2= []
        self.EKF_offset_values = []
        self.EKF_offset_values2 = []
        self.estimated_velocity = []
        self.estimated_velocity2 = []

        self.direction_list = []
        self.trial_numbers = []
        self.time_list=[]
        self.speed_list = []
        self.current_sensor_list = []

        self.columns = ['encoder', 'laser', 'EstEncoder_Obs', 'EstLaser_Obs', 'EstEncoder_Unobs', 'EstLaser_Unobs',
                        'endpoint', 'time', 'trial','direction', 'speed', 'recoveryState',
                        'EstSpeed_obs', 'EstSpeed_unobs', 'turtle', 'hlfb', 'arduinoTorque', 'torque', 'magforce']
        self.subject_data = pd.DataFrame(columns=self.columns)

      def start_reading(self):
        self.is_reading = True
        self.read_thread = threading.Thread(target=self.read_data)
        self.read_thread.start()
    def read_data(self):
        self.in_recovery = False

        while self.is_reading:
            if hasattr(self, 'arduino'):
                try:
                    data = self.arduino.readline().decode().strip()
                    if data:
                        current_time = datetime.datetime.now()
                        time_difference = current_time - self.tic
                        self.dt_seconds = time_difference.total_seconds()
                        self.tic = current_time

                        try:

                            split_data = list(map(str.strip, data.split(',')))
                            self.number1, self.number2, self.number3, self.number4 = map(float, split_data) #encoder,laser, hlfb, arduino torque

                            self.number2_values.append(self.number2)
                            # self.number1 = self.number1 + self.number2_values[0]
                            self.number1_values.append(self.number1)
                            self.number3_values.append(self.number3)
                            self.number4_values.append(self.number4)

                            self.check_duty_cycle(self.number3)

                            self.offset = self.number1 - self.number2
                            self.Offset_lbl.configure(text=f"Offset: {self.offset:.2f}")

                            self.offset_values.append(self.offset)

                            # Move turtle ==============================================
                            self.map_magnet_to_turtle(self.number1, self.mag_start, self.mag_stop, self.turtle_start_coords, self.turtle_stop_coords)
                            self.sub_window.user_turtle.goto((int(self.turtle_coords), 0))
                            self.turtle_coords_values.append(self.turtle_coords)

                            # Laser, Encoder & Torque:
                            # Observable EKF ===========================================================================================
                            self.EKF_state_mean, self.EKF_state_covariance = self.update_ekf_state(self.EKF_state_mean,
                                                                                                   self.EKF_state_covariance,
                                                                                                   self.EKF_measurement_noise_value,
                                                                                                   self.EKF_process_noise_value,
                                                                                                   self.number1,
                                                                                                   self.number2,
                                                                                                   self.number4)
                            self.number1_estimated_values.append(self.EKF_state_mean[0])
                            self.number2_estimated_values.append(self.EKF_state_mean[2])

                            # self.EKF_offset = self.EKF_state_mean[0] - self.EKF_state_mean[2]
                            self.EKF_offset = self.number1_estimated_values[-1] - self.number2_estimated_values[-1]

                            self.Est_offset_lbl.configure(text=f"Estimated offset: {self.EKF_offset:.2f}")
                            # self.text_box.insert("1.0", f"<<< Received: {np.round(self.number1, 2), np.round(self.number2, 2)}, "
                            #                             f""f" EKF: {np.round(self.EKF_state_mean[0], 2), np.round(self.EKF_state_mean[2], 2)} \n")

                            self.EKF_offset_values.append(self.EKF_offset)
                            self.estimated_velocity.append((self.EKF_state_mean[1], self.EKF_state_mean[3]))

                            # Unobservable EKF ======================================================================
                            self.EKF_state_mean2, self.EKF_state_covariance2 = self.unobservable_ekf_state(self.EKF_state_mean2,
                                                                                                           self.EKF_state_covariance2,
                                                                                                           self.EKF_measurement_noise_value2,
                                                                                                           self.EKF_process_noise_value2,
                                                                                                           self.number1,
                                                                                                           self.number4)

                            self.EKF_offset2 = self.EKF_state_mean2[0] - self.EKF_state_mean2[2]
                            # self.EKF_offset2 = self.number1_estimated_values2[-1] - self.number2_estimated_values2[-1]
                            self.Est_offset_unobs_lbl.configure(text=f"Estimated offset2: {self.EKF_offset2:.2f}")

                            # self.text_box.insert("1.0",
                            #                      f"<<< Received: {data}, "f" EKF (Unobs): {np.round(self.EKF_state_mean2[0], 2), np.round(self.EKF_state_mean2[2], 2)} \n")
                            self.text_box.insert("1.0",
                                                 f"<<< Received: {np.round(self.number1, 2), np.round(self.number2, 2)}, "
                                                 f""f" EKF: {np.round(self.EKF_state_mean[0], 2), np.round(self.EKF_state_mean[2], 2)}, "
                                                 f"EKF2: {np.round(self.EKF_state_mean2[0], 2), np.round(self.EKF_state_mean2[2], 2)} \n")

                            self.number1_estimated_values2.append(self.EKF_state_mean2[0])
                            self.number2_estimated_values2.append(self.EKF_state_mean2[2])

                            self.EKF_offset_values2.append(self.EKF_offset2)
                            self.estimated_velocity2.append((self.EKF_state_mean2[1], self.EKF_state_mean2[3]))

                            # For saving data ===================================================================

                            date_str = current_time.strftime('%Y-%m-%d_%H-%M-%S')
                            self.time_list.append(date_str)
                            self.trial_numbers.append(self.trial_n)
                            self.direction_list.append(self.direction)
                            self.speed_list.append(self.speed)
                            self.current_sensor_list.append(-1)
                            self.in_recovery_list.append(0)
                            if self.in_recovery:
                                self.in_recovery_list[-1] = 1

                            # Recover offset ========================================================================
                            if self.checkbox_var.get() == 1:
                                # print('check box checked')
                                # print(self.EKF_offset, self.EKF_offset2)
                                self.Offset_Recovery(self.offset, self.original_command)

                        except ValueError:
                            # Handle the case when data is not in the expected format
                            if data.isdigit():

                                # Data is an integer, you can process it here
                                self.current_sensor = int(data)
                                self.current_sensor_list[-1] = self.current_sensor
                                print(f"Current sensor: {self.current_sensor}")
                                self.sub_window.good_job_img()
                                self.sensor_crossed = True

                                # if self.run_trial_flag:
                                #     self.stage +=1
                                #     if self.stage == len(self.trial_traj):
                                #         self.run_trial_flag = False
                                #
                                #     setting = self.trial_traj[self.stage]
                                #     print("setting", setting)
                                #     self.pick_setting(setting)
                                # else:
                                #     return

                                self.update_plot()

                                self.subject_data['encoder'] = self.number1_values
                                self.subject_data['laser'] = self.number2_values
                                self.subject_data['EstEncoder_Obs'] = self.number1_estimated_values
                                self.subject_data['EstLaser_Obs'] = self.number2_estimated_values
                                self.subject_data['EstEncoder_Unobs'] = self.number1_estimated_values2
                                self.subject_data['EstLaser_Unobs'] = self.number2_estimated_values2
                                self.subject_data['endpoint'] = self.current_sensor_list
                                self.subject_data['time'] = self.time_list
                                # self.subject_data['trial'] = self.trial_numbers
                                self.subject_data['direction'] = self.direction_list
                                self.subject_data['speed'] = self.speed_list
                                self.subject_data['recoveryState'] = self.in_recovery_list
                                self.subject_data['EstSpeed_obs'] = self.estimated_velocity
                                self.subject_data['EstSpeed_unobs'] = self.estimated_velocity2
                                self.subject_data['turtle'] = self.turtle_coords_values
                                self.subject_data['hlfb'] = self.number3_values
                                self.subject_data['arduinoTorque'] = self.number4_values
                                # self.subject_data['torque'] = self.number4_values
                                # self.subject_data['magforce'] = self.number4_values

                                self.save_data()
                                self.subject_data = pd.DataFrame(columns=self.columns)

                            else:
                                print("Invalid data format received from Arduino.")

                except serial.SerialException:
                    print("Lost connection to Arduino.")
                    self.is_reading = False

    def generate_offset_recovery_command(self, offset_value, original_command):
        # going left: B is low, going right: B is high
        # command structure: [sensor_num][A_status][B_status]
        # define_state:
        # going left (A: low, B: low),   top behind (offset<0): change direction to right (B: high)
        # going left (A: low, B: low),   top ahead (offset>0): speed up to left (A: high)
        # going right (A: low, B: high), top behind (offset>0): change direction to left (B: low)
        # going right (A: low, B: high), top ahead (offset<0): speed up to right (A: high)

        digits = [int(char) for char in original_command]
        digits_new = [0, 0, 0]

        if digits[2] == 0: # going left
            if offset_value < 0:
                digits_new = [digits[0], 1, 1] # change to right, speed up
            else:
                digits_new = [digits[0], 1, digits[2]]  # digits[0]

        elif digits[2] == 1: # going right
            if offset_value < 0:
                digits_new = [digits[0], 1, digits[2]]  # digits[0]
            else:
                digits_new = [digits[0], 1, 0] # change to left, speed up

        recovery_command = ''.join(map(str, digits_new))

        return recovery_command

    def Offset_Recovery(self, offset_value, original_command):

        if np.abs(offset_value) > self.threshold and not self.in_recovery:
            recovery_command = self.generate_offset_recovery_command(offset_value, original_command)
            print("recovery_command", recovery_command)
            self.send_command_to_arduino(recovery_command)
            self.in_recovery = True

        elif np.abs(offset_value) <= self.threshold and self.in_recovery:
            print("original_command", original_command)
            self.send_command_to_arduino(original_command)
            self.in_recovery = False


    def update_ekf_state(self, state_mean, state_covariance, measurement_noise_value, process_noise_value, measurement1, measurement2, motor_force):
        # Measurement update step
        measurement = np.array([measurement1, measurement2])
        measurement_noise_covariance = np.eye(2) * measurement_noise_value

        state_mean_updated, state_covariance_updated = measurement_update1(state_mean, state_covariance,
                                                                          measurement, measurement_noise_covariance)

        # Prediction step
        dt = self.dt_seconds  # Time step
        process_noise_covariance = np.eye(4) * process_noise_value

        # Calculate Jacobian of the state transition function at current state and control input
        F = jacob_state(state_transition, state_mean_updated, dt, motor_force)

        state_mean_predicted = state_transition(state_mean_updated, dt, motor_force)
        # Propagate state covariance through state transition function
        state_covariance_predicted = F @ state_covariance_updated @ F.T + process_noise_covariance

        self.state_mean_predicted_list.append(state_mean_predicted)
        self.state_covariance_predicted_list.append(state_covariance_predicted)

        if self.sensor_crossed:
            for i in range(4):
                state_mean_predicted[i] = self.state_mean_predicted_list[-2][i]
            state_covariance_predicted = self.state_covariance_predicted_list[-2]
            # print("observable P:", state_covariance_predicted)
            # self.sensor_crossed = False

        return state_mean_predicted, state_covariance_predicted

    def unobservable_ekf_state(self, state_mean2, state_covariance2, measurement_noise_value2, process_noise_value2, measurement1, motor_force):
        # print(state_mean, state_covariance, measurement_noise_value, process_noise_value, measurement1, motor_force)
        # Measurement update step
        measurement = np.array([measurement1])
        measurement_noise_covariance = np.array([measurement_noise_value2])

        # print(measurement, measurement_noise_covariance)

        state_mean_updated, state_covariance_updated = measurement_update2(state_mean2, state_covariance2,
                                                                          measurement, measurement_noise_covariance)

        # Prediction step
        dt = self.dt_seconds  # Time step
        process_noise_covariance = np.eye(4) * process_noise_value2

        # Calculate Jacobian of the state transition function at current state and control input
        F = jacob_state(state_transition, state_mean_updated, dt, motor_force)

        state_mean_predicted = state_transition(state_mean_updated, dt, motor_force)
        # Propagate state covariance through state transition function
        state_covariance_predicted = F @ state_covariance_updated @ F.T + process_noise_covariance

        self.state_mean_predicted_list2.append(state_mean_predicted)
        self.state_covariance_predicted_list2.append(state_covariance_predicted)

        if self.sensor_crossed:
            for i in range(4):
                state_mean_predicted[i] = self.state_mean_predicted_list2[-2][i]
            state_covariance_predicted = self.state_covariance_predicted_list2[-2]
            # print("P unobservable P:", state_covariance_predicted)
            self.sensor_crossed = False
            print('Jump fixed===========================')

        return state_mean_predicted, state_covariance_predicted

    def turtle_starting_point(self):
        coords_xy = [LEFT, 0, RIGHT]
        coords_name = ["Left", "Center", "Right"]
        coords_xy_mag = [self.mag_dis_list[1], self.mag_dis_list[0], self.mag_dis_list[2]]

        if self.current_sensor is None:
            self.EKF_state_mean = [self.mag_dis_list[coords_name.index(self.combobox_loc.get())], 0, self.mag_dis_list[coords_name.index(self.combobox_loc.get())], 0]
            self.EKF_state_mean2 = [self.mag_dis_list[coords_name.index(self.combobox_loc.get())], 0,
                                   self.mag_dis_list[coords_name.index(self.combobox_loc.get())], 0]

            self.turtle_coords = coords_xy[coords_name.index(self.combobox_loc.get())]
            self.mag_start = [self.mag_dis_list[0], self.mag_dis_list[1], self.mag_dis_list[2]][coords_name.index(self.combobox_loc.get())]

            self.state_mean_predicted_list.append(self.EKF_state_mean)
            self.state_mean_predicted_list2.append(self.EKF_state_mean)
        else:
            self.turtle_coords = [0, LEFT, RIGHT][self.current_sensor]
            self.mag_start = coords_xy_mag[self.current_sensor]

        self.sub_window.user_turtle.goto((self.turtle_coords, 0))
        self.turtle_start_coords = self.turtle_coords
        print("turtle start point: ", self.turtle_start_coords)
        time.sleep(0.2)

    def map_magnet_to_turtle(self, value, from_min, from_max, to_min, to_max):
        proportion = (value - from_min) / (from_max - from_min)
        self.turtle_coords = to_min + proportion * (to_max - to_min)


    # def run_trial(self):
    #     self.turtle_starting_point()
    #     if not self.run_trial_flag:
    #         self.run_trial_flag = True
    #         self.trial_n = 1
    #         print("Trial started --------------")
    #     else:
    #         self.trial_n = 1
    #         print("Trial going ----------------")
    #
    # def stop_trial(self):
    #     self.run_trial_flag = True
    #     print('trial stopped')

    def pick_setting(self, setting): #setting: direction, sensor, a_stat, b_stat, steady_input

        self.trial_n = 1
        self.turtle_starting_point()

        if setting[0] == "Right center":
            self.sub_window.show_center_wall()
            self.sub_window.right_arrow()
            self.set_heading(0)
            self.turtle_stop_coords = 0
            self.mag_stop = self.mag_dis_list[1]
            self.direction = 'right'

        elif setting[0] == "Far right":
            # self.sub_window.show_right_wall()
            self.sub_window.show_right_wall_close()
            self.sub_window.right_arrow()
            self.set_heading(0)
            self.turtle_stop_coords = RIGHT
            self.mag_stop = self.mag_dis_list[2]
            self.direction = 'right'

        elif setting[0] == "Left center":
            self.sub_window.show_center_wall()
            self.sub_window.left_arrow()
            self.set_heading(180)
            self.turtle_stop_coords = 0
            self.mag_stop = self.mag_dis_list[1]
            self.direction = 'left'


        elif setting[0] == "Far left":
            self.sub_window.show_left_wall_close()
            self.sub_window.left_arrow()
            self.set_heading(180)
            self.turtle_stop_coords = LEFT
            self.mag_stop = self.mag_dis_list[0]
            self.direction = 'left'

        digits = setting[1:]
        self.original_command = ''.join(map(str, digits))
        self.send_command_to_arduino(self.original_command)
        print("Going " + self.direction, self.original_command)

    def go_to_destination(self):
        self.trial_n = 0

        self.turtle_starting_point()
        des = self.combobox_des.get()
        self.speed = self.combobox_speed.get()
        a = ["Slow", "Fast"].index(self.speed) # in slow mode A_status is 0 and in fast mode A_status is 1 always.
        s, b = 0, 0 #s is sensor Number to go to

        if des == "Right center":
            s, b = 0, 1
            self.sub_window.show_center_wall()
            self.sub_window.right_arrow()
            self.set_heading(0)
            self.turtle_stop_coords = 0
            self.mag_stop = self.mag_dis_list[1]
            self.direction = 'right'

        elif des == "Far right":
            s, b = 2, 1
            # self.sub_window.show_right_wall()
            self.sub_window.show_right_wall_close()
            self.sub_window.right_arrow()
            self.set_heading(0)
            self.turtle_stop_coords = RIGHT
            self.mag_stop = self.mag_dis_list[2]
            self.direction = 'right'

        elif des == "Left center":
            s, b = 0, 0
            self.sub_window.show_center_wall()
            self.sub_window.left_arrow()
            self.set_heading(180)
            self.turtle_stop_coords = 0
            self.mag_stop = self.mag_dis_list[1]
            self.direction = 'left'


        elif des == "Far left":
            s, b = 1, 0
            self.sub_window.show_left_wall_close()
            self.sub_window.left_arrow()
            self.set_heading(180)
            self.turtle_stop_coords = LEFT
            self.mag_stop = self.mag_dis_list[0]
            self.direction = 'left'

        steady_input = self.steady_label_var.get()
        digits = [s, a, b, steady_input]
        self.original_command = ''.join(map(str, digits))
        self.send_command_to_arduino(self.original_command)
        print("Going " + des, self.original_command)

    def update_plot(self):

        # Clear the previous plot
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        self.ax1.plot(self.number1_values, label='Encoder P1', color='navy')
        self.ax1.plot(self.number2_values, label='Sensor P2', color='darkorange')
        self.ax2.plot(self.number1_estimated_values, label='Estimated P1', color='navy')
        self.ax2.plot(self.number2_estimated_values, label='Estimated P2', color= 'darkorange')
        self.ax3.plot(self.number1_estimated_values2, label='Estimated P1', color='navy')
        self.ax3.plot(self.number2_estimated_values2, label='Estimated P2', color='darkorange')
        self.ax4.plot(self.offset_values, label='Real')
        self.ax4.plot(self.EKF_offset_values, label='EKF')
        # self.ax4.plot(self.EKF_offset_values2, label='P Unobs EKF')
        self.ax4.plot(medfilt(self.EKF_offset_values2, kernel_size=3), label='P Unobs EKF')
        self.ax1.set_title('Distances')
        self.ax2.set_title('Estimated distances (EKF)')
        self.ax3.set_title('Estimated distances (P Unobs EKF)')
        self.ax4.set_title('Offsets')
        self.ax3.set_xlabel('Time')
        self.ax4.set_xlabel('Time')
        self.ax1.set_ylabel('Amplitude (cm)')
        self.ax3.set_ylabel('Amplitude (cm)')
        self.ax1.legend(loc='upper right', fontsize=9)
        self.ax2.legend(loc='upper right', fontsize=9)
        self.ax3.legend(loc='upper right', fontsize=9)
        self.ax4.legend(loc='upper right', fontsize=9)
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.ax3.grid(True)
        self.ax4.grid(True)

        self.ax1.set_ylim(15, 60)
        self.ax2.set_ylim(15, 60)
        self.ax3.set_ylim(15, 60)
        self.ax4.set_ylim(-10, 10)

        self.fig.suptitle("Visualization")

        # Refresh the canvas
        self.canvas.draw()

    def cal_magnetic_force(self, p1, p2):
        p1_arr = np.array(p1)/10
        p2_arr = np.array(p2)/10
        d = 0.312  # Separation distance between the magnets
        h = 1.27  # cm, height of the magnets
        mag_force = (1 / d ** 2 + 1 / (d + 2 * h) ** 2 - 2 / (d + h) ** 2)
        offset = (p1_arr - p2_arr) ** 2
        mag_func = mag_force - ((3 / 2) * (mag_force ** 2) / offset)

        return mag_func

    def map_duty_cycle_to_torque(self, duty_cycle):
        return (duty_cycle - 50) / 45 * 100

    def check_duty_cycle(self, hlfb_value):
        is_high = hlfb_value > self.pwm_threshold
        duty_cycle = None

        # Check for transition from HIGH to LOW to determine a new duty cycle
        if self.prev_value_high and not is_high:
            duty_cycle = (self.high_duration / (self.high_duration + self.low_duration)) * 100
            # Reset durations for the next cycle
            self.high_duration = 0
            self.low_duration = 0
        else:
            # If there's no transition yet, keep counting
            if is_high:
                self.high_duration += 1
            else:
                self.low_duration += 1

        # If we don't have a new duty cycle, we use the most recent one (if available)
        if duty_cycle is None and self.torque_values:
            duty_cycle = self.torque_values[-1]  # Last calculated torque value

        # If we have a duty cycle, compute the torque and store it
        if duty_cycle is not None:
            torque = self.map_duty_cycle_to_torque(duty_cycle)
            self.torque_values.append(torque)
            # self.text_box.insert("1.0", f"<<< torque: {torque:.2f} \n")

        self.prev_value_high = is_high

    def set_heading(self, direction):
        self.sub_window.user_turtle.speed(10)
        self.sub_window.screen.delay(0)
        self.sub_window.user_turtle.setheading(direction)


    def open_planar_exo_window(self):
        self.sub_window = PlanarWindowCreation()
        # self.sub_window.center_wall.showturtle()
        # self.sub_window.right_wall.showturtle()
        # self.sub_window.right_wall_close.showturtle()
        # self.sub_window.left_wall.showturtle()
        # self.sub_window.left_wall_close.showturtle()
        # self.sub_window.right_arrow()
        print("Planar Window Created")

    def send_command_to_arduino(self, command):
        if hasattr(self, 'arduino'):
            command = command.encode()
            self.arduino.write(command)
        else:
            print("Arduino not connected.")

    def connect_to_arduino(self):
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to Arduino on {ARDUINO_PORT}")
            self.start_reading()
        except serial.SerialException:
            print(f"Failed to connect to Arduino on {ARDUINO_PORT}")

    def stop_reading(self):
        self.is_reading = False
        if self.read_thread:
            self.read_thread.join()
            print("Stopped reading from Arduino.")

    def clear_received_data(self):
        self.number1_values = []
        self.number2_values = []
        self.number3_values = []
        self.number4_values = []
        self.torque_values = []
        self.offset_values = []
        self.EKF_offset_values = []
        self.ax1.clear()
        self.ax2.clear()
        # self.ax3.clear()
        # self.ax4.clear()
        self.canvas.draw()

    def close_app(self):
        self.stop_reading()
        if hasattr(self, 'arduino'):
            self.arduino.close()
            print("Disconnected from Arduino.")
        self.master.quit()

    def save_data(self):
        user_input = self.user_entry_var.get()
        self.subject_data.to_csv(user_input+'.csv')


if __name__ == "__main__":
    root = CTk()
    root.geometry("%dx%d+%d+%d" % (1520, 540, 0, 50))
    root.title("Exoskeleton Technician Controls")
    main_window = RootWindow(root)
    root.mainloop()

    # def run_trial(self):
    #     self.turtle_starting_point()
    #
    #     if not self.run_trial_flag:
    #         self.run_trial_flag = True
    #         self.trial_n = 1
    #         print("Trial started --------------")
    #     else:
    #         self.trial_n = 1
    #         print("Trial going ----------------")
    #
    #
    #     if self.stage == self.ultimate_stage:
    #         self.run_trial_flag = False
    #         self.stage = -1
    #         return
    #     elif self.stage != self.ultimate_stage and self.run_trial_flag:
    #         self.stage = self.stage + 1
    #
    #         sensor, a_stat, b_stat, steady_input = 0,0,0,0
    #
    #         if self.stage == 0:
    #             sensor, a_stat, b_stat, steady_input = 2, 1, 0, 0 #center0 to far left2, fast
    #         elif self.stage == 1:
    #             sensor, a_stat, b_stat, steady_input = 0, 1, 1, 1 #far left to center, steady, fast
    #         elif self.stage == 2:
    #             sensor, a_stat, b_stat, steady_input =1, 1, 1, 2 #center to far right1, steady, fast
    #         elif self.stage == 3:
    #             sensor, a_stat, b_stat, steady_input =2, 0, 0, 0 #far right1 to far left2, slow
    #
    #         digits = [sensor, a_stat, b_stat, steady_input]
    #         self.original_command = ''.join(map(str, digits))
    #         self.send_command_to_arduino(self.original_command)
    #         print("Trial Going " , self.original_command)
    #         return
