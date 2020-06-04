#!/usr/bin/env python

# GUI packages
import tkMessageBox
from Tkinter import *
from tkinter import scrolledtext
# ROS packages
import roslib
import rospy
from std_msgs.msg import UInt32
from mavros_msgs.msg import RadioStatus
# Other 3rd party packages
from datetime import datetime
from pygeodesy import GeoidKarney
from uav_executive.preflightpdf import PreFlightPDF


class PreFlightCheck(object):
    def __init__(self, uav, update_frequency=10.):
        """
        Class for Pre-flight check interface
        """
        self.uav = uav
        self.gps_raw = dict()
        self.gps_text = None
        self.rel_alt_text = None
        self.telemetry_text = None
        self.geoid_interpolator = GeoidKarney(
            roslib.packages.get_pkg_dir('mimree_executive') +
            '/config/egm96-5.pgm')

        # Subscribers
        self._radio_status_sub = rospy.Subscriber('/%s/mavros/radio_status' %
                                                  self.uav.namespace,
                                                  RadioStatus,
                                                  self._radio_cb,
                                                  queue_size=10)
        self._hil_gps_sub = rospy.Subscriber(
            '/%s/mavros/global_position/raw/satellites' % self.uav.namespace,
            UInt32,
            self._satellites_cb,
            queue_size=10)

        # Auto call functions
        self._rel_alt_timer = rospy.Timer(
            rospy.Duration(1. / update_frequency), self._rel_alt_status)

        # interface
        self._main_window = Tk()
        self._main_window.wm_title('Pre-Flight Checklist V2.10')
        self._create_upper_frame()
        self._create_middle_frame()
        self._create_lower_frame()
        self._create_param_frame()
        self._create_submit_frame()
        self._main_window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self._main_window.mainloop()

    def _radio_cb(self, msg):
        """
        Radio status callback
        """
        if self.telemetry_text is not None:
            self.telemetry_text.set('Radio signal RSSI: %d, REMRSSI: %d' %
                                    (msg.rssi, msg.remrssi))

    def _satellites_cb(self, msg):
        """
        Raw GPS data callback
        """
        if self.gps_text is not None:
            self.gps_text.set('No. of satellites: %d' % msg.data)

    def _rel_alt_status(self, event):
        """
        Relative altitude callback
        """
        if self.rel_alt_text is not None:
            self.rel_alt_text.set('Relative Altitude: %.2f' % self.uav.rel_alt)

    def _create_upper_frame(self):
        upper_frame = LabelFrame(self._main_window, text='', padx=5, pady=5)
        upper_frame.pack(side=TOP)
        first_col_frame = LabelFrame(upper_frame, text='', padx=5, pady=6)
        first_col_frame.pack(side=LEFT)
        Label(first_col_frame, text='Aircraft:').pack(side=TOP, anchor=E)
        Label(first_col_frame, text='Date:').pack(side=TOP, anchor=E)
        Label(first_col_frame, text='Pilot:').pack(side=TOP, anchor=E)
        Label(first_col_frame, text='Ground Control:').pack(side=TOP, anchor=E)
        second_col_frame = LabelFrame(upper_frame, text='', padx=2, pady=2)
        second_col_frame.pack(side=LEFT)
        self._uav_entry = StringVar()
        self._uav_entry.set(self.uav.namespace.upper())
        Entry(second_col_frame,
              width=20,
              state='disabled',
              textvariable=self._uav_entry).pack(side=TOP)
        self._date_entry = StringVar()
        self._date_entry.set(datetime.now().date())
        Entry(second_col_frame,
              width=20,
              state='disabled',
              textvariable=self._date_entry).pack(side=TOP)
        self._pilot = StringVar()
        Entry(second_col_frame, width=20,
              textvariable=self._pilot).pack(side=TOP)
        self._gc = StringVar()
        Entry(second_col_frame, width=20, textvariable=self._gc).pack(side=TOP)
        third_col_frame = LabelFrame(upper_frame, text='', padx=5, pady=6)
        third_col_frame.pack(side=LEFT)
        Label(third_col_frame, text='Flight No:').pack(side=TOP, anchor=E)
        Label(third_col_frame, text='Location:').pack(side=TOP, anchor=E)
        Label(third_col_frame, text='Weather Acceptible:').pack(side=TOP,
                                                                anchor=E)
        Label(third_col_frame, text='Weather Description:').pack(side=TOP,
                                                                 anchor=E)
        fourth_col_frame = LabelFrame(upper_frame, text='', padx=2, pady=2)
        fourth_col_frame.pack(side=LEFT)
        self._flight_no = StringVar()
        self._flight_no.set('1')
        Entry(fourth_col_frame, width=20,
              textvariable=self._flight_no).pack(side=TOP)
        self._location = StringVar()
        Entry(fourth_col_frame, width=20,
              textvariable=self._location).pack(side=TOP)
        self._weather_acception = StringVar()
        Entry(fourth_col_frame, width=20,
              textvariable=self._weather_acception).pack(side=TOP)
        self._weather_description = StringVar()
        Entry(fourth_col_frame,
              width=20,
              textvariable=self._weather_description).pack(side=TOP)

    def _create_middle_frame(self):
        middle_frame = LabelFrame(self._main_window,
                                  text='Mission Plan',
                                  padx=5,
                                  pady=5)
        middle_frame.pack(side=TOP)
        first_col_frame = LabelFrame(middle_frame, text='', padx=5, pady=4)
        first_col_frame.pack(side=LEFT)
        Label(first_col_frame, text='Takeoff Level AMSL (m):').pack(side=TOP,
                                                                    anchor=E)
        Label(first_col_frame, text='Planned Maximum AGL (m):').pack(side=TOP,
                                                                     anchor=E)
        second_col_frame = LabelFrame(middle_frame, text='', padx=2, pady=2)
        second_col_frame.pack(side=LEFT)
        self._takeoff_amsl = StringVar()
        takeoff_amsl = (
            self.uav.home.geo.altitude - self.geoid_interpolator.height(
                self.uav.home.geo.latitude, self.uav.home.geo.longitude))
        self._takeoff_amsl.set('%.2f' % takeoff_amsl)
        Entry(second_col_frame,
              width=13,
              state='disabled',
              textvariable=self._takeoff_amsl).pack(side=TOP)
        self._planned_agl = StringVar()
        for i in range(10):
            if [wp.z_alt for wp in self.uav.waypoints] == list():
                rospy.logwarn('No waypoints are uploaded! Retry...')
                rospy.sleep(1)
            else:
                break
        try:
            self._planned_agl.set('%.2f' %
                                  max([wp.z_alt for wp in self.uav.waypoints]))
        except ValueError:
            self._planned_agl.set(0.0)
        Entry(second_col_frame, width=13,
              textvariable=self._planned_agl).pack(side=TOP)
        third_col_frame = LabelFrame(middle_frame, text='', padx=5, pady=15)
        third_col_frame.pack(side=LEFT)
        Label(third_col_frame, text='Mission Objective:').pack(side=TOP,
                                                               anchor=E)
        fourth_col_frame = LabelFrame(middle_frame, text='', padx=2, pady=5)
        fourth_col_frame.pack(side=LEFT)
        self._mission_obj = scrolledtext.ScrolledText(fourth_col_frame,
                                                      width=16,
                                                      height=2)
        self._mission_obj.pack(side=TOP)
        self._mission_obj.insert(INSERT, 'Trial')

    def _create_lower_frame(self):
        lower_frame = LabelFrame(self._main_window,
                                 text='Action Checklist',
                                 padx=5,
                                 pady=5)
        lower_frame.pack(side=TOP)
        # first column
        width = 15
        Label(lower_frame, text='Item', bg='grey', width=width,
              justify=CENTER).grid(row=0, column=0)
        Label(lower_frame,
              text='RF Noise',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=1, column=0)
        Label(lower_frame,
              text='Transmitter',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=2, column=0, rowspan=2)
        Label(lower_frame,
              text='Airframe',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=4, column=0)
        Label(lower_frame,
              text='Flight Battery',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=5, column=0, rowspan=2)
        Label(lower_frame, text='Hatch', justify=LEFT, width=width,
              anchor=E).grid(row=7, column=0)
        Label(lower_frame,
              text='Transmitter Range',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=8, column=0)
        Label(lower_frame,
              text='Ground Station',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=9, column=0)
        Label(lower_frame,
              text='Telemetry',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=10, column=0)
        Label(lower_frame,
              text='Transmitter',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=11, column=0)
        Label(lower_frame,
              text='Magnetometer',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=12, column=0)
        Label(lower_frame,
              text='Barometer',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=13, column=0)
        Label(lower_frame, text='GPS', justify=LEFT, width=width,
              anchor=E).grid(row=14, column=0)
        Label(lower_frame, text='AHRS', justify=LEFT, width=width,
              anchor=E).grid(row=15, column=0)
        Label(lower_frame, text='Motor', justify=LEFT, width=width,
              anchor=E).grid(row=16, column=0)
        Label(lower_frame,
              text='Air Traffic',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=17, column=0)
        Label(lower_frame,
              text='People & Weather',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=18, column=0)
        Label(lower_frame, text='Camera', justify=LEFT, width=width,
              anchor=E).grid(row=19, column=0)
        Label(lower_frame,
              text='Go / NoGo',
              justify=LEFT,
              width=width,
              anchor=E).grid(row=20, column=0, rowspan=2)
        # second column
        width = 40
        Label(lower_frame,
              text='Action To Complete',
              bg='grey',
              width=width,
              justify=CENTER).grid(row=0, column=1)
        Label(lower_frame, text='All local RF noise sources on?',
              width=width).grid(row=1, column=1)
        Label(lower_frame, text='Voltage Taranis (V)',
              width=width).grid(row=2, column=1)
        Label(lower_frame, text='Voltage Dragon Link (V)',
              width=width).grid(row=3, column=1)
        Message(lower_frame,
                text=('Final check on airframe including props.\n' +
                      'Payloads attached and running?\n' +
                      'Appropriate SD cards?\nAre cameras running?'),
                aspect=350,
                justify=CENTER).grid(row=4, column=1)
        Label(lower_frame, text='Initial main battery (V)',
              width=width).grid(row=5, column=1)
        Label(lower_frame,
              text='Agree low level battery to RTL (V)',
              width=width).grid(row=6, column=1)
        Label(lower_frame, text='Is hatch secured?',
              width=width).grid(row=7, column=1)
        Label(lower_frame, text='Is range check current?',
              width=width).grid(row=8, column=1)
        Message(lower_frame,
                text='Are %d waypoints for the mission loaded and verified?' %
                len(self.uav.waypoints),
                aspect=350,
                justify=CENTER).grid(row=9, column=1)
        self.telemetry_text = StringVar()
        self.telemetry_text.set('Radio signal RSSI: -, REMRSSI: -')
        Label(lower_frame, textvariable=self.telemetry_text,
              width=width).grid(row=10, column=1)
        Label(lower_frame,
              text='Check mode functions. Check RTL switch',
              width=width).grid(row=11, column=1)
        Label(lower_frame, text='Calibrated? Check directions.',
              width=width).grid(row=12, column=1)
        Label(lower_frame, text='Altimeter within 10m of ground?',
              width=width).grid(row=13, column=1)
        self.gps_text = StringVar()
        self.gps_text.set('HDOP: -, No. of satellites: -')
        Label(lower_frame, textvariable=self.gps_text,
              width=width).grid(row=14, column=1)
        Label(lower_frame,
              text='Check aircraft level and roll angle.',
              width=width).grid(row=15, column=1)
        Label(lower_frame,
              text='Arm aircraft. Place in catapult.',
              width=width).grid(row=16, column=1)
        Label(lower_frame,
              text='Visual Check. Clear? Clearance requested?',
              width=width).grid(row=17, column=1)
        Label(lower_frame,
              text='Crew in safe position? Weather update?',
              width=width).grid(row=18, column=1)
        Label(lower_frame, text='Camera on and recording?',
              width=width).grid(row=19, column=1)
        Label(lower_frame, text='Final check for pilot and GCS',
              width=width).grid(row=20, column=1, rowspan=2)
        # third colum
        width = 10
        Label(lower_frame,
              text='Findings',
              bg='grey',
              width=20,
              justify=CENTER).grid(row=0, column=2)
        self._rf_noise = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._rf_noise,
                    width=width,
                    justify=CENTER).grid(row=1, column=2)
        self._volt_taranis = StringVar()
        Entry(lower_frame, width=width,
              textvariable=self._volt_taranis).grid(row=2, column=2)
        self._volt_dragon = StringVar()
        Entry(lower_frame, width=width,
              textvariable=self._volt_dragon).grid(row=3, column=2)
        self._airframe = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._airframe,
                    width=width,
                    justify=CENTER).grid(row=4, column=2)
        self._init_batt = StringVar()
        Entry(lower_frame, width=width,
              textvariable=self._init_batt).grid(row=5, column=2)
        self._low_batt = StringVar()
        Entry(lower_frame, width=width,
              textvariable=self._low_batt).grid(row=6, column=2)
        self._hatch = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._hatch,
                    width=width,
                    justify=CENTER).grid(row=7, column=2)
        self._range_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._range_check,
                    width=width,
                    justify=CENTER).grid(row=8, column=2)
        self._ground_station = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._ground_station,
                    width=width,
                    justify=CENTER).grid(row=9, column=2)
        self._telemetry_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._telemetry_check,
                    width=width,
                    justify=CENTER).grid(row=10, column=2)
        self._transmitter_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._transmitter_check,
                    width=width,
                    justify=CENTER).grid(row=11, column=2)
        self._magnetometer_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._magnetometer_check,
                    width=width,
                    justify=CENTER).grid(row=12, column=2)
        self._barometer_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._barometer_check,
                    width=width,
                    justify=CENTER).grid(row=13, column=2)
        self._gps_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._gps_check,
                    width=width,
                    justify=CENTER).grid(row=14, column=2)
        self._ahrs_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._ahrs_check,
                    width=width,
                    justify=CENTER).grid(row=15, column=2)
        self._motor_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._motor_check,
                    width=width,
                    justify=CENTER).grid(row=16, column=2)
        self._airtraffic_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._airtraffic_check,
                    width=width,
                    justify=CENTER).grid(row=17, column=2)
        self._people_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._people_check,
                    width=width,
                    justify=CENTER).grid(row=18, column=2)
        self._camera_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='',
                    var=self._camera_check,
                    width=width,
                    justify=CENTER).grid(row=19, column=2)
        self._pilot_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='Pilot',
                    var=self._pilot_check,
                    width=width,
                    justify=CENTER).grid(row=20, column=2)
        self._gcs_check = BooleanVar()
        Checkbutton(lower_frame,
                    text='GCS',
                    var=self._gcs_check,
                    width=width,
                    justify=CENTER).grid(row=21, column=2)

    def _create_param_frame(self):
        param_frame = LabelFrame(self._main_window,
                                 text='Flight Parameters',
                                 padx=5,
                                 pady=5)
        param_frame.pack(side=TOP)
        # first column
        width = 42
        self.rel_alt_text = StringVar()
        self.rel_alt_text.set('Relative Altitude: -')
        Label(param_frame, textvariable=self.rel_alt_text,
              width=width).grid(row=0, column=0)
        # second column
        width = 30
        Button(param_frame,
               text='Reboot Autopilot/Onboard',
               command=self._reboot_confirmation_box,
               width=width).grid(row=0, column=1)

    def _reboot_confirmation_box(self):
        if tkMessageBox.askyesno(
                'Confirmation',
                'Are you sure to reboot autopilot and onboard computer?'):
            self.uav.reboot_autopilot_computer()
            while not self.uav.add_waypoints():
                self.uav._rate.sleep()

    def _create_submit_frame(self):
        submit_frame = LabelFrame(self._main_window, text='', padx=5, pady=5)
        submit_frame.pack(side=TOP)
        Button(submit_frame,
               text='Submit',
               command=self._confirmation_box,
               width=73).pack(anchor=CENTER, fill=X)

    def _confirmation_box(self):
        if tkMessageBox.askyesno('Confirmation',
                                 'Are you sure to submit this form?'):
            self._submit()

    def _submit(self):
        mission_obj = str(self._mission_obj.get('1.0', END))
        rssi = str(self.telemetry_text.get().split()[-3])
        remrssi = str(self.telemetry_text.get().split()[-1])
        hdop = str(self.gps_text.get().split()[1][:-1])
        num_of_sats = str(self.gps_text.get().split()[-1])
        try:
            takeoff_amsl = '%.2f' % float(self._takeoff_amsl.get())
            planned_agl = '%.2f' % float(self._planned_agl.get())
            volt_taranis = '%.2f' % float(self._volt_taranis.get())
            volt_dragon = '%.2f' % float(self._volt_dragon.get())
            init_batt = '%.2f' % float(self._init_batt.get())
            low_batt = '%.2f' % float(self._low_batt.get())
        except ValueError:
            takeoff_amsl = '%.2f' % self.geoid_interpolator.height(
                self.uav.home.geo.latitude, self.uav.home.geo.longitude)
            if len(self.uav.waypoints):
                planned_agl = '%.2f' % max(
                    [wp.z_alt for wp in self.uav.waypoints])
            else:
                planned_agl = '-'
            volt_taranis = '-'
            volt_dragon = '-'
            init_batt = '-'
            low_batt = '-'
        # create pdf file
        data = {
            'uav_entry': self._uav_entry.get(),
            'date_entry': self._date_entry.get(),
            'pilot': self._pilot.get(),
            'gc': self._gc.get(),
            'flight_no': self._flight_no.get(),
            'location': self._location.get(),
            'weather_acception': self._weather_acception.get(),
            'weather_description': self._weather_description.get(),
            'takeoff_amsl': takeoff_amsl,
            'planned_agl': planned_agl,
            'mission_obj': mission_obj,
            'num_of_wps': str(len(self.uav.waypoints)),
            'rssi': rssi,
            'remrssi': remrssi,
            'hdop': hdop,
            'num_of_sats': num_of_sats,
            'rf_noise': 'V' if self._rf_noise.get() else 'X',
            'volt_taranis': volt_taranis,
            'volt_dragon': volt_dragon,
            'airframe': 'V' if self._airframe.get() else 'X',
            'init_batt': init_batt,
            'low_batt': low_batt,
            'hatch': 'V' if self._hatch.get() else 'X',
            'range_check': 'V' if self._range_check.get() else 'X',
            'ground_station': 'V' if self._ground_station.get() else 'X',
            'telemetry_check': 'V' if self._telemetry_check.get() else 'X',
            'transmitter_check': 'V' if self._transmitter_check.get() else 'X',
            'magnetometer_check':
            'V' if self._magnetometer_check.get() else 'X',
            'barometer_check': 'V' if self._barometer_check.get() else 'X',
            'gps_check': 'V' if self._gps_check.get() else 'X',
            'ahrs_check': 'V' if self._ahrs_check.get() else 'X',
            'motor_check': 'V' if self._motor_check.get() else 'X',
            'airtraffic_check': 'V' if self._airtraffic_check.get() else 'X',
            'people_check': 'V' if self._people_check.get() else 'X',
            'camera_check': 'V' if self._camera_check.get() else 'X',
            'pilot_check': 'V' if self._pilot_check.get() else 'X',
            'gcs_check': 'V' if self._gps_check.get() else 'X'
        }
        PreFlightPDF(data)
        # unsubscribing and destroying window
        self.__del__()

    def __del__(self):
        # unsubscribing and destroying window
        self._radio_status_sub.unregister()
        self._hil_gps_sub.unregister()
        self._rel_alt_timer.shutdown()
        rospy.sleep(1.)
        self._main_window.destroy()

    def on_closing(self):
        if tkMessageBox.askokcancel("Quit", "Do you want to quit?"):
            # unsubscribing and destroying window
            self.__del__()
