#!/usr/bin/env python

import os
from datetime import datetime
from reportlab.pdfgen import canvas
# ROS Package
import roslib
import rospy


class PreFlightPDF(object):
    def __init__(self, data):
        """
        Class for print Pre-flight check to PDF format
        """

        pkg_path = roslib.packages.get_pkg_dir('mimree_executive')
        if not os.path.exists(pkg_path + '/docs/'):
            os.makedirs(pkg_path + '/docs/')
        self.canvas = canvas.Canvas(
            pkg_path + '/docs/preflight_check_%s.pdf' %
            datetime.fromtimestamp(rospy.Time.now().secs))

        self.canvas.setLineWidth(.3)
        self.canvas.setTitle('Pre-Flight Checklist')

        self.canvas.setFont('Helvetica', 18)
        self.canvas.drawString(200, 780, 'Pre-Flight Checklist')

        self.canvas.setFont('Helvetica', 12)
        self._create_upper_table(data)
        self._create_middle_table(data)
        self._create_lower_table(data)
        self.canvas.save()

    def _create_upper_table(self, data):
        """
        Generating upper table on pdf file for pre-flight check
        """
        self.canvas.line(50, 770, 540, 770)
        self.canvas.drawString(70, 750, 'Aircraft')
        self.canvas.drawString(70, 735, 'Date')
        self.canvas.drawString(70, 720, 'Pilot')
        self.canvas.drawString(70, 705, 'Ground Control')
        self.canvas.drawString(160, 750, ':')
        self.canvas.drawString(160, 735, ':')
        self.canvas.drawString(160, 720, ':')
        self.canvas.drawString(160, 705, ':')
        self.canvas.drawString(170, 750, data['uav_entry'])
        self.canvas.drawString(170, 735, data['date_entry'])
        self.canvas.drawString(170, 720, data['pilot'])
        self.canvas.drawString(170, 705, data['gc'])
        self.canvas.drawString(330, 750, 'Flight No.')
        self.canvas.drawString(330, 735, 'Location')
        self.canvas.drawString(330, 720, 'Weather Acceptible')
        self.canvas.drawString(330, 705, 'Weather Description')
        self.canvas.drawString(440, 750, ':')
        self.canvas.drawString(440, 735, ':')
        self.canvas.drawString(440, 720, ':')
        self.canvas.drawString(440, 705, ':')
        self.canvas.drawString(450, 750, data['flight_no'])
        self.canvas.drawString(450, 735, data['location'])
        self.canvas.drawString(450, 720, data['weather_acception'])
        self.canvas.drawString(450, 705, data['weather_description'])
        self.canvas.line(50, 695, 540, 695)

    def _create_middle_table(self, data):
        """
        Generating middle table on pdf file for pre-flight check
        """
        self.canvas.line(60, 690, 520, 690)
        self.canvas.line(60, 690, 60, 665)
        self.canvas.line(520, 690, 520, 665)
        self.canvas.drawString(255, 675, 'Mission Plan')
        self.canvas.line(60, 665, 520, 665)
        self.canvas.line(60, 618, 520, 618)
        self.canvas.line(60, 665, 60, 618)
        self.canvas.line(520, 665, 520, 618)
        self.canvas.drawString(70, 645, 'Takeoff Level AMSL (m)')
        self.canvas.drawString(70, 630, 'Mission Objective')
        self.canvas.drawString(200, 645, ':')
        self.canvas.drawString(210, 645, data['takeoff_amsl'])
        self.canvas.drawString(200, 630, ':')
        self.canvas.drawString(210, 630, data['mission_obj'])
        self.canvas.drawString(310, 645, 'Maximum Altitude AGL (m)')
        self.canvas.drawString(455, 645, ':')
        self.canvas.drawString(465, 645, data['planned_agl'])

    def _create_lower_table(self, data):
        """
        Generating lower table on pdf file for pre-flight check
        """
        self.canvas.line(60, 600, 520, 600)
        self.canvas.line(60, 600, 60, 575)
        self.canvas.line(520, 600, 520, 575)
        self.canvas.line(60, 575, 520, 575)
        self.canvas.line(60, 600, 60, 195)
        self.canvas.drawString(107, 585, 'Item')
        self.canvas.line(180, 600, 180, 195)
        self.canvas.drawString(70, 560, 'RF Noise')
        self.canvas.drawString(70, 537.5, 'Transmitter(s)')
        self.canvas.drawString(70, 492.5, 'Airframe')
        self.canvas.drawString(70, 447.5, 'Flight Battery')
        self.canvas.drawString(70, 425, 'Hatch')
        self.canvas.drawString(70, 410, 'Transmitter Range')
        self.canvas.drawString(70, 395, 'Ground Station')
        self.canvas.drawString(70, 380, 'Telemetry')
        self.canvas.drawString(70, 365, 'Transmitter')
        self.canvas.drawString(70, 350, 'Magnetometer')
        self.canvas.drawString(70, 335, 'Barometer')
        self.canvas.drawString(70, 320, 'GPS')
        self.canvas.drawString(70, 305, 'AHRS')
        self.canvas.drawString(70, 290, 'Motor')
        self.canvas.drawString(70, 275, 'Air Traffic')
        self.canvas.drawString(70, 260, 'People & Weather')
        self.canvas.drawString(70, 245, 'Camera')
        self.canvas.drawString(70, 222.5, 'Go / NoGo')
        self.canvas.drawString(70, 200, 'Power Up')
        self.canvas.drawString(278, 585, 'Action To Complete')
        self.canvas.line(480, 600, 480, 195)
        self.canvas.drawString(190, 560, 'All local RF noise source on?')
        self.canvas.drawString(190, 545, 'Voltage Taranis (V)')
        self.canvas.drawString(190, 530, 'Voltage Dragon Link (V)')
        self.canvas.drawString(190, 515,
                               'Final check on airframe including props.')
        self.canvas.drawString(190, 500, 'Payloads attached and running?')
        self.canvas.drawString(190, 485, 'Appropriate SD cards?')
        self.canvas.drawString(190, 470, 'Are cameras running?')
        self.canvas.drawString(190, 455, 'Initial main battery (V)')
        self.canvas.drawString(190, 440, 'Agree low level battery to RTL (V)')
        self.canvas.drawString(190, 425, 'Is hatch secured?')
        self.canvas.drawString(190, 410, 'Is range check current?')
        self.canvas.drawString(
            190, 395, 'Are %s waypoints for the mission loaded and verified?' %
            data['num_of_wps'])
        self.canvas.drawString(
            190, 380, 'Radio signal RSSI: %s, REMRSSI: %s' %
            (data['rssi'], data['remrssi']))
        self.canvas.drawString(190, 365,
                               'Check mode functions. Check RTL switch.')
        self.canvas.drawString(190, 350, 'Calibrated? Check directions.')
        self.canvas.drawString(190, 335, 'Altimeter within 10m of ground?')
        self.canvas.drawString(
            190, 320, 'HDOP: %s, No. of satellites: %s' %
            (data['hdop'], data['num_of_sats']))
        self.canvas.drawString(190, 305,
                               'Check aircraft level and roll angle.')
        self.canvas.drawString(190, 290, 'Arm aircraft. Place in catapult.')
        self.canvas.drawString(190, 275,
                               'Visual Check. Clear? Clearance requested?')
        self.canvas.drawString(190, 260,
                               'Crew in safe position? Weather update?')
        self.canvas.drawString(190, 245, 'Camera on and recording?')
        self.canvas.drawString(190, 222.5, 'Final check for pilot and GCS.')
        self.canvas.drawString(190, 200, 'Take-off time.')
        self.canvas.drawString(495, 585, 'V')
        self.canvas.line(520, 600, 520, 195)
        self.canvas.drawString(495, 560, data['rf_noise'])
        self.canvas.drawString(485, 545, data['volt_taranis'])
        self.canvas.drawString(485, 530, data['volt_dragon'])
        self.canvas.drawString(495, 492.5, data['airframe'])
        self.canvas.drawString(485, 455, data['init_batt'])
        self.canvas.drawString(485, 440, data['low_batt'])
        self.canvas.drawString(495, 425, data['hatch'])
        self.canvas.drawString(495, 410, data['range_check'])
        self.canvas.drawString(495, 395, data['ground_station'])
        self.canvas.drawString(495, 380, data['telemetry_check'])
        self.canvas.drawString(495, 365, data['transmitter_check'])
        self.canvas.drawString(495, 350, data['magnetometer_check'])
        self.canvas.drawString(495, 335, data['barometer_check'])
        self.canvas.drawString(495, 320, data['gps_check'])
        self.canvas.drawString(495, 305, data['ahrs_check'])
        self.canvas.drawString(495, 290, data['motor_check'])
        self.canvas.drawString(495, 275, data['airtraffic_check'])
        self.canvas.drawString(495, 260, data['people_check'])
        self.canvas.drawString(495, 245, data['camera_check'])
        self.canvas.drawString(495, 230, data['pilot_check'])
        self.canvas.drawString(495, 215, data['gcs_check'])
        time = str(datetime.fromtimestamp(rospy.Time.now().secs).time())
        time = time[:-3]
        self.canvas.drawString(485, 200, time)
        self.canvas.line(60, 195, 520, 195)
