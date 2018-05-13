#!/usr/bin/env python

from __future__ import division

import rospy
import json
import csv
import datetime

from std_msgs.msg import String


output_path = "../output_data/calculated/"



def record_calculated_velocity(jsonString):
    # Convert json string to object
    jsonData = json.loads(jsonString.data)

    # print(jsonData)

    # Extract Json Data
    velocity = jsonData['velocity']
    previous_timestamp = jsonData['previousTimestamp']
    current_timestamp = jsonData['currentTimestamp']

    # Set recorded time to be midpoint of previous and current times
    time = (previous_timestamp + current_timestamp) / 2

    # print(velocity)
    # print(previous_timestamp)
    # print(current_timestamp)

    csvwriter.writerow([time, velocity])


if __name__ == '__main__':
    # Create a csv file to write output to
    dt = datetime.datetime.now()
    dt_time = str(dt.time())[:8]
    dt_date = str(dt.date())
    file_name = dt_time + "-" + dt_date + ".csv"
    print(file_name)

    global csvwriter

    with open(output_path + file_name, "wb") as csvfile:
        csvwriter = csv.writer(csvfile, dialect='excel', delimiter=',')

        # Setup ros node and publisher
        rospy.init_node('record_calculated_velocity', anonymous=True)

        rospy.Subscriber('/terrapin/json_message', String, record_calculated_velocity)

        rospy.spin()
