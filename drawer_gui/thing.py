#!/usr/bin/env python

import rospy
from temperature_conversion.srv import TempConvert, TempConvertResponse

def convert_temperature(req):
    """Convert temperature based on the unit provided in the request."""
    if req.unit == "C":
        converted_temp = (req.temperature * 9/5) + 32  # Celsius to Fahrenheit
        rospy.loginfo("Converting from Celsius to Fahrenheit: %f -> %f", req.temperature, converted_temp)
    elif req.unit == "F":
        converted_temp = (req.temperature - 32) * 5/9  # Fahrenheit to Celsius
        rospy.loginfo("Converting from Fahrenheit to Celsius: %f -> %f", req.temperature, converted_temp)
    else:
        rospy.logerr("Invalid unit: %s", req.unit)
        return TempConvertResponse(0.0)  # Return 0.0 for invalid unit

    return TempConvertResponse(converted_temp)

def temperature_conversion_server():
    """Initialize the temperature conversion service."""
    rospy.init_node('temperature_conversion_server')
    s = rospy.Service('TempConvert', TempConvert, convert_temperature)
    rospy.loginfo("Temperature conversion service ready.")
    rospy.spin()

if __name__ == "__main__":
    temperature_conversion_server()
