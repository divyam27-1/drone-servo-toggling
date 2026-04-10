from pymavlink import mavutil
import numpy as np
import json

LAT_MAX, LON_MIN =13.719031, 79.588879
METERS_PER_DEGREE_LAT = 111319.49079
METERS_PER_DEGREE_LON = 111319.49079 * np.cos(np.radians(LAT_MAX))
SERVO_TO_SET = 4
SERVO_PWM_OFF = 900
SERVO_PWM_ON = 1600

def generate_mission_waypoints(mission, outpath, _takeoff=(LAT_MAX, LON_MIN), flight_altitude=10):
    header = "QGC WPL 110"
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

    def write_args(file, args):
        l = len(args)
        for i, arg in enumerate(args):
            if type(arg)==float:
                file.write(f"{arg:.8f}")
            else:
                file.write(f"{arg}")
            if i == l-1:
                file.write("\n")
            else:
                file.write("\t")

    with open(outpath, "w") as f:
        idx = 0
        
        f.write(f"{header}\n")
        #home position
        write_args(f, [
            idx,
            1,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            _takeoff[0],
            _takeoff[1],
            flight_altitude,
            1])
        
        idx += 1
        
        #takeoff command
        write_args(f, [
            idx,                                      #index
            0,                                      #is current wp
            frame,                                  #coord frame
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    #command
            0,                                      #param
            0,                                      #param
            0,                                      #param
            0,                                      #param
            _takeoff[0],                            #lat
            _takeoff[1],                            #lon
            flight_altitude,                        #alt
            1                                       #autocontinue
        ])
        
        idx += 1
        
        #rest of waypoints
        for wp in mission:
            
            # If waypoint toggle command is given, it sets SERVO3 to 1500 (ON) or 1000 (OFF)
            if wp.get("toggle", None) is not None:
                pwm = SERVO_PWM_OFF + (SERVO_PWM_ON - SERVO_PWM_OFF) * int(wp["toggle"])
                write_args(f, [
                    idx,
                    0,
                    frame,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    SERVO_TO_SET,
                    pwm,
                    0,
                    0,
                    0,
                    0,
                    1
                ])
                
                idx += 1
                continue
            
            write_args(f, [
                idx,
                0,
                frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                wp["lat"],
                wp["lon"],
                flight_altitude,
                1
            ])
            
            idx += 1

def generate_mission_kml(waypoints, filename="mission.kml"):
    with open(filename, "w") as f:
        f.write("""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
<name>Mission</name>
<Placemark>
<LineString>
<coordinates>
""")

        for wp in waypoints:
            if "lon" in wp and "lat" in wp:
                f.write(f"{wp['lon']},{wp['lat']},0\n")

        f.write("""
</coordinates>
</LineString>
</Placemark>
</Document>
</kml>
""")

def main():
    with open("motor_toggle_points.json", "r") as f:
        data = json.load(f)
        mission_px = data["path"]
        gsd = data["gsd"]
        
        for wp in mission_px:
            wp["lat"] = LAT_MAX - (wp["y"] * gsd / 100) / METERS_PER_DEGREE_LAT
            wp["lon"] = LON_MIN + (wp["x"] * gsd / 100) / METERS_PER_DEGREE_LON
    
    generate_mission_waypoints(mission_px, "mission.waypoints")
    generate_mission_kml(mission_px, "mission.kml")
    
if __name__ == "__main__":
    main()