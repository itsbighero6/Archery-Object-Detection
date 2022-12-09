import math
def pidtune():
    while True:
        lat = vehicle.location.global_relative_frame.lat
        lon = vehicle.location.global_relative_frame.lon
        alt = vehicle.location.global_relative_frame.alt
        newlat = 
        newlon = 
        if get_distance(lat , lon, newlat, newlon) == 30 : #set an arbitrary minimum distance between the drone and the center
            detected = 1
        a = get_xy(lat , lon , newlat, newlon)
        get_pixel_errors(a) 


def get_distance(lat1, lon1, lat2, lon2):
    #distance between two location
    p = 0.017453292519943295     #Pi/180
    a = 0.5 - cos((lat2 - lat1) * p)/2 + cos(lat1 * p) * cos(lat2 * p) * (1 - cos((lon2 - lon1) * p)) / 2
    return 12742 * asin(sqrt(a)) * 1000
def get_dEast(lat, newlong, lon):
    R = 6378137.0
    p = math.pi / 180
    dEast = R*p*(math.cos(p*lat))*(newlong-lon)
    return dEast
def get_dNorth(newlat, lat):
    R = 6378137.0
    p = math.pi / 180
    dNorth = R*p*(newlat - lat)
    return dNorth
def get_xy(lat , lon , newlat, newlon):
    dEast = get_dEast(lat, newlon, lon)
    dNorth = get_dNorth(newlat, lat)
    yaw = vehicle.attitude.yaw
    x = dEast*math.cos(yaw) - dNorth*math.sin(yaw)
    y = dEast*math.sin(yaw) + dNorth*math.cos(yaw)
    xy = [x,y]
    return xy

def get_pixel_errors(xy):
    global x_c
    global y_c
    altitude = vehicle.location.global_relative_frame.alt
    x_fov = 2*altitude*tan(hfov/2)
    y_fov = 2*altitude*tan(vfov/2)
    # suppose the camera is vga 640X480p 
    # x_fov corresponds to 640p therefore
    # 1m corresponds to 
    p_x = 640/x_fov
    p_y = 480/y_fov
    xy = get_xy()
    x = xy[0]
    y = xy[1]
    x_pe = x*p_x
    y_pe = y*p_y
    x_c = int(320 + x_pe)
    y_c = int(240 - y_pe)
