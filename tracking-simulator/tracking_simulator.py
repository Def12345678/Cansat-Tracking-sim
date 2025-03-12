import tkinter as tk
from tkinter import messagebox
import folium
from geopy.distance import geodesic
import serial
import math
from tkintermapview import TkinterMapView
import serial
from serial.tools import list_ports
from tkinter import ttk
import time
import threading
import random

# Serial port setup 
SERIAL_PORT = "COM3"  
BAUD_RATE = 9600
ser = None


# Function to attempt a new serial connection
def set_serial_port():
    global ser, SERIAL_PORT

    # Get the new serial port from the user input
    new_port = serial_port_combobox.get().strip()

    if new_port != SERIAL_PORT:
        SERIAL_PORT = new_port
        try:
            # Try to open the new serial port
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Connected to {SERIAL_PORT}")
        except Exception as e:
            ser = None
            print(f"Failed to connect to {SERIAL_PORT}\n{e}")
        print(f"Serial port set to: {SERIAL_PORT}")
    else:
        print("The serial port is already set to the given port.")


# magical global variable
antenna_coords = None
cansat_coords = None
cansat_altitude = 0
antenna_marker = None
cansat_marker = None
falling = False
initial_cansat_coords = None

# Simulation parameters
fall_speed = 5  # m/s
fall_direction = (0, 0)  # Direction of fall (lat_delta, lon_delta)
radius = 50  # Max radius from the initial drop point

# Simulate functions
def set_fall_direction():
    global fall_direction
    # Randomize the fall direction within a certain radius (e.g., -0.0005 to 0.0005 degrees)
    lat_delta = random.uniform(-0.0000700, 0.0000702)
    lon_delta = random.uniform(-0.0000702, 0.0000702)
    fall_direction = (lat_delta, lon_delta)

def start_fall():
    global falling, initial_cansat_coords
    if falling:
        return
    falling = True
    set_fall_direction()  # Set the random fall direction
    initial_cansat_coords = cansat_coords
    simulate_fall()

def stop_fall():
    global falling
    falling = False

def simulate_fall():
    global cansat_coords, cansat_altitude, falling, cansat_marker, initial_cansat_coords, fall_direction

    if not cansat_coords or not falling:
        return

    cansat_altitude -= fall_speed

    if cansat_altitude <= 0:
        cansat_altitude = 0
        falling = False
        messagebox.showinfo("Cansat Landed", "Cansat has reached the ground!")
        return

    # Calculate the new position based on the fall direction
    new_lat = cansat_coords[0] + fall_direction[0]
    new_lon = cansat_coords[1] + fall_direction[1]


    # Check if the new position is within the radius (50 meters) from the initial position
    distance_from_start = geodesic(initial_cansat_coords, (new_lat, new_lon)).meters
    if distance_from_start <= radius:  # Only move if within the radius
        cansat_coords = (new_lat, new_lon)
    else:
        # If out of radius, reverse the fall direction in a random way
        print(f"Out of radius! Distance from start: {distance_from_start:.2f} meters")

        # Generate a random new direction (within -0.0005 to 0.0005 degrees latitude and longitude)
        lat_delta = random.uniform(-0.0000700, 0.0000700)
        lon_delta = random.uniform(-0.0000700, 0.0000700)
        
        # Reverse the direction by negating the lat and lon deltas
        fall_direction = (-lat_delta, -lon_delta)
        print(f"New random direction: {fall_direction}")

    # Remove the old marker and add a new one
    if cansat_marker:
        map_widget.delete(cansat_marker)
        cansat_marker = None  # Reset the marker reference

    cansat_marker = map_widget.set_marker(cansat_coords[0], cansat_coords[1], text=f"Cansat ({cansat_altitude:.1f}m)")
    
    send_data()  

    # Continue the simulation
    root.after(1000, simulate_fall)  # Continue falling every second

# Function to set the antenna coordinates and marker on the map
def select_antenna(coords):
    global antenna_marker, antenna_coords

    # Delete previous antenna marker if it exists
    if antenna_marker:
        map_widget.delete(antenna_marker)
    # Add a new antenna marker
    antenna_marker = map_widget.set_marker(coords[0], coords[1], text="Antenna")
    antenna_coords = coords
    print(f"Antenna: {coords}")
    # Start updating the distance
    update_distance()
    
# Function to set a fall speed of cansat
def update_fall_speed(event=None):
    try:
        new_speed = float(speed_entry.get())  # Get value
        if new_speed > 0:
            global fall_speed
            fall_speed = new_speed  # Update that shit
            print(f"New speed: {fall_speed} m/s")
        else:
            messagebox.showerror("ERROR", "SPEED MUST BE POSITIVE NUMBER")
    except ValueError:
        messagebox.showerror("ERORR", "GIVE ME THAT NUMBER")

def update_radius(event=None):
    try:
        new_radius = float(radius_entry.get())  # Get value
        if new_radius > 0:
            global radius
            radius = new_radius  # Update that shit
            print(f"New radius: {radius} m")
        else:
            messagebox.showerror("ERROR", "RADIUS MUST BE POSITIVE NUMBER")
        update_circle()
    except ValueError:
        messagebox.showerror("ERORR", "Im politely ask you to give me that")

# Function to draw a radius circle on the map
def update_circle():
    global radius, cansat_coords, map_widget

    if not cansat_coords:
        return  # No cansat coordinates set, don't draw the circle

    # Delete any previous circle (if necessary)
    if hasattr(map_widget, "circle_marker"):
        map_widget.delete(map_widget.circle_marker)
    
    # Calculate the circle coordinates based on the radius in meters
    # Earth radius is approximately 6371 km (or 6371000 meters)
    earth_radius = 6371000  

    # Calculate the offset in degrees for the given radius
    # Approximate one degree of latitude as 111.32 km or 111320 meters
    lat_offset = radius / 111320
    # Longitude offset varies with latitude, so we calculate it as:
    lon_offset = radius / (111320 * math.cos(math.radians(cansat_coords[0])))

    # Create a list of points for the circle (we will approximate it as a polygon)
    circle_points = []
    num_points = 36  # Number of points in the polygon (for smooth circle)
    for i in range(num_points):
        angle = (2 * math.pi * i) / num_points
        # Latitude and Longitude changes in small steps around the circle
        lat_point = cansat_coords[0] + lat_offset * math.sin(angle)
        lon_point = cansat_coords[1] + lon_offset * math.cos(angle)
        circle_points.append((lat_point, lon_point))

    # Draw the circle as a polygon
    map_widget.circle_marker = map_widget.set_polygon(circle_points, fill_color = "red", outline_color = "red", border_width=2)

    print(f"Circle drawn at {cansat_coords} with radius {radius} meters.")

# Function to set the cansat coordinates and marker on the map
def select_cansat(coords):
    global cansat_coords, cansat_marker, cansat_altitude

    if cansat_marker:
        map_widget.delete(cansat_marker)
    
    cansat_coords = coords

    # Check if altitude was previously set
    if cansat_altitude == 0:
        try:
            cansat_altitude = float(altitude_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Incorrect Altitude!")
            return

    cansat_marker = map_widget.set_marker(cansat_coords[0], cansat_coords[1], text=f"Cansat ({cansat_altitude:.1f}m)")
    print(f"Cansat: {cansat_coords}")
    update_distance()
    update_circle()

def cansat_altitude_func(event=None):
    global cansat_altitude
    try:
        cansat_altitude = float(altitude_entry.get()) 
        print("New altitude: ", cansat_altitude)
    except:
        messagebox.showerror("Error", "Incorrect Altitude!")
        return
    

# Function to send data (antenna and cansat coordinates, altitude) to MCU
def send_data():
    global antenna_coords, cansat_coords, cansat_altitude

    try:
        antenna_altitude = float(antenna_altitude_entry.get())
    except:
        antenna_altitude = 0
    
    # Check if both points are selected
    if not antenna_coords or not cansat_coords:
        messagebox.showerror("Error", "Select both points!")
        return
    
    # Try to get cansat altitude from the input field


    # Prepare the data string: latitude and longitude of antenna and cansat, and altitude of cansat
    data_str = f"{antenna_coords[0]:.6f},{antenna_coords[1]:.6f},{antenna_altitude:.2f},{cansat_coords[0]:.6f},{cansat_coords[1]:.6f},{cansat_altitude:.2f}\n"
    
    # Check if the serial connection is available
    if ser:
        ser.write(data_str.encode())
        print("Sent:", data_str)
    else:
        print("No connection with serial port")
        print(data_str)

    
    # Call this function again in 1000 ms (1 second)
    root.after(1000, update_distance)
 
def update_distance():
    if antenna_coords and cansat_coords:
        distance = geodesic(antenna_coords, cansat_coords).meters
        distance_label.config(text=f"Distance: {distance:.2f} meters")
        


# GUI setup
root = tk.Tk()
root.title("Cansat Tracker Simulation")

# Get screen resolution
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Set window size to screen size (maximized window)
root.geometry(f"{screen_width}x{screen_height}")  # Set window to full screen size with taskbar visible

# Maximize window (use system window manager to ensure it behaves like full-screen app)
root.state('normal')  # In case the window is in minimized state initially
root.update_idletasks()

# Main frame to hold left, center, right, and map frames
main_frame = tk.Frame(root)
main_frame.pack(fill="both", expand=True)

# Left frame (for altitude input and serial settings)
left_frame = tk.Frame(main_frame)
left_frame.pack(side="left", padx=10, pady=5, fill="y")

# Frame for Cansat altitude input
altitude_frame = tk.Frame(left_frame)
altitude_frame.pack(padx=10, pady=5, fill="x")

altitude_label = tk.Label(altitude_frame, text="Cansat altitude (m):")
altitude_label.pack(side="left")
altitude_entry = tk.Entry(altitude_frame)
altitude_entry.pack(side="left", fill="x", expand=False)
altitude_entry.bind("<Return>", cansat_altitude_func)

# Frame for Antenna altitude input
antenna_frame = tk.Frame(left_frame)
antenna_frame.pack(padx=10, pady=5, fill="x")

antenna_altitude_label = tk.Label(antenna_frame, text="Antenna altitude (m):")
antenna_altitude_label.pack(side="left")
antenna_altitude_entry = tk.Entry(antenna_frame)
antenna_altitude_entry.pack(side="left", fill="x", expand=False)

# Frame for Send button and serial port settings
serial_frame = tk.Frame(left_frame)
serial_frame.pack(padx=10, pady=5, fill="x")

send_button = tk.Button(serial_frame, text="Send", command=send_data)
send_button.pack(side="left", padx=5)

serial_port_label = tk.Label(serial_frame, text="Select Serial Port:")
serial_port_label.pack(side="left", padx=5)

# Combobox to select available serial ports
serial_ports = serial.tools.list_ports.comports()
serial_port_combobox = ttk.Combobox(serial_frame, values=serial_ports)
serial_port_combobox.set(SERIAL_PORT)  # Set default port
serial_port_combobox.pack(side="left", padx=5)

# Button to set the serial port
set_port_button = tk.Button(serial_frame, text="Set Serial Port", command=set_serial_port)
set_port_button.pack(side="left", padx=5)

# Center frame (for distance label in the middle)
center_frame = tk.Frame(main_frame)
center_frame.pack(side="left", padx=10, pady=5, fill="both", expand=True)

# Frame for Distance label
distance_frame = tk.Frame(center_frame)
distance_frame.pack(padx=10, pady=5, fill="x")

distance_label = tk.Label(distance_frame, text="Distance: 0.00 meters")
distance_label.pack(side="top", fill="x")

# Right frame (for fall speed, radius, and simulation controls)
right_frame = tk.Frame(main_frame)
right_frame.pack(side="right", padx=10, pady=5, fill="y")

# Frame for Fall speed control
fall_speed_frame = tk.Frame(right_frame)
fall_speed_frame.pack(padx=10, pady=5, fill="x")

speed_label = tk.Label(fall_speed_frame, text="Fall speed (m/s):")
speed_label.pack(side="right")
speed_entry = tk.Entry(fall_speed_frame)
speed_entry.insert(0, "5")
speed_entry.pack(side="right", fill="x", expand=False)
speed_entry.bind("<Return>", update_fall_speed)

# Frame for Drop radius control
radius_frame = tk.Frame(right_frame)
radius_frame.pack(padx=10, pady=5, fill="x")

radius_label = tk.Label(radius_frame, text="Drop radius (m):")
radius_label.pack(side="right")
radius_entry = tk.Entry(radius_frame)
radius_entry.insert(0, str(radius))
radius_entry.pack(side="right", fill="x", expand=False)
radius_entry.bind("<Return>", update_radius)

# Frame for Simulation control buttons
simulation_frame = tk.Frame(right_frame)
simulation_frame.pack(padx=10, pady=5, fill="x")

start_button = tk.Button(simulation_frame, text="Start Simulation", command=start_fall)
start_button.pack(side="right", padx=5)

stop_button = tk.Button(simulation_frame, text="Stop Simulation", command=stop_fall)
stop_button.pack(side="right", padx=5)

# Frame for Map widget
map_frame = tk.Frame(root)
map_frame.pack(padx=0, pady=0, fill="both", expand=True)  # Removed extra padding

map_widget = TkinterMapView(map_frame, width=1200, height=1200)
map_widget.pack(fill="both", expand=True)
map_widget.set_position(50.33952679413111, 19.520858590375372)  # Example coordinates
map_widget.set_zoom(14)

# Set right-click menu for setting antenna and cansat
map_widget.add_right_click_menu_command("set Antenna", command=select_antenna, pass_coords=True)
map_widget.add_right_click_menu_command("set Cansat", command=select_cansat, pass_coords=True)

# Start the Tkinter event loop
root.mainloop()
