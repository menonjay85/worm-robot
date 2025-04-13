import omni
from omni.isaac.range_sensor import _range_sensor
import matplotlib.pyplot as plt
import time
from collections import deque
import numpy as np

# --- Constants/paths ---
ULTRASONIC_PATH = "/World/UltrasonicArray"  # Change if your sensor is at a different path

# 1) Acquire the ultrasonic sensor interface
ul = _range_sensor.acquire_ultrasonic_sensor_interface()

# 2) Grab Isaac Sim's timeline so we know if simulation is playing
timeline = omni.timeline.get_timeline_interface()

# 3) Set up a deque to hold envelope rows for a rolling 10-second window
window_duration = 10.0  # seconds
envelope_buffer = deque()  # Each element will be a tuple: (timestamp, envelope_row)

# 4) Prepare Matplotlib for live updates (scrolling envelope display)
plt.ion()
fig2, ax2 = plt.subplots()
ax2.set_title("Scrolling Envelope Data (last 10 seconds)")
ax2.set_xlabel("Bin Index")
ax2.set_ylabel("Time (s)")
# We'll initialize the image object later when we have data
im2 = None

# 5) Define our update callback
def on_update(e):
    global im2, envelope_buffer

    # Only process sensor data if the simulation is playing
    if not timeline.is_playing():
        return

    # --- Retrieve and print sensor angular data ---
    # Get azimuth data (horizontal angles)
    azimuth = ul.get_azimuth_data(ULTRASONIC_PATH)
    if azimuth is not None and len(azimuth) > 0:
        print("Azimuth values:", azimuth)
    else:
        print("No azimuth data found.")

    # Get zenith data (vertical angles)
    zenith = ul.get_zenith_data(ULTRASONIC_PATH)
    if zenith is not None and len(zenith) > 0:
        print("Zenith values:", zenith)
    else:
        print("No zenith data found.")

    # --- Retrieve envelope data and update rolling window ---
    envelope_arr = ul.get_envelope_array(ULTRASONIC_PATH)
    if envelope_arr is not None and envelope_arr.shape[0] > 0:
        # Use the first row (for a single emitter scenario)
        first_row = envelope_arr[0].copy()

        # Record the current time along with this envelope row
        current_time = time.time()
        envelope_buffer.append((current_time, first_row))
        
        # Remove rows older than the defined window duration
        while envelope_buffer and (current_time - envelope_buffer[0][0]) > window_duration:
            envelope_buffer.popleft()
        
        # Convert the deque to a 2D numpy array for plotting
        if envelope_buffer:
            timestamps, envelope_rows = zip(*envelope_buffer)
            data = np.array(envelope_rows)  # shape: (num_samples, num_bins)
            
            # Create an extent for the image: 
            # x-axis: bin indices (0 to number of bins)
            # y-axis: time relative to the oldest sample (0 to window_duration)
            extent = [0, data.shape[1], 0, window_duration]
            
            # Initialize the image if not done already, else update it
            if im2 is None:
                im2 = ax2.imshow(data, aspect='auto', origin='lower', extent=extent, cmap='viridis')
                fig2.colorbar(im2, ax=ax2)
            else:
                im2.set_data(data)
                im2.set_extent(extent)
            
            # Set the axes limits
            ax2.set_xlim(0, data.shape[1])
            ax2.set_ylim(0, window_duration)
            
            # Redraw the figure to update the plot
            plt.draw()
            plt.pause(0.001)

# 6) Subscribe to Isaac Sim's update event so on_update is called every frame
update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
