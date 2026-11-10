# Obstacle Detection - By: Rebekka Alve - Tue Nov 4 2025

import sensor, time, struct
from pyb import UART, LED
from machine import I2C
from vl53l1x import VL53L1X

# Enable UART4, Tx = pin1 (SDA i2c), Rx = pin2 (SCL i2c)
uart = UART(4, 115200, timeout_char=200)

# Parameters BLOB DETECTION
OFFSET = 30  # threshold offset around mean background brightness
min_area = 300       # ignore tiny blobs
min_pixels = 300     # ignore tiny blobs
max_fraction = 0.95   # ignore blobs covering more than 90% of image (not working?)

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # grayscale for detection
sensor.set_framesize(sensor.QVGA) # smaller frame size for speed
sensor.skip_frames(time=500)

# Camera specifications and focal length
pixel_size_mm = 1.75e-3
sensor_px_width = 1616      # active pixel array width
image_px_width  = 320       # QVGA capture width
f_mm = 2.2                  # focal length in mm
f_tuned_mm = 100.0            # tuned focal length, increase value to decrease object width estimates

sensor_width_mm = pixel_size_mm * sensor_px_width
focal_length_px = f_tuned_mm + int((f_mm * image_px_width) / sensor_width_mm) # approx 248 + tuned value

# Distance sensor setup (ToF = time of flight)
i2c = I2C(2)
tof = VL53L1X(i2c)
MIN_VALID_DISTANCE = 40  # mm
MAX_VALID_DISTANCE = 2000 # mm

# Buffer (last 10 readings) for smoothing noisy detections
readings = []
MAX_READINGS = 10

# LEDs
red = LED(1)
green = LED(2) # streaming video
blue = LED(3)  # setting up stream

def leds_off():
    red.off()
    green.off()
    blue.off()

clock = time.clock()

def uart_request():
    if uart.any():
        request = uart.read(1)
        print('Request:', request)
        if request == b'r':
            return True
    return False

def average_offset_width_distance_mm(target,center_x,dist):

    # Get x-range of the blob, NB: pixels 0-320
    x_min = target.x
    x_max = target.x + target.w

    # Store in buffer
    readings.append((x_min,x_max,dist))

    # Find average of the last N readings
    if len(readings) == MAX_READINGS:
        x_min = sum(r[0] for r in readings) // MAX_READINGS
        x_max = sum(r[1] for r in readings) // MAX_READINGS
        dist = sum(r[2] for r in readings) // MAX_READINGS
        readings.pop(0)  # remove oldest reading

    # Convert from pixel to mm
    width_px = x_max - x_min
    object_width_mm = (width_px * dist) / focal_length_px

    # offset from image center to x_start in mm (negative = left)
    x_offset_mm = ((x_min - center_x) * dist) / focal_length_px

    return int(x_offset_mm),int(object_width_mm),int(dist)


def detect_obstacles():
    clock.tick()
    img = sensor.snapshot()

    ## DISTANCE SENSOR
    dist = tof.read()  # Read distance in mm

    # Dynamic background brightness
    stats = img.get_statistics()
    mean_val = stats.l_mean
    limit = max(0, mean_val - OFFSET)
    color_thresholds = [(0, limit)]

    # Find dark blobs
    blobs = img.find_blobs(color_thresholds, pixels_threshold=min_pixels, area_threshold=min_area)

    if blobs and dist > MIN_VALID_DISTANCE and dist < MAX_VALID_DISTANCE:
        # Center of the image (where ToF points)
        center_x = img.width() // 2
        center_y = img.height() // 2

        # Filter blobs to ignore very large ones
        max_pixels = int(img.width() * img.height() * max_fraction)
        valid_blobs = [b for b in blobs if b.pixels < max_pixels]

        # Find the blob covering the distance sensor's line-of-sight (center)
        target = None
        for b in valid_blobs:
            if b.pixels < (img.width() * img.height() * max_fraction):
                if b.x <= center_x <= b.x+b.w and b.y <= center_y <= b.y+b.h:
                    target = b
                    break
        if not target:
            # If no blob covers the center, find the nearest blob
            nearest = None
            nearest_dist = float('inf')
            for b in blobs:
                # Calculate distance from blob center to image center
                dx = b.cx - center_x
                dy = b.cy - center_y
                distance = (dx*dx + dy*dy)**0.5
                if distance < nearest_dist:
                    nearest = b
                    nearest_dist = distance
            target = nearest

        if target:
            # Draw target square and distance
            img.draw_cross((target.cx, target.cy), color=(0, 255, 0))
            img.draw_rectangle(target.rect, color=(0, 0, 0), thickness=3)
            img.draw_string((10, 10), f"Distance: {dist} mm", color=(255, 255, 255),scale=1.5)

            # Calculate average object offset, width and distance
            return average_offset_width_distance_mm(target,center_x,dist)
    return 0,0,0


camera_on = True # Set to False to disable camera processing
sequence = 0
try:
    green.on()
    print("focal length px:", focal_length_px)
    print("x offset mm, width mm, distance mm")
    while True:
        if camera_on:
            x_offset, width, distance = detect_obstacles()
        else:
            x_offset, width, distance = 50, 60, 70

        data = struct.pack(
            '<Hhhh',
            sequence,
            x_offset,
            width,
            distance
        ) # 2 bytes each

        if(uart_request()):
            uart.write(data)
            #print('Data sent')
            print(
                "SEQ:", sequence,
                "X:", x_offset,
                "W:", width,
                "D:", distance
            ) # print values, for testing

            sequence = (sequence + 1) % 65536

        time.sleep_ms(100)

except Exception as e:
    leds_off()
    print("\nProgram finished\n")
    print("Error:", e)
