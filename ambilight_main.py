import time
import mss
import cv2
import numpy as np
import socket
import asyncio

# ESP8266 IP address and port
esp8266_ip = '192.168.1.15'  # Replace with your ESP8266 IP address
esp8266_port = 4210

async def send_data(sock, data):
    # Convert the numpy array to a comma-separated string
    data_str = ','.join(map(str, data.flatten())) + '\n'
    
    # Send the data to the ESP8266
    sock.sendto(data_str.encode(), (esp8266_ip, esp8266_port))
    
    # Wait for 100 ms
    # await asyncio.sleep(0.1)

def get_pixel_as_image(pixels, horizontal, box_size=10):
    # Repeat the pixels vertically and horizontally to create boxes of box_size x box_size
    if horizontal:
        image = np.tile(pixels, (box_size, 1, 1))
        image = np.repeat(image, box_size, axis=1)
    else:
        image = pixels.reshape([pixels.shape[0], 1, pixels.shape[1]])
        image = np.tile(image, (1, box_size*25, 1))
        image = np.repeat(image, box_size, axis=0)
        
    return image

def get_horizontal_pixels(frame, edge, output_pixels_len=60, offset=20, convolution_len=1):
    if edge == 'T':
        strip = frame[offset:offset+convolution_len, :]
    else:
        strip = frame[-(offset+convolution_len):-offset, :]
    
    step_size = strip.shape[1] / output_pixels_len
    
    # Use numpy to pick pixels at calculated intervals
    indices = (np.arange(output_pixels_len) * step_size).astype(int)
    output_pixels = strip[0, indices]

    return output_pixels

def get_vertical_pixels(frame, edge, output_pixels_len=60, offset=20, convolution_width=1):
    if edge == 'L':
        strip = frame[:, offset:offset+convolution_width]
    else:
        strip = frame[:, -(offset+convolution_width):-offset]

    step_size = strip.shape[0] / output_pixels_len
    
    # Use numpy to pick pixels at calculated intervals
    indices = (np.arange(output_pixels_len) * step_size).astype(int)
    output_pixels = strip[indices, 0]

    return output_pixels

def get_filtered_pixels(pixels, previous_brightness=None):
    """
    Filters the pixels based on the brightness and saturation values.
    
    Parameters:
    pixels (numpy array): 3D array of HSV pixel values with shape (x, y, 3).
    
    Returns:
    numpy array: Filtered 3D array of HSV pixel values.
    """
    
    BRIGHTNESS_THRESHOLD_PERCENT = 66           # increase for more overall brightness, less flicker
    BRIGHTNESS_NORMAL_PIXEL_MULTIPLIER = 0.1    # increase for more overall brightness, less flicker
    BRIGHTNESS_HIGHLIGHT_PIXEL_MULTIPLIER = 0.2 # increase for more flashes, more flicker

    SATURATION_THRESHOLD_PERCENT = 50           # increase for less noise
    SATURATION_NORMAL_PIXEL_MULTIPLIER = 0.5    # increase for overall colors, more distractions  
    SATURATION_HIGHLIGHT_PIXEL_MULTIPLIER = 3   # increase for more imersion, more distractions

    ALPHA = 0.9 # less flicker more delay | 0 - 1 | more flicker less delay

    brightness_threshold = int(255 * BRIGHTNESS_THRESHOLD_PERCENT / 100)
    saturation_threshold = int(255 * SATURATION_THRESHOLD_PERCENT / 100)
    
    # Create masks for pixels with brightness and saturation greater than the thresholds
    brightness_mask = pixels[:, :, 2] > brightness_threshold
    saturation_mask = pixels[:, :, 1] > saturation_threshold
    
    # Create a copy of the original pixels array to avoid modifying the input array
    filtered_pixels = np.copy(pixels)
    
    # Multiply pixel brightness based on the brightness mask
    filtered_pixels[:, :, 2] = np.where(
        brightness_mask,
        filtered_pixels[:, :, 2] * BRIGHTNESS_HIGHLIGHT_PIXEL_MULTIPLIER,
        filtered_pixels[:, :, 2] * BRIGHTNESS_NORMAL_PIXEL_MULTIPLIER
    )
    
    # Multiply pixel saturation based on the saturation mask
    filtered_pixels[:, :, 1] = np.where(
        saturation_mask,
        filtered_pixels[:, :, 1] * SATURATION_HIGHLIGHT_PIXEL_MULTIPLIER,
        filtered_pixels[:, :, 1] * SATURATION_NORMAL_PIXEL_MULTIPLIER
    )
    
    # Ensure the brightness and saturation values are within the valid range [0, 255]
    filtered_pixels[:, :, 2] = np.clip(filtered_pixels[:, :, 2], 0, 255)
    filtered_pixels[:, :, 1] = np.clip(filtered_pixels[:, :, 1], 0, 255)
    
    # Apply exponential smoothing to the brightness values to reduce flickering
    if previous_brightness is not None:
        filtered_pixels[:, :, 2] = ALPHA * filtered_pixels[:, :, 2] + (1 - ALPHA) * previous_brightness
    
    # Return the filtered pixels and the current brightness values for the next frame
    return filtered_pixels, filtered_pixels[:, :, 2]


def hsv_filter(pixels, prev_brightness):
    reshaped_pixels = pixels.reshape([1, pixels.shape[0], pixels.shape[1]])
    hsv_pixels = cv2.cvtColor(reshaped_pixels, cv2.COLOR_BGR2HSV)

    hsv_pixels, cur_brightness = get_filtered_pixels(hsv_pixels, prev_brightness)

    recoded_pixels = cv2.cvtColor(hsv_pixels, cv2.COLOR_HSV2BGR)
    output_pixels = recoded_pixels.reshape([-1, recoded_pixels.shape[2]])
    
    return output_pixels, cur_brightness

def capture_screen(sock):
    try:
        top_prev_brightness = None
        bottom_prev_brightness = None
        left_prev_brightness = None
        right_prev_brightness = None

        with mss.mss() as sct:
            # Get the monitor dimensions
            monitor = sct.monitors[2]
            while True:
                # Capture the screen
                img = sct.grab(monitor)
                frame = np.array(img)

                # Convert the frame to BGR (OpenCV format)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Fetch a strip of height 200 pixels from the top of the frame
                
                convolved_pixels_top = get_horizontal_pixels(frame, 'T', 57)
                convolved_pixels_bottom = get_horizontal_pixels(frame, 'B', 60)
                
                convolved_pixels_left = get_vertical_pixels(frame, 'L', 33)
                convolved_pixels_right = get_vertical_pixels(frame, 'R', 33)

                convolved_pixels_top_recoded, top_cur_brightness = hsv_filter(convolved_pixels_top, top_prev_brightness)
                convolved_pixels_bottom_recoded, bottom_cur_brightness = hsv_filter(convolved_pixels_bottom, bottom_prev_brightness)
                convolved_pixels_left_recoded, left_cur_brightness = hsv_filter(convolved_pixels_left, left_prev_brightness)
                convolved_pixels_right_recoded, right_cur_brightness = hsv_filter(convolved_pixels_right, right_prev_brightness)

                top_prev_brightness = top_cur_brightness 
                bottom_prev_brightness = bottom_cur_brightness 
                left_prev_brightness = left_cur_brightness 
                right_prev_brightness = right_cur_brightness 

                # # Reverse the bottom and left arrays
                convolved_pixels_bottom_recoded = np.flip(convolved_pixels_bottom_recoded, axis=0)
                convolved_pixels_left_recoded = np.flip(convolved_pixels_left_recoded, axis=0)

                # Concatenate the arrays in the specified order
                concatenated_array = np.concatenate([
                    convolved_pixels_left_recoded,
                    convolved_pixels_top_recoded,
                    convolved_pixels_right_recoded,
                    convolved_pixels_bottom_recoded
                ])

                # Display the convolved pixels
                # cv2.imshow('Convolved Pixels top ORIG', get_pixel_as_image(convolved_pixels_top, horizontal=True))
                # cv2.imshow('Convolved Pixels Bottom ORIG', get_pixel_as_image(convolved_pixels_bottom, horizontal=True))
                # cv2.imshow('Convolved Pixels Left ORIG', get_pixel_as_image(convolved_pixels_left, horizontal=False))
                # cv2.imshow('Convolved Pixels Right ORIG', get_pixel_as_image(convolved_pixels_right, horizontal=False))

                # cv2.imshow('Convolved Pixels top RECODED', get_pixel_as_image(convolved_pixels_top_recoded, horizontal=True))
                # cv2.imshow('Convolved Pixels Bottom RECODED', get_pixel_as_image(convolved_pixels_bottom_recoded, horizontal=True))
                # cv2.imshow('Convolved Pixels Left RECODED', get_pixel_as_image(convolved_pixels_left_recoded, horizontal=False))
                # cv2.imshow('Convolved Pixels Right RECODED', get_pixel_as_image(convolved_pixels_right_recoded, horizontal=False))

                asyncio.run(send_data(sock, concatenated_array))

                # time.sleep(0.2)
                # Break the loop on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except Exception as e:
        print(e)
    finally:
        cv2.destroyAllWindows()
        print("destroyed")

if __name__ == "__main__":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    capture_screen(sock)