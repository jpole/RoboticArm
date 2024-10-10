import cv2
import numpy as np
import serial
import time

# Establish a serial connection with the Arduino on the COM4 port at a speed of 2000000 baud
arduino = serial.Serial("COM4", 2000000)
time.sleep(2)  # Initial 2-second wait to ensure the connection is established correctly

# Configure numpy's print options for better readability
np.set_printoptions(suppress=True, formatter={'float': '{: .2f}'.format})

# Robot arm lengths in centimeters
arm1 = 18
arm2 = 17

# Initial joint angles in degrees
theta1 = 0
theta2 = 0

# Convert angles from degrees to radians
q1 = np.deg2rad(theta1) 
q2 = np.deg2rad(theta2) 

current_config = (q1, q2)

# Initial position in x and y coordinates
posx = 0
posy = 0

# Start video capture from the default camera
cap = cv2.VideoCapture(0)
time.sleep(1)

# Yellow color range in the HSV color space
YellowUp = np.array([65, 255, 255], np.uint8)
YellowDown = np.array([25, 20, 20], np.uint8)

# Image dimensions in centimeters
cm_x = 23  # centimeters in x
cm_y = 17.5  # centimeters in y

# Initialize the previous valid position
prev_posx = posx
prev_posy = posy

paused = False  # Initial pause state

# Function to transform coordinates based on camera rotation angles
def transform_coordinates(x, y, q1, q2):
    # Rotation matrices for each axis
    rotation_matrix1 = np.array([
        [np.cos(q1), -np.sin(q1)],
        [np.sin(q1), np.cos(q1)]
    ])

    rotation_matrix2 = np.array([
        [np.cos(q2), -np.sin(q2)],
        [np.sin(q2), np.cos(q2)]
    ])

    # Combined rotation matrix
    combined_rotation_matrix = np.dot(rotation_matrix2, rotation_matrix1)

    # Original coordinates as a vector
    coords = np.array([x, y])

    # Transformed coordinates
    transformed_coords = np.dot(combined_rotation_matrix, coords)
    return transformed_coords[0], transformed_coords[1]

# Function to calculate motor angles based on x, y coordinates
def CalculateMotor(x, y):
    # Angle beta using atan2
    beta = np.arctan2(y, x)

    # Distance a using the Pythagorean theorem
    a = np.hypot(x, y)

    # Gamma angle using the law of cosines
    cos_gamma = (arm1**2 + a**2 - arm2**2) / (2 * a * arm1)
    gamma = np.arccos(np.clip(cos_gamma, -1, 1))

    # Angle c using the law of cosines
    cos_cc = (arm1**2 + arm2**2 - a**2) / (2 * arm1 * arm2)
    cc = np.arccos(np.clip(cos_cc, -1, 1))

    Motor1_conf1 = np.degrees(beta - gamma)
    Motor2_conf1 = np.degrees(np.pi - cc)
    Motor1_conf2 = np.degrees(beta + gamma)
    Motor2_conf2 = np.degrees(cc - np.pi)

    if a > arm1 + arm2:
        print('Out of Range')

    return (Motor1_conf1, Motor2_conf1), (Motor1_conf2, Motor2_conf2)

# Function to create a transformation matrix
def Transformation(q1, q2):
    T = [[np.cos(q1 + q2), -np.sin(q1 + q2), 0, arm2 * np.cos(q1 + q2) + arm1 * np.cos(q1)],
         [np.sin(q1 + q2), np.cos(q1 + q2), 0, arm2 * np.sin(q1 + q2) + arm1 * np.sin(q1)],
         [0, 0, 1, 0],
         [0, 0, 0, 1]]

    return np.array(T)

# Function to convert degrees to motor steps
def deg2step(q1, q2):
    Motor1 = int(np.rint(q1 / 1.8))
    Motor2 = int(np.rint(q2 / 1.8))

    return Motor1, Motor2

# Function to limit motor2 steps within the range of -83 to 83
def limit_motor2(Motor2_steps):
    return np.clip(Motor2_steps, -83, 83)

# Function to calculate the angular difference between two configurations
def angular_difference(current_config, new_config):
    delta_theta1 = new_config[0] - current_config[0]
    delta_theta2 = new_config[1] - current_config[1]
    return np.sqrt(delta_theta1**2 + delta_theta2**2)

# Function to smoothly move to the new configuration
def smoothmove_config(x, y, current_config):
    config1, config2 = CalculateMotor(x, y)
    
    delta1 = angular_difference(current_config, config1)
    delta2 = angular_difference(current_config, config2)
    
    if delta1 < delta2:
        print('config 1')
        return config1
    else:
        print('config 2')
        return config2

# Function to send data to the Arduino
def SendData(Motor_1, Motor_2):
    message = f"{Motor_1},{Motor_2}"
    arduino.write(message.encode())  # Send the message as a byte-encoded string
    arduino.flush()
    print(f"Sent message: {message}")
    time.sleep(0.25)

# Main 
while True:
    ret, frame = cap.read()  # Read a frame from the camera
    if ret:
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2

        # Convert the frame to HSV
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Apply Gaussian blur
        frame_blur = cv2.GaussianBlur(frame_hsv, (11, 11), 0)

        # Create a Yellow color mask
        mask = cv2.inRange(frame_blur, YellowDown, YellowUp)

        # Erosion and dilation
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), 0, 30, 2)

        if contours:
            # Find the contour with the largest area
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            if area < 20000 and area > 3000:
                M = cv2.moments(max_contour)
                if M["m00"] == 0:
                    M["m00"] = 1

                x = int(M["m10"] / M["m00"]) - center_x
                y = -(int(M["m01"] / M["m00"]) - center_y)

                # Calculate the coordinates in centimeters
                pixels_per_cm_x = width / cm_x
                pixels_per_cm_y = height / cm_y

                x_cm = x / pixels_per_cm_x
                y_cm = y / pixels_per_cm_y

                cv2.circle(frame, (x + center_x, -y + center_y), 7, (0, 0, 255), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX

                # Adjust text position for visibility
                text = f'{x_cm:.2f} cm, {y_cm:.2f} cm'
                text_size, _ = cv2.getTextSize(text, font, 1, 2)

                text_pos = (max(10, min(x + center_x - text_size[0] // 2, width - text_size[0])), max(30, center_y - y - 20))

                cv2.putText(frame, text, text_pos, font, 1, (0, 0, 0), 2, cv2.LINE_AA)

                # Draw the object's contour
                newContour = cv2.convexHull(max_contour)
                cv2.drawContours(frame, [newContour], 0, (255, 0, 0), 3)

                posx = x_cm
                posy = y_cm

                # Change the axis according to the camera
                x = posy
                y = -posx

                T = Transformation(q1, q2)
                coord = transform_coordinates(x, y, q1, q2)

                print('Transformation Matrix T:\n', T)
                print(f'transformed x= {coord[0]:.2f}', f' transformed y= {coord[1]:.2f}')

                actual_point = np.array([T[0, 3], T[1, 3]])
                new_point = actual_point + coord

                print(f'actual x= {actual_point[0]:.2f}', f'actual y= {actual_point[1]:.2f}')
                print(f'new x= {new_point[0]:.2f}', f'new y= {new_point[1]:.2f}')

                # Check if position has changed
                if not paused:
                    current_config = smoothmove_config(new_point[0], new_point[1], current_config)

                    Motor1_steps, Motor2_steps = deg2step(current_config[0], current_config[1])
                    Motor2_steps = limit_motor2(Motor2_steps)

                    # Send data to the Arduino
                    SendData(Motor1_steps, Motor2_steps)

                prev_posx = posx
                prev_posy = posy

    cv2.imshow("Object Detection", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
