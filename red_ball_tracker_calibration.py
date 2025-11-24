"""
Red Ball Tracking System for PDE 4432 Robot Control and Sensing
This program uses computer vision to detect and track a red ball in real-time
using a webcam. It provides an advanced calibration interface for fine-tuning
the detection parameters and will eventually control servos to track the ball.
"""

import cv2
import numpy as np

def nothing(x):
    """
    Empty callback function required for OpenCV trackbars.
    Trackbars need a function to call when their values change, even if we don't
    need to do anything immediately when they change.
    """
    pass

def setup_trackbars():
    """
    Creates and configures all trackbars for the HSV calibration window.
    These trackbars allow real-time adjustment of color detection parameters.
    """
    # Create a named window for the trackbar interface
    cv2.namedWindow('HSV Calibration')
    
    # =========================================================================
    # FIRST RED RANGE TRACKBARS (0-10 degrees on HSV hue scale)
    # Red appears at both ends of the hue spectrum (0-10 and 170-180)
    # =========================================================================
    cv2.createTrackbar('H1 Min', 'HSV Calibration', 0, 179, nothing)    # Lower hue bound for first red range
    cv2.createTrackbar('H1 Max', 'HSV Calibration', 10, 179, nothing)   # Upper hue bound for first red range
    cv2.createTrackbar('S1 Min', 'HSV Calibration', 120, 255, nothing)  # Lower saturation bound (color purity)
    cv2.createTrackbar('S1 Max', 'HSV Calibration', 255, 255, nothing)  # Upper saturation bound
    cv2.createTrackbar('V1 Min', 'HSV Calibration', 70, 255, nothing)   # Lower value bound (brightness)
    cv2.createTrackbar('V1 Max', 'HSV Calibration', 255, 255, nothing)  # Upper value bound
    
    # =========================================================================
    # SECOND RED RANGE TRACKBARS (170-180 degrees on HSV hue scale)
    # =========================================================================
    cv2.createTrackbar('H2 Min', 'HSV Calibration', 170, 179, nothing)  # Lower hue bound for second red range
    cv2.createTrackbar('H2 Max', 'HSV Calibration', 180, 179, nothing)  # Upper hue bound for second red range
    cv2.createTrackbar('S2 Min', 'HSV Calibration', 120, 255, nothing)  # Lower saturation bound
    cv2.createTrackbar('S2 Max', 'HSV Calibration', 255, 255, nothing)  # Upper saturation bound
    cv2.createTrackbar('V2 Min', 'HSV Calibration', 70, 255, nothing)   # Lower value bound
    cv2.createTrackbar('V2 Max', 'HSV Calibration', 255, 255, nothing)  # Upper value bound
    
    # =========================================================================
    # FINE-TUNING ADJUSTMENT TRACKBARS
    # These provide additional precision control over the main ranges
    # =========================================================================
    cv2.createTrackbar('LH Adjust', 'HSV Calibration', 0, 50, nothing)   # Lower Hue adjustment
    cv2.createTrackbar('LS Adjust', 'HSV Calibration', 0, 50, nothing)   # Lower Saturation adjustment
    cv2.createTrackbar('LV Adjust', 'HSV Calibration', 0, 50, nothing)   # Lower Value adjustment
    cv2.createTrackbar('UH Adjust', 'HSV Calibration', 0, 50, nothing)   # Upper Hue adjustment
    cv2.createTrackbar('US Adjust', 'HSV Calibration', 0, 50, nothing)   # Upper Saturation adjustment
    cv2.createTrackbar('UV Adjust', 'HSV Calibration', 0, 50, nothing)   # Upper Value adjustment
    
    # =========================================================================
    # NOISE REDUCTION AND IMAGE PROCESSING TRACKBARS
    # These help filter out false positives and improve detection quality
    # =========================================================================
    cv2.createTrackbar('Min Radius', 'HSV Calibration', 10, 100, nothing)  # Minimum detected circle radius
    cv2.createTrackbar('Blur', 'HSV Calibration', 5, 20, nothing)          # Gaussian blur kernel size
    cv2.createTrackbar('Erode', 'HSV Calibration', 1, 10, nothing)         # Erosion iterations to remove small noise
    cv2.createTrackbar('Dilate', 'HSV Calibration', 1, 10, nothing)        # Dilation iterations to fill gaps

def get_trackbar_values():
    """
    Reads all current trackbar values and applies fine-tuning adjustments.
    
    Returns:
        tuple: Contains all calibrated values including HSV ranges and noise reduction parameters
    """
    # =========================================================================
    # READ FIRST RED RANGE VALUES
    # =========================================================================
    h1_min = cv2.getTrackbarPos('H1 Min', 'HSV Calibration')
    h1_max = cv2.getTrackbarPos('H1 Max', 'HSV Calibration')
    s1_min = cv2.getTrackbarPos('S1 Min', 'HSV Calibration')
    s1_max = cv2.getTrackbarPos('S1 Max', 'HSV Calibration')
    v1_min = cv2.getTrackbarPos('V1 Min', 'HSV Calibration')
    v1_max = cv2.getTrackbarPos('V1 Max', 'HSV Calibration')
    
    # =========================================================================
    # READ SECOND RED RANGE VALUES
    # =========================================================================
    h2_min = cv2.getTrackbarPos('H2 Min', 'HSV Calibration')
    h2_max = cv2.getTrackbarPos('H2 Max', 'HSV Calibration')
    s2_min = cv2.getTrackbarPos('S2 Min', 'HSV Calibration')
    s2_max = cv2.getTrackbarPos('S2 Max', 'HSV Calibration')
    v2_min = cv2.getTrackbarPos('V2 Min', 'HSV Calibration')
    v2_max = cv2.getTrackbarPos('V2 Max', 'HSV Calibration')
    
    # =========================================================================
    # READ FINE-TUNING ADJUSTMENT VALUES
    # =========================================================================
    lh_adj = cv2.getTrackbarPos('LH Adjust', 'HSV Calibration')  # Lower hue adjustment
    ls_adj = cv2.getTrackbarPos('LS Adjust', 'HSV Calibration')  # Lower saturation adjustment
    lv_adj = cv2.getTrackbarPos('LV Adjust', 'HSV Calibration')  # Lower value adjustment
    uh_adj = cv2.getTrackbarPos('UH Adjust', 'HSV Calibration')  # Upper hue adjustment
    us_adj = cv2.getTrackbarPos('US Adjust', 'HSV Calibration')  # Upper saturation adjustment
    uv_adj = cv2.getTrackbarPos('UV Adjust', 'HSV Calibration')  # Upper value adjustment
    
    # =========================================================================
    # APPLY FINE-TUNING ADJUSTMENTS TO MAIN RANGES
    # This allows for precise control beyond the main trackbar ranges
    # =========================================================================
    h1_min_adj = max(0, h1_min - lh_adj)      # Adjust lower hue bound downward
    h1_max_adj = min(179, h1_max + uh_adj)    # Adjust upper hue bound upward
    s1_min_adj = max(0, s1_min - ls_adj)      # Adjust lower saturation bound
    s1_max_adj = min(255, s1_max + us_adj)    # Adjust upper saturation bound
    v1_min_adj = max(0, v1_min - lv_adj)      # Adjust lower value bound
    v1_max_adj = min(255, v1_max + uv_adj)    # Adjust upper value bound
    
    h2_min_adj = max(0, h2_min - lh_adj)      # Apply same adjustments to second range
    h2_max_adj = min(179, h2_max + uh_adj)
    s2_min_adj = max(0, s2_min - ls_adj)
    s2_max_adj = min(255, s2_max + us_adj)
    v2_min_adj = max(0, v2_min - lv_adj)
    v2_max_adj = min(255, v2_max + uv_adj)
    
    # =========================================================================
    # READ NOISE REDUCTION PARAMETERS
    # =========================================================================
    min_radius = cv2.getTrackbarPos('Min Radius', 'HSV Calibration')  # Minimum object size to detect
    blur_value = cv2.getTrackbarPos('Blur', 'HSV Calibration')        # Blur intensity for noise reduction
    erode_value = cv2.getTrackbarPos('Erode', 'HSV Calibration')      # Erosion strength to remove small noise
    dilate_value = cv2.getTrackbarPos('Dilate', 'HSV Calibration')    # Dilation strength to fill gaps
    
    return (h1_min_adj, h1_max_adj, s1_min_adj, s1_max_adj, v1_min_adj, v1_max_adj,
            h2_min_adj, h2_max_adj, s2_min_adj, s2_max_adj, v2_min_adj, v2_max_adj,
            min_radius, blur_value, erode_value, dilate_value,
            lh_adj, ls_adj, lv_adj, uh_adj, us_adj, uv_adj)

def track_red_ball_with_calibration():
    """
    Main function that captures video, processes frames, and tracks red balls.
    Provides real-time calibration and visualization of the tracking process.
    """
    # Initialize video capture from default camera (camera index 0)
    cap = cv2.VideoCapture(0)
    
    # Set camera resolution to 640x480 for consistent processing
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Setup the calibration trackbars
    setup_trackbars()
    
    # =========================================================================
    # USER INSTRUCTIONS AND INTERFACE GUIDE
    # =========================================================================
    print("=== ADVANCED RED BALL TRACKING CALIBRATION MODE ===")
    print("MAIN CONTROLS:")
    print("H1/H2 (Hue): Red color range (0-10 and 170-180 for red)")
    print("S1/S2 (Saturation): Color purity - higher = more vivid colors")
    print("V1/V2 (Value): Brightness - adjust for lighting conditions")
    print("")
    print("FINE-TUNING CONTROLS:")
    print("LH/LS/LV Adjust: Fine-tune LOWER bounds with precision")
    print("UH/US/UV Adjust: Fine-tune UPPER bounds with precision")
    print("")
    print("NOISE REDUCTION:")
    print("Min Radius: Ignore small detections (pixels)")
    print("Blur: Smooths image to reduce noise (odd values only)")
    print("Erode: Removes small noise particles")
    print("Dilate: Fills gaps in detection areas")
    print("")
    print("KEYBOARD SHORTCUTS:")
    print("'r' - Reset all sliders to default values")
    print("'s' - Save and print current calibration settings")
    print("'q' - Quit program and close all windows")
    print("")
    print("CALIBRATION TIPS:")
    print("1. Adjust main HSV ranges first to capture the red ball")
    print("2. Use fine-tuning sliders for perfect edge detection")
    print("3. Monitor the MASK window - only the ball should appear white")
    print("4. Use noise reduction to eliminate false positives")
    print("5. Save your optimal settings for future use")
    
    # =========================================================================
    # MAIN PROCESSING LOOP
    # =========================================================================
    while True:
        # Capture frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame from camera")
            break
        
        # Mirror the frame horizontally for more intuitive interaction
        frame = cv2.flip(frame, 1)
        
        # Keep a copy of the original frame for display
        original_frame = frame.copy()
        
        # Get current values from all trackbars
        values = get_trackbar_values()
        (h1_min, h1_max, s1_min, s1_max, v1_min, v1_max,
         h2_min, h2_max, s2_min, s2_max, v2_min, v2_max,
         min_radius, blur_value, erode_value, dilate_value,
         lh_adj, ls_adj, lv_adj, uh_adj, us_adj, uv_adj) = values
        
        # =====================================================================
        # IMAGE PRE-PROCESSING: NOISE REDUCTION
        # =====================================================================
        # Apply Gaussian blur to reduce noise and smooth the image
        # blur_value*2+1 ensures the kernel size is always odd (required by GaussianBlur)
        if blur_value > 0:
            frame_blur = cv2.GaussianBlur(frame, (blur_value*2+1, blur_value*2+1), 0)
        else:
            frame_blur = frame
        
        # =====================================================================
        # COLOR SPACE CONVERSION: BGR to HSV
        # HSV (Hue, Saturation, Value) is better for color segmentation than RGB
        # =====================================================================
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        
        # =====================================================================
        # COLOR MASK CREATION
        # Create two masks for the two red ranges and combine them
        # =====================================================================
        # Define HSV range for first red segment (0-10 degrees)
        lower_red1 = np.array([h1_min, s1_min, v1_min])
        upper_red1 = np.array([h1_max, s1_max, v1_max])
        
        # Define HSV range for second red segment (170-180 degrees)  
        lower_red2 = np.array([h2_min, s2_min, v2_min])
        upper_red2 = np.array([h2_max, s2_max, v2_max])
        
        # Create masks for both red ranges and combine them
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)  # Mask for first red range
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)  # Mask for second red range
        mask = mask1 + mask2  # Combined mask (logical OR operation)
        
        # =====================================================================
        # MORPHOLOGICAL OPERATIONS: NOISE FILTERING
        # =====================================================================
        # Create a kernel for morphological operations
        kernel = np.ones((5, 5), np.uint8)
        
        # Erosion: Removes small white noise by eroding white regions
        if erode_value > 0:
            mask = cv2.erode(mask, kernel, iterations=erode_value)
        
        # Dilation: Fills holes and gaps in detected regions
        if dilate_value > 0:
            mask = cv2.dilate(mask, kernel, iterations=dilate_value)
        
        # =====================================================================
        # CONTOUR DETECTION AND ANALYSIS
        # =====================================================================
        # Find contours in the binary mask
        # RETR_EXTERNAL: Retrieves only the extreme outer contours
        # CHAIN_APPROX_SIMPLE: Compresses horizontal, vertical, and diagonal segments
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # =====================================================================
        # BALL DETECTION AND VISUALIZATION
        # =====================================================================
        balls_detected = 0
        for contour in contours:
            # Calculate area of the contour to filter out small noise
            area = cv2.contourArea(contour)
            if area > 100:  # Ignore contours smaller than 100 pixels
                # Find the minimum enclosing circle around the contour
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                
                # Only process if the circle is larger than minimum radius
                if radius > min_radius:
                    balls_detected += 1
                    
                    # Draw the detected circle around the ball
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)  # Yellow circle
                    
                    # Draw a small circle at the center of the ball
                    cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)  # Red center dot
                    
                    # Add label above the ball
                    cv2.putText(frame, f"Ball {balls_detected}", (int(x), int(y)-15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Display radius below the ball
                    cv2.putText(frame, f"R:{int(radius)}", (int(x), int(y)+15),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # =====================================================================
        # DISPLAY CURRENT CALIBRATION PARAMETERS ON FRAME
        # =====================================================================
        y_offset = 30
        line_height = 25
        
        # Display ball count
        cv2.putText(frame, f"Balls detected: {balls_detected}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        # Display first HSV range
        cv2.putText(frame, f"H1: {h1_min}-{h1_max}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += line_height
        
        cv2.putText(frame, f"S1: {s1_min}-{s1_max}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += line_height
        
        cv2.putText(frame, f"V1: {v1_min}-{v1_max}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += line_height
        
        # Display fine-tuning adjustments
        cv2.putText(frame, f"Fine-tune: LH:{lh_adj} LS:{ls_adj} LV:{lv_adj}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        y_offset += line_height
        
        cv2.putText(frame, f"Fine-tune: UH:{uh_adj} US:{us_adj} UV:{uv_adj}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # =====================================================================
        # DISPLAY ALL WINDOWS
        # =====================================================================
        cv2.imshow('Red Ball Tracker - Live View', frame)           # Main tracking view with annotations
        cv2.imshow('Mask - Adjust until ONLY ball is white', mask)  # Binary mask for calibration
        cv2.imshow('Original Camera', original_frame)               # Unprocessed camera feed
        
        # =====================================================================
        # KEYBOARD INPUT HANDLING
        # =====================================================================
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # Quit the program
            print("Exiting program...")
            break
        elif key == ord('r'):
            # Reset all trackbars to default values
            print("Resetting all trackbars to default values...")
            
            # Reset first red range
            cv2.setTrackbarPos('H1 Min', 'HSV Calibration', 0)
            cv2.setTrackbarPos('H1 Max', 'HSV Calibration', 10)
            cv2.setTrackbarPos('S1 Min', 'HSV Calibration', 120)
            cv2.setTrackbarPos('S1 Max', 'HSV Calibration', 255)
            cv2.setTrackbarPos('V1 Min', 'HSV Calibration', 70)
            cv2.setTrackbarPos('V1 Max', 'HSV Calibration', 255)
            
            # Reset second red range
            cv2.setTrackbarPos('H2 Min', 'HSV Calibration', 170)
            cv2.setTrackbarPos('H2 Max', 'HSV Calibration', 180)
            cv2.setTrackbarPos('S2 Min', 'HSV Calibration', 120)
            cv2.setTrackbarPos('S2 Max', 'HSV Calibration', 255)
            cv2.setTrackbarPos('V2 Min', 'HSV Calibration', 70)
            cv2.setTrackbarPos('V2 Max', 'HSV Calibration', 255)
            
            # Reset fine-tuning adjustments
            cv2.setTrackbarPos('LH Adjust', 'HSV Calibration', 0)
            cv2.setTrackbarPos('LS Adjust', 'HSV Calibration', 0)
            cv2.setTrackbarPos('LV Adjust', 'HSV Calibration', 0)
            cv2.setTrackbarPos('UH Adjust', 'HSV Calibration', 0)
            cv2.setTrackbarPos('US Adjust', 'HSV Calibration', 0)
            cv2.setTrackbarPos('UV Adjust', 'HSV Calibration', 0)
            
            # Reset noise reduction parameters
            cv2.setTrackbarPos('Min Radius', 'HSV Calibration', 10)
            cv2.setTrackbarPos('Blur', 'HSV Calibration', 5)
            cv2.setTrackbarPos('Erode', 'HSV Calibration', 1)
            cv2.setTrackbarPos('Dilate', 'HSV Calibration', 1)
            
        elif key == ord('s'):
            # Save and display current calibration settings
            print("\n" + "="*50)
            print("CURRENT CALIBRATION SETTINGS SAVED")
            print("="*50)
            print(f"First red range:  H[{h1_min}-{h1_max}] S[{s1_min}-{s1_max}] V[{v1_min}-{v1_max}]")
            print(f"Second red range: H[{h2_min}-{h2_max}] S[{s2_min}-{s2_max}] V[{v2_min}-{v2_max}]")
            print(f"Fine-tuning adjustments: LH:{lh_adj} LS:{ls_adj} LV:{lv_adj} UH:{uh_adj} US:{us_adj} UV:{uv_adj}")
            print(f"Noise reduction: Min Radius:{min_radius} Blur:{blur_value} Erode:{erode_value} Dilate:{dilate_value}")
            print("="*50)
            print("Copy these settings for use in your final tracking code!")
    
    # =========================================================================
    # CLEANUP: RELEASE RESOURCES
    # =========================================================================
    cap.release()  # Release the camera
    cv2.destroyAllWindows()  # Close all OpenCV windows

# Main entry point of the program
if __name__ == "__main__":
    track_red_ball_with_calibration()
