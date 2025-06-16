import cv2
import time

def main():
    # Initialize the camera
    camera = cv2.VideoCapture('/dev/video0')
    
    # Set camera properties
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 30)
    
    # Check if camera opened successfully
    if not camera.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Camera opened successfully")
    print("Press 'q' to quit")
    
    try:
        while True:
            # Capture frame-by-frame
            ret, frame = camera.read()
            
            if not ret:
                print("Error: Failed to capture frame")
                break
            
            # Display the frame
            cv2.imshow('Camera Stream', frame)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        # Release the camera and close windows
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
