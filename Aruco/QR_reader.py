import cv2
print(cv2.__version__)
# Basic QR code reader with video capture in OpenCV -> https://note.nkmk.me/en/python-opencv-qrcode/
# QR Code Detectors in OpenCV documentation -> https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html
# Free Web QR code generator -> https://www.qr-code-generator.com/
# Use Text format: item:___;x:___;y:___;z:___

# video constants
cam = 0
delay = 1
window_name = 'OpenCV QR code reader'

# CV2 objects
qcd = cv2.QRCodeDetector()
cap = cv2.VideoCapture(cam)

# video capture loop until 'q' pressed
while True:
    # read frame from camera 0
    ret,frame = cap.read()
    # if receiving frame
    if ret:
        # detecting and decoding multiple QR codes simultaneously
        ret_qr,decoded_info,points,_ = qcd.detectAndDecodeMulti(frame)
        # if any QR code detected
        if ret_qr:
            # for each QR code detected
            for s,p in zip(decoded_info,points):
                # if there is any info decoded in the code
                if s:
                    print(s)    # show
                    color = (0,255,0)   # green
                else:
                    color = (0,0,255)   # red
                # draw surrounding the QRs detected
                frame = cv2.polylines(frame,[p.astype(int)],True,color,8)
        # show
        cv2.imshow(window_name,frame)

    # if 'q' -> exit video capture
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break
# destroy window to end program
cap.release()
cv2.destroyWindow(window_name)