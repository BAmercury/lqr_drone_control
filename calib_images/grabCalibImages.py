#!/usr/bin/env python

import cv2

def main(webcam=0, number_of_images=30, write_images = False, stream = False):
    cam = cv2.VideoCapture(webcam)
    image_id = 1
    points = []
    if (stream == False):
        for i in range(number_of_images):

            # Read in image from webcam and save as png
            ret_val, img = cam.read()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if (write_images == True):
                cv2.imwrite((str(image_id)+".png"), img)
                image_id = image_id + 1
            cv2.imshow('Output', img)
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
    else:
        print("Streaming")
        while True:

            # Read in image from webcam and save as png
            ret_val, img = cam.read()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Output Image
            cv2.imshow('Output', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    main(webcam=1,write_images=True,stream=False)

