import ImageTransferService
import numpy as np
import cv2

if __name__ == "__main__":

    host = '127.0.0.1'
    pw = input('Redis password: ')
    src = ImageTransferService.ImageTransferService(host, pw)

    print(src.ping())

    while True:
        im = src.receiveImage()
        cv2.imshow('Image',im)
        cv2.waitKey(1)