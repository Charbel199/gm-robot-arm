from skimage.metrics import structural_similarity as compare_ssim
import cv2
import imutils
import numpy as np
import math 

def draw_contour_bounding_box(image, contours, color =(0, 0, 255), thickness =2  ):
    for c in contours:
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(image, (x, y), (x + w, y + h), color,thickness)


def get_images_difference(image1, image2):
    (score, diff) = compare_ssim(image1, image2, full=True)
    diff = (diff * 255).astype("uint8")
    return score, diff


def get_contours(image):
    cnts = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return cnts

def concat_images(images, titles, with_titles = True):


    font = cv2.FONT_HERSHEY_SIMPLEX
    position= (15,35)
    fontScale = 1
    fontColor= (0,255,0)
    thickness = 2
    lineType= 1
    for i,image in enumerate(images):
        # Make all images 3 channels
        images[i] = np.stack((image,)*3, axis=-1) if len(image.shape)< 3 else image
        image = images[i]
        if with_titles:
            print("Image size ",image.shape," image[i] ",images[i].shape)
            cv2.putText(image,titles[i], 
                position, 
                font, 
                fontScale,
                fontColor,
                thickness,
                lineType)

    num_images = len(images)
    s = int(math.sqrt(num_images))

    # Append empty images
    empty_images = num_images%s
    [images.append(np.zeros_like(images[0])) for _ in range(empty_images)]
     


    def join_images(ims, horizontal = 1):
        return np.concatenate(tuple(ims), axis = 1 if horizontal else 0)
    
    split_arrays = np.array_split(images, s)
    vert_arrays = []
    for arr in split_arrays:
        vert_arrays.append(join_images(arr,horizontal =1) )

    final_image = join_images(vert_arrays, horizontal= 0)

    return final_image