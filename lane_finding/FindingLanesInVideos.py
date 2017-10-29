'''
Udacity Self-Driving Car - Assignment 1: Finding Lines
Created on 10 Sep 2017, code base: https://github.com/udacity/CarND-LaneLines-P1 
@author: rob
'''

#importing some useful packages
import numpy as np
import cv2
from moviepy.editor import VideoFileClip
from IPython.display import HTML
#%matplotlib inline


######### HELPER STUFF ##########

def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, calibrationData, color=[255, 0, 0], thickness=5):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    leftLaneX=np.array([]);
    leftLaneY=np.array([]);
    rightLaneX=np.array([]);
    rightLaneY=np.array([]);
    
    
    
    for line in lines:
        for x1,y1,x2,y2 in line:
            myM=((y2-y1)/(x2-x1))
            if(myM<0):
                leftLaneX=np.hstack([leftLaneX, x1,x2])
                leftLaneY=np.hstack([leftLaneY, y1,y2])
            else:
                rightLaneX=np.hstack([rightLaneX, x1,x2])
                rightLaneY=np.hstack([rightLaneY, y1,y2])
            
    fittedLeft=np.polyfit(leftLaneX, leftLaneY, 1)
    fitPolyL=np.poly1d(fittedLeft)
    
    y1=fitPolyL(calibrationData[0][0])
    y2=fitPolyL(calibrationData[1][0])
    
    cv2.line(img, (calibrationData[0][0], int(y1)), (calibrationData[1][0], int(y2)), color, thickness)
    
    fittedRight=np.polyfit(rightLaneX, rightLaneY, 1)    
    
    fitPolyR=np.poly1d(fittedRight)
    
    y1=fitPolyR(calibrationData[3][0])
    y2=fitPolyR(calibrationData[2][0])
    
    cv2.line(img, (calibrationData[3][0], int(y1)), (calibrationData[2][0], int(y2)), color, thickness)
    
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap, calibrationData):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines, calibrationData)
    return line_img

# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    
    `initial_img` should be the image before any processing.
    
    The result image is computed as follows:
    
    initial_img * α + img * β + λ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, λ)

########### HELPER STUFF END ###########



def findLanes(myImage):    
    #printing out some stats and plotting
    #print('This image is:', type(myImage), 'with dimensions:', myImage.shape)
    
    #convert to grayscale
    gray=grayscale(myImage);
    
    #smooth using gaussian blur
    kernel_size = 7
    blur_gray =  gaussian_blur(gray, kernel_size)
    #cv2.medianBlur(gray,3)
    
    #run Canny 
    low_threshold = 120
    high_threshold = 150
    edges = canny(blur_gray, low_threshold, high_threshold)
    
    imshape = myImage.shape
    #calibration
    lowerLeftVertex=(120,imshape[0])
    upperLeftVertex=(455, 310)
    upperRightVertex=(510, 300)
    lowerRightVertex=(850,imshape[0])
    calibrationData=(lowerLeftVertex, upperLeftVertex,upperRightVertex,lowerRightVertex)
    
    vertices = np.array([[calibrationData[0][:], calibrationData[1][:], calibrationData[2][:], calibrationData[3][:]]], dtype=np.int32)
    masked_edges = region_of_interest(edges, vertices)
    
    # Define the Hough transform parameters
    # Make a blank the same size as our image to draw on
    rho = 3 # distance resolution in pixels of the Hough grid
    theta = (np.pi/180) # angular resolution in radians of the Hough grid
    threshold = 50     # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 20 #minimum number of pixels making up a line
    max_line_gap = 20 # maximum gap in pixels between connectable line segments
    
    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = hough_lines(masked_edges, rho, theta, threshold, min_line_length, max_line_gap, calibrationData) #,lowerLeftVertex, upperLeftVertex, upperRightVertex, lowerRightVertex)
    
    return weighted_img(lines, myImage)
    
if __name__ == '__main__':

    
    yellow_output = 'test_videos_output/solidYellowLeftWithLanes.mp4'
    #yellow_output = 'test_videos_output/solidWhiteRightWithLanes.mp4'
    ## To speed up the testing process you may want to try your pipeline on a shorter subclip of the video
    ## To do so add .subclip(start_second,end_second) to the end of the line below
    ## Where start_second and end_second are integer values representing the start and end of the subclip
    ## You may also uncomment the following line for a subclip of the first 5 seconds
    clip2 = VideoFileClip('test_videos/solidYellowLeft.mp4')#.subclip(12,17)
    #clip2 = VideoFileClip('test_videos/solidWhiteRight.mp4')#.subclip(12,17)    
    yellow_clip = clip2.fl_image(findLanes)
    yellow_clip.write_videofile(yellow_output, audio=False)
    
    HTML("""
    <video width="960" height="540" controls>
      <source src="{0}">
    </video>
    """.format(yellow_output))

    
    
