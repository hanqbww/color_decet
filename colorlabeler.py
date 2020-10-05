# import the necessary packages
from scipy.spatial import distance as dist
from collections import OrderedDict
import numpy as np
import cv2

kernel_5 = np.ones((5,5),np.uint8)
kernel_2 = np.ones((2,2),np.uint8)
kernel_50 = np.ones((1,1),np.uint8)
class ColorLabeler:
	def __init__(self):
		# initialize the colors dictionary, containing the color
		# name as the key and the RGB tuple as the value
		colors = OrderedDict({
			"red": (255, 0, 0),
			"green": (0, 255, 0),
			"blue": (0, 0, 255)})

		# allocate memory for the L*a*b* image, then initialize
		# the color names list

		self.lab = np.zeros((len(colors), 1, 3), dtype="uint8")
		self.colorNames = []
		# loop over the colors dictionary
		for (i, (name, rgb)) in enumerate(colors.items()):
			# update the L*a*b* array and the color names list
			self.lab[i] = rgb
			self.colorNames.append(name)
			# print ("this is ",colors)
		# convert the L*a*b* array from the RGB color space
		# to L*a*b*
		self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)


	# #---------------------------------------------------------------------#
	# def label(self, image, c):
	# 	# construct a mask for the contour, then compute the
	# 	# average L*a*b* value for the masked region
	# 	mask = np.zeros(image.shape[:2], dtype="uint8")
	# 	cv2.drawContours(mask, [c], -1, 255, -1)
	# #---------------------------------------------------------------------#


	def label(self, image, cnts, max_idx):
		# construct a mask for the contour, then compute the
		# average L*a*b* value for the masked region
		mask = np.zeros(image.shape[:2], dtype="uint8")
		# cv2.drawContours(mask, [c], -1, 255, -1)
		# for i in range(max_idx - 1):
		# 	cv2.fillConvexPoly(mask, cnts[max_idx - 1], 0)
		# cv2.fillConvexPoly(mask, cnts[max_idx], 255)

		mask = cv2.drawContours(mask, cnts, max_idx, 255, cv2.FILLED)
		mask = cv2.erode(mask, kernel_2, iterations=1)
		cv2.imshow("ROS_mask",mask)
		cv2.waitKey(1)

		mean = cv2.mean(image, mask=mask)[:3]
		print("THis is mean = %",mean)

		# initialize the minimum distance found thus far
		minDist = (np.inf, None)
		# loop over the known L*a*b* color values
		for (i, row) in enumerate(self.lab):
			print i,row
			# compute the distance between the current L*a*b*
			# color value and the mean of the image
			d = dist.euclidean(row[0], mean)
			# print "row =", row
			# print "d = ",d
			# print "minDist[0]=", minDist[0]
			# if the distance is smaller than the current distance,
			# then update the bookkeeping variable
			if d < minDist[0]:
				minDist = (d, i)
			# print "(d,i)=", d,i
			# print "minDIst = " ,minDist

		# return the name of the color with the smallest distance
		return self.colorNames[minDist[1]]