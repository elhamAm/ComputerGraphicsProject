#%%
import cv2
import numpy as np
from  matplotlib import pyplot as plt
import math
import time
#%%
# Gamma Correction
def GammaCorrection(img):
    result = img.copy()
    case1 = result<=0.0031308
    result[case1] *= 12.92
    result[~case1] = 1.055 * result[~case1]**(1/2.4) - 0.055

    return result

def saveImage(filename, img):
    # img_to_save = GammaCorrection(img)
    cv2.imwrite(filename, img.clip(0,1)*255)

def showImage(img):
    im2 = img[:,:,::-1] 	# transform image to rgb
    plt.imshow(im2)
    plt.show()
# %%
color = cv2.imread("output.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
color_variance = cv2.imread("output_variance.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  

normal1 = cv2.imread("normal1.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
texture1 = cv2.imread("texture1.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
depth1 = cv2.imread("depth1.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
visibility1 = cv2.imread("visibility1.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
normal2 = cv2.imread("normal2.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
texture2 = cv2.imread("texture2.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
depth2 = cv2.imread("depth2.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
visibility2 = cv2.imread("visibility2.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
normal_variance = cv2.imread("normal_variance.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
texture_variance = cv2.imread("texture_variance.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
depth_variance = cv2.imread("depth_variance.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  
visibility_variance = cv2.imread("visibility_variance.exr",  cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)  

img_to_save = GammaCorrection(color)
saveImage("original.jpg", img_to_save)


depth_max = max(depth1.max(), depth2.max())
depth1 /= depth_max
depth2 /= depth_max 
depth_variance /= depth_max * depth_max


#%%
def toExtdIdx(i, j, border_size):
    return i+border_size, j+border_size

def shiftImage(src, dx, dy, original_shape, r):
    return src[r+dx : r + original_shape[0]+dx, r+dy : r+original_shape[1]+dy]

def d2_color(u_p, u_q, var_p, var_q, kc):
    epsilon = 1e-10
    var_pq = np.minimum(var_p, var_q)
    d2pixel = ((u_p - u_q)**2 - (var_p + var_pq)) / (epsilon + kc*kc*(var_p+var_q))
    
    return d2pixel.mean(2)

def d2_feature(f_p, f_q, var_p, var_q, sqrgrad_p, tau, kf):

    d2 = ((f_p - f_q) ** 2 - (var_p + np.minimum(var_p, var_q)))/ (kf**2 * np.maximum(tau, np.maximum(var_p, sqrgrad_p)))
    return d2.mean(2)
# %%
def nl_means_denoising(colors, color_variance, features=None, feature_variances=None, r=10, f=3, kc=0.45, kf = 1e9, tau=1e-3):
    
    use_half = not isinstance(colors, np.ndarray)
    if use_half:
        color1, color2 = colors
        color = (color1 + color2) / 2
    else:
        color = color1 = color2 = colors
    original_shape = color.shape
    
    use_feature = features is not None
    if use_feature:
        features_extended = []
        feature_variances_extended = []
        feature_sqrgrads = []
        for f_idx in range(len(features)):

            feature = features[f_idx]
            feature_variance = feature_variances[f_idx]
            
            feature_extended = cv2.copyMakeBorder(feature, r, r, r, r, cv2.BORDER_REPLICATE)        
            features_extended.append(feature_extended)
            
            feature_variance_extended = cv2.copyMakeBorder(feature_variance, r, r, r, r, cv2.BORDER_REPLICATE) 
            feature_variances_extended.append(feature_variance_extended)
            
            gL = (feature - shiftImage(feature_extended, 0, -1, original_shape, r)) / 2
            gR = (feature - shiftImage(feature_extended, 0, 1, original_shape, r)) / 2
            gU = (feature - shiftImage(feature_extended, 1, 0, original_shape, r)) / 2
            gD = (feature - shiftImage(feature_extended, -1, 0, original_shape, r)) / 2

            feature_sqrgrad = np.minimum(gL**2, gR**2) + np.minimum(gU**2, gD**2)
            feature_sqrgrads.append(feature_sqrgrad)
    
    color_extended = cv2.copyMakeBorder(color, r, r, r, r, cv2.BORDER_REPLICATE)
    color1_extended = cv2.copyMakeBorder(color1, r, r, r, r, cv2.BORDER_REPLICATE)
    color2_extended = cv2.copyMakeBorder(color2, r, r, r, r, cv2.BORDER_REPLICATE)
    # variance filering
    color_variance_filtered = cv2 .boxFilter(color_variance, -1, (5, 5))
    color_variance = np.maximum(color_variance, color_variance_filtered)
    color_variance_extended = cv2.copyMakeBorder(color_variance, r, r, r, r, cv2.BORDER_REPLICATE)

    flt1 = np.zeros_like(color)
    flt2 = np.zeros_like(color)
    wgtsum = np.zeros(original_shape[:2])

    for dx in range(-r, r+1):
        for dy in range(-r, r+1):
            color_shifted = shiftImage(color_extended, dx, dy, original_shape, r)
            color1_shifted = shiftImage(color1_extended, dx, dy, original_shape, r)
            color2_shifted = shiftImage(color2_extended, dx, dy, original_shape, r)
            color_variance_shifted = shiftImage(color_variance_extended, dx, dy, original_shape, r)
            # w_c
            d2c_pixel = d2_color(color, color_shifted, color_variance, color_variance_shifted, kc)
            d2c_patch = cv2.boxFilter(d2c_pixel, -1, (2*f+1, 2*f+1))
            wgt = np.exp(-np.maximum(d2c_patch, 0))
            # w_f
            if use_feature:
                d2f_max = np.zeros(original_shape[:2])
                for f_idx in range(len(features)):
                    feature = features[f_idx]
                    feature_variance = feature_variances[f_idx]
                    feature_shifted = shiftImage(features_extended[f_idx], dx, dy, original_shape, r)
                    feature_variance_shifted = shiftImage(feature_variances_extended[f_idx], dx, dy, original_shape, r)
                    feature_sqrgrad = feature_sqrgrads[f_idx]
                    d2f_pixel = d2_feature(feature, feature_shifted, feature_variance, feature_variance_shifted, feature_sqrgrad, tau, kf)
                    d2f_max = np.maximum(d2f_max, d2f_pixel)
                wgt_f = np.exp(-d2f_max)
                wgt = np.minimum(wgt, wgt_f)
            
            # wgt = cv2.boxFilter(wgt, -1, (2*f-1, 2*f-1))
            flt1 += np.expand_dims(wgt, -1) * color1_shifted
            flt2 += np.expand_dims(wgt, -1) * color2_shifted
            wgtsum += wgt

    if use_half:
        flt1 /= np.expand_dims(wgtsum, -1)
        flt2 /= np.expand_dims(wgtsum, -1)
        result = (flt1+flt2) / 2
        variance = (flt1 - flt2) **2 / 4
        # apply gaussian blur on variance
        variance = cv2.GaussianBlur(variance, (0,0), 0.5)
        return result, variance
    else:
        return flt1 / np.expand_dims(wgtsum, -1)


#%%
# Feature prefiltering
normal_denoised, normal_variance_denoised = nl_means_denoising([normal1, normal2], normal_variance, r=5, f=3, kc=1)
texture_denoised, texture_variance_denoised = nl_means_denoising([texture1, texture2], texture_variance, r=5, f=3, kc=1)
depth_denoised, depth_variance_denoised = nl_means_denoising([depth1, depth2], depth_variance, r=5, f=3, kc=1)
visibility_denoised, visibility_variance_denoised = nl_means_denoising([visibility1, visibility2], visibility_variance, r=5, f=3, kc=1)

# %%
features = [
    normal_denoised, 
    texture_denoised, 
    depth_denoised, 
    visibility_denoised
]
feature_variances = [
    normal_variance_denoised, 
    texture_variance_denoised, 
    depth_variance_denoised, 
    visibility_variance_denoised
]
color_nl_f_denoised = nl_means_denoising(color, color_variance, features, feature_variances, r=10, f=3, kc=0.45, kf = 0.6, tau=1e-3)

#%%

img_to_save = GammaCorrection(color_nl_f_denoised)
saveImage("nl_feature_denoised.jpg", img_to_save)

# %%
