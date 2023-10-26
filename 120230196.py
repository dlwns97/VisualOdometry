import cv2
import numpy as np
import argparse

# FUNCTION.
#
# RANSAC ALGORITHM
# ARGUMENT: IMG1 KEYPOINTS, IMG2 KEYPOINTS
#
def DIY_RANSAC(src, dst,  max_iters, threshold):
    # Exception handling #
    if len(src) < 4:
        print("Error, Insufficient matching points")
        return
    k_length = len(src)

    homography = None
    best_inliers = []

    # RANSAC algorithm #
    for i in range(max_iters):
        # From entire keypoints, select random four points #
        indices = np.random.choice(k_length, 4, replace=False)
        sample_src = src[indices]
        sample_dst = dst[indices]

        # Candidate homography matrix #
        homomatrix, _ = cv2.findHomography(sample_src, sample_dst, method=0)

        """
        tmp = []
        for (x1,y1), (x2,y2) in zip(sample_src, sample_dst):
            tmp.append([-x1, -y1, -1, 0, 0, 0, x1*x2, y1*x2, x2])
            tmp.append([0, 0, 0, -x1, -y1, -1, x1*y2, y1*y2, y2])
        tmp = np.array(tmp)
        _, _, tmp_homo = np.linalg.svd(tmp)
        homomatrix = tmp_homo[-1].reshape(3,3)
        homomatrix /= homomatrix[2, 2]
        """

        # Transform img1 points to predict (match) img2 points and Regularization #
        homo_points = np.dot(homomatrix, np.vstack((src.T, np.ones(src.shape[0]))))
        homo_points /= homo_points[2]
        homo_points = homo_points[:2].T

        # Compute distance from homography points and ground truth points #
        e_distance = np.sqrt(np.sum((dst - homo_points)**2, axis=1))
        inliers = [index for index, dist in enumerate(e_distance) if dist < threshold]

        # Set best homography matrix and inliers #
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            homography = homomatrix

    return homography, best_inliers

# FUNCTION.
#
# FIND THE BEST HOMOGRAPHY MATRIX WITH GREAT KEYPOINT MATCHING USING RANSAC
# ARGUMENT: MATHCED KEYPOINTS (MATCHING RESULT, KEYPOINTS, ITERATIONS, THRESHOLD)
#
def DIY_HOMOGRAPHY(k1, k2, matches, max_iters, threshold):
    # Image1, Image2 matching keypoints #
    src = np.float32([k1[match.queryIdx].pt for match in matches])
    dst = np.float32([k2[match.trainIdx].pt for match in matches])

    homography, best_inliers = DIY_RANSAC(src, dst, max_iters, threshold)
    best_matches = [matches[i] for i in best_inliers]

    return homography, best_matches

# FUNCTION.
#
# WARPING IMAGE TO PANORAMA FRAME BY WARPING MATRIX
# ARGUMENT: ONE IMAGE, WARPING MATRIX, SIZE
#
def DIY_WARPING(matrix, im2, panorama_size):
    im2_h, im2_w = im2.shape[:2]
    
    # Set panorama frame #
    panorama = np.zeros(panorama_size, dtype=np.uint8)
    panorama_height, panorama_width = panorama_size[:2]

    I_matrix = np.linalg.inv(matrix)
    for y in range(panorama_height):
        for x in range(panorama_width):
            im = np.dot(I_matrix, [x,y,1])
            im /= im[2]
            im = im[:2].astype(int)
            # If available position, put image2 pixels in frame #
            if 0<=im[1]<im2_h and 0<=im[0]<im2_w:
                panorama[y, x] = im2[im[1], im[0]]
    return panorama
# FUNCTION.
#
# GENERATE PANORAM IMAGE BY HOMOGRAPHY MATRIX
# ARGUMENT: INPUT IMAGES, HOMOGRAPHY MATRIX
#
def DIY_PANORAMA(im1, im2, homography):
    im1_h, im1_w = im1.shape[:2]
    im2_h, im2_w = im2.shape[:2]

    im1_corner = np.float32([[0,0], [0,im1_h], [im1_w, im1_h], [im1_w, 0]])
    im2_corner = np.float32([[0,0], [0,im2_h], [im2_w, im2_h], [im2_w, 0]])

    # Apply homography in image1 and image2 to find stiching point #
    homo_im1 = []
    for x,y in im1_corner:
        homo_point = np.dot(homography, [x,y,1])
        homo_point /= homo_point[2]
        homo_im1.append(homo_point[:2])
    homo_im1 = np.array(homo_im1)
    homo_points = np.vstack([im2_corner, homo_im1])

    # Compute range of panorama image #
    [x_min, y_min] = np.int32(np.min(homo_points, axis=0))
    [x_max, y_max] = np.int32(np.max(homo_points, axis=0))

    # RT matrix #
    rt_matrix = np.array([[1,0,-x_min], [0,1,-y_min], [0,0,1]])
    warping_matrix = np.dot(rt_matrix, homography)

    # Set size of final output #
    panorama_size = (y_max-y_min, x_max-x_min, 3)

    # Generate panorama image #
    panorama = DIY_WARPING(warping_matrix, im1, panorama_size)
    panorama[-y_min:-y_min+im2_h, -x_min:-x_min+im2_w] = im2

    return panorama

# FUNCTION.
#
# DO FUNDAMENTAL WORKS, LOAD IMAGES, EXTRACT KEYPOINT, SAVE IMAGE
# CALL OTHER FUNCTIONS
#
def main(path1, path2, output_path, max_iters, threshold):
    # Load images #
    im1 = cv2.imread(path1)
    im2 = cv2.imread(path2)

    # Generate ORB keypoint descriptor #
    orb = cv2.ORB_create()
    k1, d1 = orb.detectAndCompute(im1, None)
    k2, d2 = orb.detectAndCompute(im2, None)

    # Brute force matching with hamming distance #
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(d1, d2)
    matches = sorted(matches, key=lambda x: x.distance)

    # Save matching result, before homography #
    img_matches = cv2.drawMatches(
    im1, k1, im2, k2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite('Output/keypoint_match.png', img_matches)

    # Find homography matrix with RANSAC algorithm #
    homography, best_matches = DIY_HOMOGRAPHY(k1, k2, matches, max_iters, threshold)
    # Fail to generate homography matrix, return #
    if homography is None:
        print("Error, No homography matrix")
        return

    # Save best matching reuslt, after homography
    img_matches = cv2.drawMatches(
        im1, k1, im2, k2, best_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(f'Output/keypoint_best_{max_iters}_{threshold}_match.png', img_matches)

    # Genrate panorama image by homography matrix #
    panorama = DIY_PANORAMA(im1, im2, homography)

    # Save final panorama image #
    output_path = output_path + f"{max_iters}_{threshold}.jpg"
    cv2.imwrite(output_path, panorama)
    

if __name__ == '__main__':
    # Set argument parser #
    parser = argparse.ArgumentParser(description='Use two images with dif view point to make panorama')
    parser.add_argument('--input_path', nargs="+", type=str, default=["Input/myseat1.jpg", "Input/myseat2.jpg"], help='Directory path to save captured images.')
    parser.add_argument('--output_path', type=str, default="Output/", help='Path to save panorama images')
    parser.add_argument('--max_iters', type=int, default=3000, help='Max iterations of RANSAC')
    parser.add_argument('--threshold', type=int, default=5, help='Threshold of RANSAC')

    args = parser.parse_args()
    main(args.input_path[0], args.input_path[1], args.output_path, args.max_iters, args.threshold)
