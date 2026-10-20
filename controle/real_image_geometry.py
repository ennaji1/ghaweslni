"""Passage des coins réels au repère image d'entraînement, sans pose externe."""
import numpy as np
WIDTH, HEIGHT = 1280, 720
K_SIM = np.array([[1043.866511210409, 0, 640.5], [0, 1043.866511210409, 360.5], [0, 0, 1.]])
def canonical_corners(points, matrix, distortion):
    import cv2
    return cv2.undistortPoints(np.asarray(points, dtype=np.float64).reshape(4,1,2),
                              np.asarray(matrix,dtype=float), np.asarray(distortion,dtype=float),
                              P=K_SIM).reshape(4,2)
