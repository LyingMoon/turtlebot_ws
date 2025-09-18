#!/usr/bin/env python3
"""
ICP (Iterative Closest Point) Utilities for TurtleBot3

This module provides utility functions for point cloud registration
using the Iterative Closest Point algorithm for SLAM applications.
"""

from typing import Optional, Tuple
import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
from tqdm import tqdm, trange


def best_fit_transform(A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Calculate the least-squares best-fit transform that maps points A to B.

    Args:
        A: Naxm numpy array of source points
        B: Nbxm numpy array of destination points

    Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix
        R: mxm rotation matrix
        t: mx1 translation vector
    """
    if A.shape[1] != B.shape[1]:
        raise ValueError("Point sets must have the same dimensionality")

    # Get number of dimensions
    m = A.shape[1]

    # Translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # Compute rotation matrix using SVD
    H = np.dot(AA.T, BB)
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # Handle special reflection case
    if np.linalg.det(R) < 0:
        Vt[m - 1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Compute translation
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # Create homogeneous transformation matrix
    T = np.eye(m + 1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(
    src: np.ndarray,
    dst: np.ndarray,
    radius: float = 0.01
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find the nearest Euclidean neighbor in dst for each point in src.

    Args:
        src: Nxm array of source points
        dst: Nxm array of destination points
        radius: Search radius for nearest neighbors

    Returns:
        distances: Euclidean distances of the nearest neighbors
        indices: Destination indices of the nearest neighbors
    """
    if src.shape[1] != dst.shape[1]:
        raise ValueError("Source and destination must have same dimensionality")

    try:
        neigh = NearestNeighbors(n_neighbors=1, radius=radius)
        neigh.fit(dst)
        distances, indices = neigh.kneighbors(src, return_distance=True)
        return distances.ravel(), indices.ravel()

    except Exception as e:
        raise RuntimeError(f"Nearest neighbor search failed: {e}")


def icp(
    A: np.ndarray,
    B: np.ndarray,
    init_pose: Optional[np.ndarray] = None,
    max_iterations: int = 20,
    tolerance: float = 0.001,
    knn_radius: float = 0.01,
    verbose: bool = False
) -> np.ndarray:
    """
    Iterative Closest Point algorithm for point cloud registration.

    Finds the best-fit transform that maps points A onto points B using
    iterative refinement of point correspondences.

    Args:
        A: Nxm numpy array of source points
        B: Nxm numpy array of destination points
        init_pose: Initial (m+1)x(m+1) homogeneous transformation
        max_iterations: Maximum number of iterations
        tolerance: Convergence criteria for mean error change
        knn_radius: Search radius for nearest neighbor matching
        verbose: Enable verbose output

    Returns:
        T: Final homogeneous transformation matrix that maps A onto B

    Raises:
        ValueError: If input arrays have incompatible shapes
        RuntimeError: If ICP fails to converge or encounters errors
    """
    if A.shape[1] != B.shape[1]:
        raise ValueError("Point clouds must have same dimensionality")

    if A.shape[0] == 0 or B.shape[0] == 0:
        raise ValueError("Point clouds cannot be empty")

    # Get number of dimensions
    m = A.shape[1]

    # Make points homogeneous, copy to preserve originals
    src = np.ones((m + 1, A.shape[0]))
    dst = np.ones((m + 1, B.shape[0]))
    src[:m, :] = np.copy(A.T)
    dst[:m, :] = np.copy(B.T)

    # Apply initial pose estimation
    if init_pose is not None:
        if init_pose.shape != (m + 1, m + 1):
            raise ValueError(f"Initial pose must be {m+1}x{m+1} matrix")
        src = np.dot(init_pose, src)

    prev_error = float('inf')
    T_total = np.eye(m + 1)

    try:
        iteration_range = trange(max_iterations) if verbose else range(max_iterations)

        for i in iteration_range:
            # Find nearest neighbors
            distances, indices = nearest_neighbor(
                src[:m, :].T, dst[:m, :].T, radius=knn_radius
            )

            # Get matched point pairs
            matched_src = src[:m, :].T
            matched_dst = dst[:m, :].T[indices]

            # Compute transformation for this iteration
            T, _, _ = best_fit_transform(matched_src, matched_dst)

            # Apply transformation
            src = np.dot(T, src)
            T_total = np.dot(T, T_total)

            # Check for convergence
            mean_error = np.mean(distances)

            if verbose:
                print(f"Iteration {i+1}: Mean error = {mean_error:.6f}")

            if abs(prev_error - mean_error) < tolerance:
                if verbose:
                    print(f"Converged at iteration {i+1}")
                break

            prev_error = mean_error

        else:
            if verbose:
                print(f"Reached maximum iterations ({max_iterations})")

    except Exception as e:
        raise RuntimeError(f"ICP algorithm failed: {e}")

    # Calculate final transformation from original A to final position
    T_final, _, _ = best_fit_transform(A, src[:m, :].T)

    return T_final


def open3d_icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    T_init: np.ndarray,
    max_distance: float = 1.0
) -> np.ndarray:
    """
    Perform ICP using Open3D's optimized implementation.

    Args:
        source: Source point cloud
        target: Target point cloud
        T_init: Initial transformation estimate
        max_distance: Maximum correspondence distance

    Returns:
        transformation: Final transformation matrix

    Raises:
        ValueError: If inputs are invalid
        RuntimeError: If ICP fails
    """
    if not isinstance(source, o3d.geometry.PointCloud):
        raise ValueError("Source must be Open3D PointCloud")
    if not isinstance(target, o3d.geometry.PointCloud):
        raise ValueError("Target must be Open3D PointCloud")

    try:
        # Estimate normals for point-to-plane ICP
        o3d.geometry.PointCloud.estimate_normals(
            source,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
        )

        o3d.geometry.PointCloud.estimate_normals(
            target,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
        )

        # Perform point-to-plane ICP registration
        reg_result = o3d.pipelines.registration.registration_icp(
            source, target, max_distance, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )

        return reg_result.transformation

    except Exception as e:
        raise RuntimeError(f"Open3D ICP failed: {e}")


def validate_transformation(T: np.ndarray, dim: int = 3) -> bool:
    """
    Validate that a transformation matrix is properly formed.

    Args:
        T: Transformation matrix to validate
        dim: Expected spatial dimension

    Returns:
        bool: True if transformation is valid
    """
    expected_shape = (dim + 1, dim + 1)

    if T.shape != expected_shape:
        return False

    # Check if rotation part is orthogonal
    R = T[:dim, :dim]
    if not np.allclose(np.dot(R, R.T), np.eye(dim), rtol=1e-6):
        return False

    # Check if determinant is 1 (proper rotation)
    if not np.allclose(np.linalg.det(R), 1.0, rtol=1e-6):
        return False

    # Check if bottom row is [0, 0, 0, 1]
    expected_bottom = np.zeros(dim + 1)
    expected_bottom[-1] = 1.0
    if not np.allclose(T[dim, :], expected_bottom, rtol=1e-6):
        return False

    return True