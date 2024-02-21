# -*- coding: utf-8 -*-
import numpy as np

"""

Implementation of Lloyd's algorithm for K-means clustering.

Lloyd, S.(1982). Least squares quantization in PCM. IEEE Transactions On Information Theory 28 129–137.

Args:
    X (numpy.ndarray, shape=(n_samples, n_dim)): The dataset
    k (int): Number of cluster
    num_iter (int): Number of iterations to run the algorithm
    centers (numpy.ndarray, shape=(k, )): Starting centers

Returns:
    cluster_assignments (numpy.ndarray; shape=(n_samples, n_dim)): The clustering label for each data point
    centers (numpy.ndarray, shape=(k, )): The cluster centers

    Source: https://gist.github.com/colonialjelly/e980d83cae2f909c1cb777c697aa0369

"""


def k_means(X, k, centers=None, num_iter=100):
    if centers is None:
        rnd_centers_idx = np.random.choice(np.arange(X.shape[0]), k, replace=False)
        centers = X[rnd_centers_idx]
    for _ in range(num_iter):
        distances = np.sum(np.sqrt((X - centers[:, np.newaxis]) ** 2), axis=-1)
        cluster_assignments = np.argmin(distances, axis=0)
        for i in range(k):
            msk = (cluster_assignments == i)
            centers[i] = np.mean(X[msk], axis=0) if np.any(msk) else centers[i]
    return cluster_assignments, centers
