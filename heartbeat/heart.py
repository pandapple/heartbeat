import numpy as np
from skimage import measure
import matplotlib.pyplot as plt

def heart_func(x, y, z):
    return (x**2 + (9/4)*y**2 + z**2 - 1)**3 - x**2 * z**3 - (9/80)*y**2 * z**3


def extract_mesh(resolution=128):
    x = np.linspace(-1.5, 1.5, resolution)
    y = np.linspace(-1.5, 1.5, resolution)
    z = np.linspace(-1.5, 1.5, resolution)

    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")

    V = heart_func(X, Y, Z)

    verts, faces, normals, values = measure.marching_cubes(V, level=0, spacing=(x[1]-x[0], y[1]-y[0], z[1]-z[0]))
    return verts, faces


def sample_points_from_mesh(verts, faces, n_points=20000):
    # 三角形顶点
    v0 = verts[faces[:, 0]]
    v1 = verts[faces[:, 1]]
    v2 = verts[faces[:, 2]]

    tri_areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)

    prob = tri_areas / np.sum(tri_areas)
    cdf = np.cumsum(prob)

    samples = np.random.rand(n_points)
    tri_ids = np.searchsorted(cdf, samples)

    A = v0[tri_ids]
    B = v1[tri_ids]
    C = v2[tri_ids]

    u = np.random.rand(n_points)
    v = np.random.rand(n_points)
    mask = u + v > 1
    u[mask] = 1 - u[mask]
    v[mask] = 1 - v[mask]

    sampled = A + u[:, None] * (B - A) + v[:, None] * (C - A)
    return sampled


if __name__ == "__main__":
    print("Extracting heart mesh...")
    verts, faces = extract_mesh(160)

    print("Sampling points...")
    pts = sample_points_from_mesh(verts, faces, n_points=30000)

    print("Plotting result...")
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1, c=pts[:, 2], cmap="rainbow")
    ax.set_box_aspect([1,1,1])
    plt.show()
