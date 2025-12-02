import numpy as np
from skimage import measure
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def heart_func(x, y, z):
    return (x**2 + (9/4)*y**2 + z**2 - 1)**3 - x**2 * z**3 - (9/80)*y**2 * z**3


def extract_mesh(resolution=180):
    # 稍微放大一点范围，保证心形完整
    x = np.linspace(-1.8, 1.8, resolution)
    y = np.linspace(-1.8, 1.8, resolution)
    z = np.linspace(-1.8, 1.8, resolution)

    X, Y, Z = np.meshgrid(x, y, z, indexing="ij")
    V = heart_func(X, Y, Z)

    verts, faces, normals, values = measure.marching_cubes(
        V, level=0, spacing=(x[1]-x[0], y[1]-y[0], z[1]-z[0])
    )
    return verts, faces


def sample_points_from_mesh(verts, faces, n_points=35000):
    v0 = verts[faces[:, 0]]
    v1 = verts[faces[:, 1]]
    v2 = verts[faces[:, 2]]

    tri_areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    prob = tri_areas / tri_areas.sum()
    cdf = np.cumsum(prob)

    r = np.random.rand(n_points)
    tri_ids = np.searchsorted(cdf, r)

    A = v0[tri_ids]
    B = v1[tri_ids]
    C = v2[tri_ids]

    u = np.random.rand(n_points)
    v = np.random.rand(n_points)
    mask = u + v > 1
    u[mask] = 1 - u[mask]
    v[mask] = 1 - v[mask]

    pts = A + u[:, None] * (B - A) + v[:, None] * (C - A)
    return pts


if __name__ == "__main__":
    print("Extracting mesh...")
    verts, faces = extract_mesh()

    print("Sampling points...")
    pts = sample_points_from_mesh(verts, faces, n_points=3000)

    center = pts.mean(axis=0)
    pts_orig = pts.copy()


    amplitude = 0.22      # 心跳幅度
    period_frames = 20    # 心跳周期帧数

    fig = plt.figure(figsize=(6, 6), facecolor='white', dpi=120)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor("white")


    scat = ax.scatter(
        pts[:, 0], pts[:, 1], pts[:, 2],
        s=2, c='#ff7eb9'  # 浅粉色
    )

    ax.set_axis_off()

    ax.set_box_aspect([1, 1, 1])

    margin_ratio = 0.03
    xmin, ymin, zmin = pts_orig.min(axis=0)
    xmax, ymax, zmax = pts_orig.max(axis=0)

    xspan = xmax - xmin
    yspan = ymax - ymin
    zspan = zmax - zmin
    span = max(xspan, yspan, zspan)
    cx, cy, cz = center

    half = span * (0.5 + margin_ratio)
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_zlim(cz - half, cz + half)

    plt.subplots_adjust(left=0, right=1, bottom=0, top=1)

    ax.view_init(elev=15, azim=-90)

    def update(frame):
        phase = 2 * np.pi * frame / period_frames
        scale = 1.0 + amplitude * np.sin(phase)

        pts_scaled = center + (pts_orig - center) * scale

        scat._offsets3d = (
            pts_scaled[:, 0],
            pts_scaled[:, 1],
            pts_scaled[:, 2],
        )
        return scat,

    anim = FuncAnimation(
        fig, update,
        frames=period_frames,
        interval=40,
        blit=False,
        repeat=True
    )

    plt.show()

