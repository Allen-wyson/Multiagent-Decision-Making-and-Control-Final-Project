import numpy as np

def get_frenet_coords(x, y, centerline, headings):
    
    # Compute closest point on the centerline
    dx = centerline[:,0] - x
    dy = centerline[:,1] - y
    distances = np.hypot(dx,dy)
    idx = np.argmin(distances)

    # Compute arc length s
    s = np.sum(np.hypot(np.diff(centerline[:idx+1, 0]), np.diff(centerline[:idx+1, 1])))

    # Compute deviation d
    path_heading = headings[idx]
    normal = np.array([-np.sin(path_heading), np.cos(path_heading)])
    rel_pos = np.array([x, y]) - centerline[idx]
    d = np.dot(rel_pos, normal)

    return s, d