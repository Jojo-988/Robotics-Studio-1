"""Free-centre geometric fits. Quality gates are diagnostics, not acceptance tolerances."""
import numpy as np


def spatial_clusters(points, distance=.12):
    """XY connected components, independent of the expected landmark centre."""
    p=np.asarray(points)
    remaining=set(range(len(p))); groups=[]
    while remaining:
        seed=remaining.pop(); group=[seed]; pending=[seed]
        while pending:
            i=pending.pop()
            neighbours=[j for j in remaining if np.linalg.norm(p[i,:2]-p[j,:2])<=distance]
            remaining.difference_update(neighbours);group.extend(neighbours);pending.extend(neighbours)
        groups.append(p[group])
    return groups


def fit_round(points):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] not in (2, 3) or len(p) < 12 or not np.isfinite(p).all():
        raise ValueError('INSUFFICIENT_SUPPORT: need >=12 finite 2D/3D points')
    origin = p.mean(axis=0)
    q = p-origin
    design = np.column_stack((2*q, np.ones(len(q))))
    condition = np.linalg.cond(design)
    if condition > 1000:
        raise ValueError('INSUFFICIENT_SUPPORT: ill-conditioned geometry')
    solution, _, rank, _ = np.linalg.lstsq(design, np.sum(q*q, axis=1), rcond=None)
    if rank < p.shape[1]+1:
        raise ValueError('INSUFFICIENT_SUPPORT: rank deficient')
    centre = solution[:-1]+origin
    r2 = solution[-1]+np.dot(solution[:-1],solution[:-1])
    if r2 <= 0:
        raise ValueError('INVALID_GEOMETRY: nonpositive radius')
    radius = np.sqrt(r2)
    residual = np.abs(np.linalg.norm(p-centre,axis=1)-radius)
    angles = np.sort(np.mod(np.arctan2(p[:,1]-centre[1],p[:,0]-centre[0]),2*np.pi))
    span = float((2*np.pi-np.max(np.diff(np.r_[angles,angles[0]+2*np.pi])))*180/np.pi)
    if span < 60:
        raise ValueError('INSUFFICIENT_SUPPORT: visible XY arc below 60 degrees')
    return dict(centre=centre.tolist(), radius=float(radius), points=len(p),
                condition=float(condition), xy_arc_deg=span,
                p95_residual_m=float(np.percentile(residual,95)))
