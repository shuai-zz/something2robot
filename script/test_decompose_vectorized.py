"""Standalone equivalence test: old triple-loop voxel classification vs new vectorized version.

Replicates the exact logic from auto_design/modules/mesh_decomp.py decompose()
on synthetic data and asserts identical assignments.
"""
import numpy as np

rng = np.random.default_rng(42)

# Synthetic link tree: BODY with shared joints to two legs and a head
all_links_joints = {
    'BODY': {'neck': (0, 3, 11), 'l_hip': (9, 2, -14), 'r_hip': (-7, 2, -14), 'belly': (0, 2, -11)},
    'HEAD': {'neck': (0, 3, 11), 'hat': (0, 3, 35), 'nose': (0, -14, 18)},
    'L_LEG': {'l_hip': (9, 2, -14), 'l_knee': (9, 4, -27)},
    'L_LOWER_LEG': {'l_knee': (9, 4, -27), 'l_ankle': (9, 4, -37)},
    'L_FOOT': {'l_ankle': (9, 4, -37), 'l_foot': (15, -6, -38)},
    'R_LEG': {'r_hip': (-7, 2, -14), 'r_knee': (-8, 4, -27)},
    'SINGLE': {'lone': (30, 30, 30)},  # single-joint link (no segments)
}

all_joints = {}
for link_name, joints in all_links_joints.items():
    for jn, jp in joints.items():
        all_joints.setdefault(jn, []).append((link_name, np.asarray(jp, dtype=float)))

link_segments = {}
for link_name, joints in all_links_joints.items():
    jps = [np.asarray(p, dtype=float) for p in joints.values()]
    link_segments[link_name] = [(jps[i], jps[j]) for i in range(len(jps)) for j in range(i + 1, len(jps))]

# Voxels: cloud around the skeleton + noise
segs_all = [s for segs in link_segments.values() for s in segs]
voxels = []
for _ in range(20000):
    p1, p2 = segs_all[rng.integers(len(segs_all))]
    voxels.append(p1 + (p2 - p1) * rng.random() + rng.normal(0, 2, 3))
voxels = np.asarray(voxels)


def point_to_segment_distance(point, segment):
    p1, p2 = segment
    line_vec = p2 - p1
    point_vec = point - p1
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return np.linalg.norm(point_vec)
    t = max(0, min(1, np.dot(point_vec, line_vec) / (line_len * line_len)))
    projection = p1 + t * line_vec
    return np.linalg.norm(point - projection)


# ---------- OLD (reference) ----------
old_result = np.array(['UNCLASSIFIED'] * len(voxels), dtype=object)
for i, voxel in enumerate(voxels):
    closest_joint, min_distance = None, float('inf')
    for joint_name, joint_info in all_joints.items():
        for link_name, joint_pos in joint_info:
            d = np.linalg.norm(voxel - joint_pos)
            if d < min_distance:
                min_distance, closest_joint = d, joint_name
    if len(all_joints[closest_joint]) == 1:
        old_result[i] = all_joints[closest_joint][0][0]
    else:
        min_seg_dist, closest_link = float('inf'), None
        for link_name, joint_pos_for_link in all_joints[closest_joint]:
            if len(link_segments[link_name]) == 0:
                d = np.linalg.norm(voxel - joint_pos_for_link)
                if d < min_seg_dist:
                    min_seg_dist, closest_link = d, link_name
            else:
                for segment in link_segments[link_name]:
                    d = point_to_segment_distance(voxel, segment)
                    if d < min_seg_dist:
                        min_seg_dist, closest_link = d, link_name
        old_result[i] = closest_link

# ---------- NEW (vectorized, mirrors the edited decompose()) ----------
def points_to_segments_min_distance(points, segments):
    p1 = segments[:, 0, :]
    line_vec = segments[:, 1, :] - p1
    point_vec = points[:, None, :] - p1[None, :, :]
    line_len_sq = np.einsum('ij,ij->i', line_vec, line_vec)
    safe_len_sq = np.where(line_len_sq == 0, 1.0, line_len_sq)
    t = np.einsum('msj,sj->ms', point_vec, line_vec) / safe_len_sq[None, :]
    t = np.clip(t, 0.0, 1.0)
    projection = p1[None, :, :] + t[:, :, None] * line_vec[None, :, :]
    dist = np.linalg.norm(points[:, None, :] - projection, axis=-1)
    return dist.min(axis=1)

joint_names = list(all_joints.keys())
joint_positions = np.array([np.asarray(all_joints[jn][0][1], dtype=float) for jn in joint_names])
joint_single_link = np.array(
    [all_joints[jn][0][0] if len(all_joints[jn]) == 1 else '' for jn in joint_names], dtype=object)

v = voxels.astype(float)
joint_sq = (joint_positions ** 2).sum(axis=1)
dist_sq = (v ** 2).sum(axis=1)[:, None] + joint_sq[None, :] - 2.0 * v @ joint_positions.T
closest_joint_idx = np.argmin(dist_sq, axis=1)

new_result = joint_single_link[closest_joint_idx].copy()
ambiguous = np.nonzero(new_result == '')[0]
amb_joint_idx = closest_joint_idx[ambiguous]
for j_idx in np.unique(amb_joint_idx):
    rows = ambiguous[amb_joint_idx == j_idx]
    pts = v[rows]
    candidate_links = [ln for ln, _ in all_joints[joint_names[j_idx]]]
    candidate_dists = []
    for link_name, joint_pos_for_link in all_joints[joint_names[j_idx]]:
        if len(link_segments[link_name]) == 0:
            candidate_dists.append(np.linalg.norm(pts - joint_pos_for_link, axis=1))
        else:
            candidate_dists.append(points_to_segments_min_distance(
                pts, np.asarray(link_segments[link_name], dtype=float)))
    best = np.argmin(np.stack(candidate_dists, axis=1), axis=1)
    new_result[rows] = np.array(candidate_links, dtype=object)[best]

mismatch = np.nonzero(old_result != new_result)[0]
print(f'voxels: {len(voxels)}, ambiguous: {len(ambiguous)}, mismatches: {len(mismatch)}')
if len(mismatch):
    for i in mismatch[:10]:
        print(' ', voxels[i], old_result[i], new_result[i])
unique, counts = np.unique(new_result, return_counts=True)
print('assignment counts:', dict(zip(unique.tolist(), counts.tolist())))
assert len(mismatch) == 0, 'MISMATCH between old and new classification'
print('OK: vectorized classification identical to old loop')
