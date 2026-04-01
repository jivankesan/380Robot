import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

R = 0.15    # turn arc radius (m)
L = 0.3     # tile side length (m)
N = 60      # centerline sample-points per tile


class TrackBuilder:
    def __init__(self):
        self.pts        = []
        self.joints     = [(0.0, 0.0, 0.0)]  # (x, y, heading) at every tile seam
        self.tile_types = []
        self.x = self.y = 0.0
        self.h = 0.0  # heading (rad), 0 = east

    def S(self, kind='S'):
        for i in range(1, N + 1):
            t = i / N
            self.pts.append((
                self.x + t * L * np.cos(self.h),
                self.y + t * L * np.sin(self.h),
            ))
        self.x += L * np.cos(self.h)
        self.y += L * np.sin(self.h)
        self.joints.append((self.x, self.y, self.h))
        self.tile_types.append(kind)
        return self

    def T(self, d):   # d = -1 CW (right),  +1 CCW (left)
        cx = self.x + R * np.cos(self.h + d * np.pi / 2)
        cy = self.y + R * np.sin(self.h + d * np.pi / 2)
        a0 = np.arctan2(self.y - cy, self.x - cx)
        for i in range(1, N + 1):
            t = i / N
            a = a0 + d * t * np.pi / 2
            self.pts.append((cx + R * np.cos(a), cy + R * np.sin(a)))
        a1 = a0 + d * np.pi / 2
        self.x = cx + R * np.cos(a1)
        self.y = cy + R * np.sin(a1)
        self.h += d * np.pi / 2
        self.joints.append((self.x, self.y, self.h))
        self.tile_types.append('R' if d == -1 else 'L')
        return self

    def R(self):      return self.T(-1)
    def Lf(self):     return self.T(+1)
    def Target(self): return self.S(kind='T')

    def build(self):
        return np.array(self.pts), self.joints, self.tile_types


def build_course():
    tb = TrackBuilder()
    (tb
     .S().S().S().S().S()
     .R()
     .S().S().S().S()
     .R().R()
     .Lf().Lf()
     .R()
     .S().S()
     .R()
     .S()
     .R()
     .S()
     .Lf()
     .Target()
    )
    return tb.build()


def tile_corners(entry_x, entry_y, entry_h):
    """0.3 × 0.3 m square aligned with entry heading. Entry point at centre of back edge."""
    fx, fy = np.cos(entry_h), np.sin(entry_h)
    nx, ny = -fy, fx                           # left-perpendicular unit vector
    half = L / 2

    return np.array([
        [entry_x + half * nx,            entry_y + half * ny],             # back-left
        [entry_x - half * nx,            entry_y - half * ny],             # back-right
        [entry_x + L * fx - half * nx,   entry_y + L * fy - half * ny],   # front-right
        [entry_x + L * fx + half * nx,   entry_y + L * fy + half * ny],   # front-left
    ])


def tile_mid(joints, i):
    x0, y0, h = joints[i]
    return x0 + (L / 2) * np.cos(h), y0 + (L / 2) * np.sin(h)


def main():
    pts, joints, tile_types = build_course()
    n_tiles = len(tile_types)

    tx0, ty0, th = joints[-2]
    target_x = tx0 + (L / 2) * np.cos(th)
    target_y  = ty0 + (L / 2) * np.sin(th)

    fig, ax = plt.subplots(figsize=(15, 11))

    face_colors = {
        'S': '#d4a96a',
        'R': '#c8974f',
        'L': '#c8974f',
        'T': '#b8c8a0',
    }

    for i, ttype in enumerate(tile_types):
        ex, ey, eh = joints[i]
        corners = tile_corners(ex, ey, eh)
        ax.add_patch(plt.Polygon(
            corners,
            facecolor=face_colors[ttype],
            edgecolor='#7a5c2e',
            linewidth=1.2,
            zorder=1,
        ))

    ax.plot(pts[:, 0], pts[:, 1],
            color='red', lw=2.8, zorder=3,
            solid_capstyle='round', label='Red line (centerline)')

    step = max(1, len(pts) // 20)
    for i in range(0, len(pts) - step, step):
        dx = pts[i + step, 0] - pts[i, 0]
        dy = pts[i + step, 1] - pts[i, 1]
        ax.annotate('',
                    xy=(pts[i, 0] + dx * 0.45, pts[i, 1] + dy * 0.45),
                    xytext=(pts[i, 0], pts[i, 1]),
                    arrowprops=dict(arrowstyle='->', color='#003399', lw=1.6),
                    zorder=4)

    for i, ttype in enumerate(tile_types):
        mx, my = tile_mid(joints, i)
        label = f"T{i+1}" if ttype != 'T' else "TARGET"
        ax.text(mx, my, label,
                ha='center', va='center', fontsize=6.5,
                color='#1a1a1a', fontweight='bold', zorder=5,
                bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='none', alpha=0.55))

    ax.plot(*pts[0], 'go', ms=11, zorder=7, markeredgecolor='darkgreen', label='Start')
    ax.text(pts[0, 0], pts[0, 1] + 0.09, 'START',
            ha='center', color='darkgreen', fontweight='bold', fontsize=10, zorder=7)

    # Target bull's-eye: blue → white → red → white → black
    ring_r      = [0.070, 0.060, 0.045, 0.030, 0.010]
    ring_colors = ['#1144EE', 'white', 'red', 'white', '#111111']
    for r, c in zip(ring_r, ring_colors):
        ax.add_patch(plt.Circle((target_x, target_y), r,
                                fc=c, ec='#222222', lw=0.7, zorder=8))
    ax.plot(target_x, target_y, 'k+', ms=9, mew=1.8, zorder=9)
    ax.text(target_x, target_y - 0.12, 'TARGET\n(Ø ≈ 0.14 m)',
            ha='center', fontsize=8, color='darkred', fontweight='bold', zorder=9)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25, zorder=0)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(
        f'Robot Course Map  ─  {n_tiles} tiles  |  tile = {L} m × {L} m square  |  turn radius = {R} m  (arc centre at inner corner)',
        fontsize=13, fontweight='bold',
    )

    legend_handles = [
        mpatches.Patch(fc='#d4a96a', ec='#7a5c2e', label='Straight tile'),
        mpatches.Patch(fc='#c8974f', ec='#7a5c2e', label='Turn tile (R or L)'),
        mpatches.Patch(fc='#b8c8a0', ec='#7a5c2e', label='Target tile'),
        plt.Line2D([0], [0], color='red', lw=2.5, label='Red centerline'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green',
                   markersize=10, label='Start'),
    ]
    ax.legend(handles=legend_handles, loc='upper right', fontsize=9)

    plt.tight_layout()
    out = 'track_map.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"[track_viz] saved → {out}")
    print(f"[track_viz] total tiles : {n_tiles}  (straight={tile_types.count('S')}  "
          f"R={tile_types.count('R')}  L={tile_types.count('L')}  target={tile_types.count('T')})")
    print(f"[track_viz] target centre: ({target_x:.3f}, {target_y:.3f}) m")
    print(f"[track_viz] bounding box : "
          f"x=[{pts[:,0].min():.3f}, {pts[:,0].max():.3f}]  "
          f"y=[{pts[:,1].min():.3f}, {pts[:,1].max():.3f}]")
    plt.show()


if __name__ == '__main__':
    main()
