#!/usr/bin/env python3
# robot_arm_ccd.py
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# -------------------------
# Operações com quaternions
# -------------------------
def quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    w = np.cos(angle / 2.0)
    xyz = axis * np.sin(angle / 2.0)
    return np.array([w, xyz[0], xyz[1], xyz[2]])

def quat_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quat_mul(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.,0.,0.,0.])
    return q / n

def quat_rotate(q, v):
    qv = np.array([0.0, v[0], v[1], v[2]])
    res = quat_mul(quat_mul(q, qv), quat_conj(q))
    return res[1:]

# -------------------------
# Braço serial (bones)
# -------------------------
class SerialArmCCD:
    def __init__(self, lengths, joint_angle_limits=None):
        self.lengths = np.asarray(lengths, dtype=float)
        self.n = len(self.lengths)
        self.bones = np.array([[L, 0.0, 0.0] for L in self.lengths], dtype=float)
        self.initial_dirs = np.array([v/np.linalg.norm(v) for v in self.bones])
        if joint_angle_limits is None:
            self.limits = np.full(self.n, np.pi)
        else:
            self.limits = np.asarray(joint_angle_limits, dtype=float)
        self.joints = self.compute_joints()

    def compute_joints(self):
        pts = [np.array([0.0, 0.0, 0.0])]
        pos = np.array([0.0, 0.0, 0.0])
        for v in self.bones:
            pos = pos + v
            pts.append(pos.copy())
        self.joints = np.array(pts)
        return self.joints

    def end_effector(self):
        return self.joints[-1]

    def enforce_joint_limit(self, idx):
        cur_dir = self.bones[idx] / np.linalg.norm(self.bones[idx])
        ref = self.initial_dirs[idx]
        dot = np.clip(np.dot(cur_dir, ref), -1.0, 1.0)
        angle = np.arccos(dot)
        max_angle = self.limits[idx]
        if angle <= max_angle + 1e-12:
            return
        axis = np.cross(ref, cur_dir)
        if np.linalg.norm(axis) < 1e-12:
            if abs(ref[0]) < 0.9:
                axis = np.cross(ref, np.array([1.,0.,0.]))
            else:
                axis = np.cross(ref, np.array([0.,1.,0.]))
        axis = axis / np.linalg.norm(axis)
        correction_angle = angle - max_angle
        q_apply = quat_from_axis_angle(axis, -correction_angle)
        q_apply = quat_normalize(q_apply)
        for j in range(idx, self.n):
            self.bones[j] = quat_rotate(q_apply, self.bones[j])

    def solve_ik_ccd(self, target, max_iter=200, tol=1e-3, record=False):
        target = np.asarray(target, dtype=float)
        history = []
        self.compute_joints()

        for it in range(max_iter):
            if record:
                history.append(self.joints.copy())

            end = self.end_effector()
            err = np.linalg.norm(target - end)
            if err <= tol:
                if record:
                    history.append(self.joints.copy())
                print(f"Converged in {it} iterations with error {err:.6f}")
                return True, history

            for i in reversed(range(self.n)):
                joint_pos = self.joints[i]
                end_pos = self.end_effector()

                v_end = end_pos - joint_pos
                v_target = target - joint_pos

                n1 = np.linalg.norm(v_end)
                n2 = np.linalg.norm(v_target)
                if n1 < 1e-12 or n2 < 1e-12:
                    continue

                v_end_u = v_end / n1
                v_target_u = v_target / n2

                dot = np.clip(np.dot(v_end_u, v_target_u), -1.0, 1.0)
                if np.allclose(v_end_u, v_target_u):
                    continue

                axis = np.cross(v_end_u, v_target_u)
                axis_norm = np.linalg.norm(axis)
                if axis_norm < 1e-12:
                    if abs(v_end_u[0]) < 0.9:
                        tmp = np.array([1.0, 0.0, 0.0])
                    else:
                        tmp = np.array([0.0, 1.0, 0.0])
                    axis = np.cross(v_end_u, tmp)
                    axis_norm = np.linalg.norm(axis)
                    if axis_norm < 1e-12:
                        continue
                axis = axis / axis_norm

                angle = np.arccos(dot)
                alpha = 1.0
                angle_to_apply = alpha * angle

                q = quat_from_axis_angle(axis, angle_to_apply)
                q = quat_normalize(q)

                for j in range(i, self.n):
                    self.bones[j] = quat_rotate(q, self.bones[j])

                self.compute_joints()
                self.enforce_joint_limit(i)

        if record:
            history.append(self.joints.copy())
        return False, history

# -------------------------
# Plotagem e interface
# -------------------------
def plot_arm_3d(ax, joints, target=None):
    ax.cla()
    xs = joints[:,0]
    ys = joints[:,1]
    zs = joints[:,2]
    ax.plot(xs, ys, zs, marker='o', linewidth=2)
    if target is not None:
        ax.scatter([target[0]], [target[1]], [target[2]], marker='x', s=80)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    all_pts = joints
    if target is not None:
        all_pts = np.vstack([all_pts, target.reshape(1,3)])
    mins = all_pts.min(axis=0) - 0.5
    maxs = all_pts.max(axis=0) + 0.5
    rng = (maxs - mins).max() / 2.0
    center = (maxs + mins) / 2.0
    ax.set_xlim(center[0]-rng, center[0]+rng)
    ax.set_ylim(center[1]-rng, center[1]+rng)
    ax.set_zlim(center[2]-rng, center[2]+rng)
    ax.view_init(elev=25, azim=30)
    plt.pause(0.001)
    fig.canvas.draw_idle()

def run_interactive():
    N_SEGMENTS = 3
    LENGTHS = [2.0, 1.5, 1.0]
    JOINT_LIMITS_DEG = [120.0] * N_SEGMENTS
    JOINT_LIMITS = np.radians(JOINT_LIMITS_DEG)
    # alvo atual (pose do target mostrado)
    target_display = np.array([2.0, 1.0, 0.8])

    arm = SerialArmCCD(LENGTHS, joint_angle_limits=JOINT_LIMITS)
    arm.compute_joints()

    global fig
    fig = plt.figure(figsize=(10,6))
    ax3d = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(left=0.1, bottom=0.28)

    plot_arm_3d(ax3d, arm.joints, target=target_display)

    total_length = np.sum(LENGTHS)
    bound = total_length * 1.2
    ax_x = plt.axes([0.15, 0.16, 0.7, 0.03])
    ax_y = plt.axes([0.15, 0.12, 0.7, 0.03])
    ax_z = plt.axes([0.15, 0.08, 0.7, 0.03])
    s_x = Slider(ax_x, 'Target X', -bound, bound, valinit=target_display[0])
    s_y = Slider(ax_y, 'Target Y', -bound, bound, valinit=target_display[1])
    s_z = Slider(ax_z, 'Target Z', -bound, bound, valinit=target_display[2])

    ax_button_solve = plt.axes([0.15, 0.02, 0.15, 0.05])
    ax_button_reset = plt.axes([0.33, 0.02, 0.15, 0.05])
    btn_solve = Button(ax_button_solve, 'Solve IK')
    btn_reset = Button(ax_button_reset, 'Reset')

    info_text = fig.text(0.55, 0.02, '', fontsize=9)

    # --- UPDATED: sliders now only update target_display and redraw, without running IK ---
    def update_plot(_=None):
        nonlocal target_display
        # pegar novo alvo dos sliders (APENAS atualizar o marcador do alvo)
        target_display = np.array([s_x.val, s_y.val, s_z.val])
        # redesenha mantendo a pose atual do braço (sem chamar arm.solve_ik_ccd)
        plot_arm_3d(ax3d, arm.joints, target=target_display)
        info_text.set_text(f'End-effector: {arm.end_effector()}')
        fig.canvas.draw_idle()
        plt.pause(0.001)

    s_x.on_changed(update_plot)
    s_y.on_changed(update_plot)
    s_z.on_changed(update_plot)

    # botão solve: roda IK usando o target atual (o solve usa valores dos sliders)
    def on_solve(event):
        # aqui pegamos os valores atuais dos sliders como target real para o solver
        new_target = np.array([s_x.val, s_y.val, s_z.val])
        start_time = time.perf_counter()
        converged, history = arm.solve_ik_ccd(new_target, max_iter=400, tol=1e-4, record=True)
        arm.compute_joints()
        end_time = time.perf_counter()
        duration = end_time - start_time
        print(f"Solve IK completed in {duration:.6f} seconds.")

        plot_arm_3d(ax3d, arm.joints, target=new_target)
        info_text.set_text(f'Converged: {converged} | End-effector: {arm.end_effector()} | Err: {np.linalg.norm(arm.end_effector()-new_target):.6f}')
        fig.canvas.draw_idle()
        plt.pause(0.01)
        if history is None or len(history) <= 1:
            print("Nenhuma iteração registrada (history vazio) — nada para animar.")
            return
        animate_history(history, new_target)

    btn_solve.on_clicked(on_solve)

    def on_reset(event):
        nonlocal arm, target_display
        arm = SerialArmCCD(LENGTHS, joint_angle_limits=JOINT_LIMITS)
        arm.compute_joints()
        target_display = np.array([s_x.val, s_y.val, s_z.val])
        plot_arm_3d(ax3d, arm.joints, target=target_display)
        info_text.set_text('Reset to initial straight pose.')
        fig.canvas.draw_idle()
        plt.pause(0.01)

    btn_reset.on_clicked(on_reset)

    plt.show()

def animate_history(history, target=None):
    import matplotlib.animation as animation
    if history is None or len(history) == 0:
        print("History vazio — nada para animar.")
        return None

    fig2 = plt.figure(figsize=(7,6))
    ax = fig2.add_subplot(111, projection='3d')
    line, = ax.plot([], [], [], marker='o', linewidth=2)
    if target is not None:
        ax.scatter([target[0]],[target[1]],[target[2]], marker='x', s=80)

    all_pts = np.vstack(history)
    mins = all_pts.min(axis=0) - 0.5
    maxs = all_pts.max(axis=0) + 0.5
    rng = (maxs - mins).max() / 2.0
    center = (maxs + mins) / 2.0
    ax.set_xlim(center[0]-rng, center[0]+rng)
    ax.set_ylim(center[1]-rng, center[1]+rng)
    ax.set_zlim(center[2]-rng, center[2]+rng)

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        return (line,)

    def update(i):
        joints = history[i]
        xs, ys, zs = joints[:,0], joints[:,1], joints[:,2]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        return (line,)

    ani = animation.FuncAnimation(fig2, update, frames=len(history), init_func=init, interval=120, blit=False)
    plt.show(block=False)
    plt.pause(0.01)
    return ani

if __name__ == "__main__":
    run_interactive()
