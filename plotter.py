import numpy as np
import matplotlib.pyplot as plt
import cv2
from mpl_toolkits.mplot3d import Axes3D
from utils import *



def combineFrame(sat_image, gt, ins, particles, min_w=256):
    """
    Crop a square region from an image, centered at the ground-truth UAV pixel location,
    that covers the ground truth, INS dead reckoning (if provided),
    and all particle positions (if provided).

    'particles' can have shape:
      - (N, 2) -> no weights, each row is [x, y].
      - (N, 3) or more -> [x, y, weight, ...].
    
    If 'particles' is None or empty, no particle drawing is done.
    
    Parameters:
    -----------
        sat_image (np.ndarray): The input image (grayscale or color),
                                shape (H, W) or (H, W, C).
        gt (tuple or list): Ground-truth pixel (x, y).
        ins (tuple or list or None): INS dead reckoning pixel (x, y) or None.
        particles (np.ndarray, list, or None): 
            - shape (N,2): each row is (px, py), or
            - shape (N,3): each row is (px, py, weight)
            - None or empty => skipped
        min_w (int): The width of the final cropped (square) image.

    Returns:
    --------
        np.ndarray:
            The cropped square image with markers for:
              - GT in green
              - INS in blue (if provided)
              - Particles in grey (if no weights) or
                colormap (if weights are given).
    """

    # 1. Extract ground-truth coordinates.
    gx, gy = float(gt[0]), float(gt[1])

    # 2. If INS is provided, compute difference from GT to define crop area.
    if ins is not None:
        dx_ins = abs(ins[0] - gx)
        dy_ins = abs(ins[1] - gy)
    else:
        dx_ins = 0
        dy_ins = 0

    # 3. Process particles if provided; otherwise, set them to None.
    if particles is None or len(particles) == 0:
        max_dx = dx_ins
        max_dy = dy_ins
        particles_arr = None
    else:
        # Ensure 'particles' is a numpy array.
        particles_arr = np.array(particles)
        # For bounding box purposes, we only need x and y columns:
        px = particles_arr[:, 0]
        py = particles_arr[:, 1]

        dx_particles = np.abs(px - gx)
        dy_particles = np.abs(py - gy)

        max_dx = max(dx_ins, dx_particles.max())
        max_dy = max(dy_ins, dy_particles.max())

    # 4. Determine the half-side length for the square crop.
    half_side = int(np.ceil(max(max_dx, max_dy))) + (min_w // 4)

    # 5. Compute the crop boundaries (centered at GT).
    left   = int(round(gx)) - half_side
    right  = int(round(gx)) + half_side
    top    = int(round(gy)) - half_side
    bottom = int(round(gy)) + half_side

    # Extend right and bottom by 1 to include boundary pixels.
    right  += 1
    bottom += 1

    # 6. Clip the boundaries to the image dimensions.
    H, W = sat_image.shape[:2]
    left   = max(0, left)
    top    = max(0, top)
    right  = min(W, right)
    bottom = min(H, bottom)

    # 7. Crop the image.
    cropped_img = sat_image[top:bottom, left:right].copy()

    # If the image is grayscale, convert it to BGR for drawing colored markers.
    if len(cropped_img.shape) == 2:
        cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_GRAY2BGR)

    # 8. Shift ground-truth coords to cropped image coords; draw GT (green).
    gt_c = (int(gx - left), int(gy - top))
    cv2.circle(cropped_img, gt_c, 10, (0, 255, 0), -1)  # BGR => green

    # 9. If INS is provided, shift coords and draw in blue.
    if ins is not None:
        ins_c = (int(ins[0] - left), int(ins[1] - top))
        cv2.circle(cropped_img, ins_c, 10, (255, 0, 0), -1)  # BGR => blue

    # 10. If particles exist, shift coords and draw them.
    if particles_arr is not None:
        # We check how many columns the array has.
        # If >= 3, we treat the third column as weights; else just (x,y).
        num_cols = particles_arr.shape[1]

        if num_cols >= 3:
            # Particles have weights
            px = particles_arr[:, 0]
            py = particles_arr[:, 1]
            w  = particles_arr[:, 2]

            # Find min & max weight
            w_min = w.min()
            w_max = w.max()
            w_range = max(w_max - w_min, 1e-9)  # avoid div by zero

            # define color extremes for the blue channel:
            #   high weight => darker (smaller B value)
            #   low weight => lighter (larger B value)
            B_light = 255  # light-blue
            B_dark  = 50   # dark-blue

            for (px_i, py_i, w_i) in particles_arr:
                # map weight to [B_dark..B_light]
                w_norm = (w_i - w_min) / w_range
                B_val = int(B_light - (B_light - B_dark) * w_norm)

                p_c = (int(px_i - left), int(py_i - top))
                cv2.circle(cropped_img, p_c, 5, (B_val, 0, 0), -1)

        else:
            # No weights => Nx2 array, draw in gray
            for (px_i, py_i) in particles_arr:
                p_c = (int(px_i - left), int(py_i - top))
                cv2.circle(cropped_img, p_c, 5, (128, 128, 128), -1)  # gray

    # 11. Resize the cropped image to (min_w, min_w).
    cropped_img = cv2.resize(cropped_img, (min_w, min_w))

    return cropped_img

    
    


def plot_positions(
    ground_truth=None, 
    ins_positions=None, 
    estimated_position=None, 
    particles_positions=None, 
    plot_2d=False
):
    """
    Plots:
      1) Ground Truth positions (optional)
      2) INS positions (optional)
      3) Estimated positions (optional)
      4) Particle positions (optional), either a list of shape (N, 2) or (N, 3) 
         for each timestep.

    Also computes and displays the RMSE for:
      - INS vs Ground Truth
      - Estimated vs Ground Truth

    Only computes RMSE if *both* the reference (ground_truth) and the 
    comparison set (INS or estimated) are provided.

    The function can plot either 2D or 3D, based on 'plot_2d'. 
    - If 2D, uses (x, y), ignoring the z-coordinate if provided.
    - If 3D, uses (x, y, z).
    """

    # ------------------------------
    #  Helper to convert Nx3 -> Nx2
    # ------------------------------
    def to_2d(positions):
        """Return a list of (x, y) from (x, y) or (x, y, z)."""
        if positions is None:
            return None
        return [(p[0], p[1]) for p in positions]

    # ------------------------------
    #  RMSE helper
    # ------------------------------
    def compute_rmse(gt, est):
        """
        Compute MSE between two lists of positions (2D or 3D).
        Each is a list/tuple of length 2 or 3. 
        Returns float RMSE.
        """
        # Ensure same length
        if len(gt) != len(est):
            raise ValueError("Ground truth and estimate must have same length to compute RMSE.")
        
        # Distances squared
        sq_diffs = []
        for g, e in zip(gt, est):
            if len(g) == 2:
                # 2D
                sq_diffs.append((g[0] - e[0])**2 + (g[1] - e[1])**2)
            else:
                # 3D
                sq_diffs.append((g[0] - e[0])**2 + (g[1] - e[1])**2 + (g[2] - e[2])**2)
        mse = np.mean(sq_diffs)
        return np.sqrt(mse)

    # ------------------------------
    #  Convert data as needed
    # ------------------------------
    # If 2D plot, reduce all data to 2D
    if plot_2d:
        ground_truth_2d     = to_2d(ground_truth)
        ins_positions_2d    = to_2d(ins_positions)
        estimated_position_2d = to_2d(estimated_position)
        # For particles, each entry in the list is an array of shape (N,2 or 3)
        # We convert each to shape (N,2) if 2D
        if particles_positions is not None:
            particles_2d = []
            for arr in particles_positions:
                if arr.shape[1] == 3:
                    particles_2d.append(arr[:, :2])  # keep only first 2 columns
                else:
                    particles_2d.append(arr)
        else:
            particles_2d = None

        # -----------------------
        # Create figure & axis
        # -----------------------
        fig, ax = plt.subplots(figsize=(10, 6))

        # -----------------------
        # Plot each data set if available
        # -----------------------
        # We'll store X/Y for axis scaling
        combined_x = []
        combined_y = []

        # Ground truth
        if ground_truth_2d is not None:
            gt_x, gt_y = zip(*ground_truth_2d)
            ax.plot(gt_x, gt_y, label="Ground Truth", linestyle="--", marker=".", color="blue")
            combined_x += gt_x
            combined_y += gt_y
        else:
            gt_x = gt_y = None

        # INS
        if ins_positions_2d is not None:
            ins_x, ins_y = zip(*ins_positions_2d)
            ax.plot(ins_x, ins_y, label="INS Positions", linestyle="-", marker="x", color="red")
            combined_x += ins_x
            combined_y += ins_y

        # Estimated
        if estimated_position_2d is not None:
            est_x, est_y = zip(*estimated_position_2d)
            ax.plot(est_x, est_y, label="Estimated Positions", linestyle="-", marker="o", color="green")
            combined_x += est_x
            combined_y += est_y

        # Particles
        if particles_2d is not None:
            for arr in particles_2d:
                px = arr[:, 0]
                py = arr[:, 1]
                ax.scatter(px, py, color="gray", marker=".", s=6, alpha=0.7)
                combined_x += px.tolist()
                combined_y += py.tolist()

        # -----------------------
        #  Compute & display RMSE
        # -----------------------
        text_y = 0.95  # start from top of the axis
        if ground_truth_2d is not None:
            # INS vs GT
            if ins_positions_2d is not None:
                rmse_ins = compute_rmse(ground_truth_2d, ins_positions_2d)
                ax.text(0.05, text_y, f"RMSE (INS vs GT): {rmse_ins:.4f}", 
                        transform=ax.transAxes, fontsize=10, color="red")
                text_y -= 0.05
            # Estimated vs GT
            if estimated_position_2d is not None:
                rmse_est = compute_rmse(ground_truth_2d, estimated_position_2d)
                ax.text(0.05, text_y, f"RMSE (Estimated vs GT): {rmse_est:.4f}", 
                        transform=ax.transAxes, fontsize=10, color="green")
                text_y -= 0.05
        # If ground_truth_2d is None, we skip RMSE altogether

        # -----------------------
        #  Final axis formatting
        # -----------------------
        ax.set_xlabel("North Position(m)")
        ax.set_ylabel("East Position(m)")        
        ax.legend()
        ax.grid(True)

        if combined_x and combined_y:  # if not empty
            min_x, max_x = min(combined_x), max(combined_x)
            min_y, max_y = min(combined_y), max(combined_y)
            range_x = max_x - min_x
            range_y = max_y - min_y
            max_range = max(range_x, range_y) / 2.0
            mid_x = (max_x + min_x) / 2.0
            mid_y = (max_y + min_y) / 2.0

            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_ylim(ax.get_ylim()[::-1])

    else:
        # -----------------------
        # 3D case
        # -----------------------
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111, projection='3d')

        combined_x = []
        combined_y = []
        combined_z = []

        # Ground truth
        if ground_truth is not None:
            gt_x, gt_y, gt_z = zip(*ground_truth)
            ax.plot(gt_x, gt_y, gt_z, label="Ground Truth", 
                    linestyle="--", marker=".", color="blue")
            combined_x += gt_x
            combined_y += gt_y
            combined_z += gt_z
        else:
            gt_x = gt_y = gt_z = None

        # INS
        if ins_positions is not None:
            ins_x, ins_y, ins_z = zip(*ins_positions)
            ax.plot(ins_x, ins_y, ins_z, label="INS Positions",
                    linestyle="-", marker="x", color="red")
            combined_x += ins_x
            combined_y += ins_y
            combined_z += ins_z

        # Estimated
        if estimated_position is not None:
            est_x, est_y, est_z = zip(*estimated_position)
            ax.plot(est_x, est_y, est_z, label="Estimated Positions",
                    linestyle="-", marker="o", color="green")
            combined_x += est_x
            combined_y += est_y
            combined_z += est_z

        # Particles
        if particles_positions is not None:
            for arr in particles_positions:
                px = arr[:, 0]
                py = arr[:, 1]
                pz = arr[:, 2] if arr.shape[1] == 3 else np.zeros(len(arr))  # in case of shape (N,2)
                ax.scatter(px, py, pz, color="gray", marker=".", s=6, alpha=0.7)
                combined_x += px.tolist()
                combined_y += py.tolist()
                combined_z += pz.tolist()

        # RMSE text in 3D
        text_y = 0.95
        if ground_truth is not None:
            # INS vs GT
            if ins_positions is not None:
                rmse_ins = compute_rmse(ground_truth, ins_positions)
                ax.text2D(0.05, text_y, f"RMSE (INS vs GT): {rmse_ins:.4f}",
                          transform=ax.transAxes, fontsize=10, color="red")
                text_y -= 0.05
            # Estimated vs GT
            if estimated_position is not None:
                rmse_est = compute_rmse(ground_truth, estimated_position)
                ax.text2D(0.05, text_y, f"RMSE (Estimated vs GT): {rmse_est:.4f}",
                          transform=ax.transAxes, fontsize=10, color="green")
                text_y -= 0.05

        # Axis labels, legend, grid
        ax.set_title("3D Plot")
        ax.set_xlabel("North Position(m)")
        ax.set_ylabel("East Position(m)")
        ax.set_zlabel("Down Position(m)")
        ax.legend()

        ax.grid(True)

        if combined_x and combined_y and combined_z:  # not empty
            max_range = max(
                max(combined_x) - min(combined_x),
                max(combined_y) - min(combined_y),
                max(combined_z) - min(combined_z)
            ) / 2.0

            mid_x = (max(combined_x) + min(combined_x)) / 2.0
            mid_y = (max(combined_y) + min(combined_y)) / 2.0
            mid_z = (max(combined_z) + min(combined_z)) / 2.0

            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
            ax.set_ylim(ax.get_ylim()[::-1])

    plt.show(block=True)

        
class PlotCamera:
    def __init__(self, useFramePlotter=True):
        """
        Initializes the PlotCamera.

        Parameters:
            useFramePlotter (int or bool): If truthy, interactive plot updating is enabled.
        """
        self.useFramePlotter = useFramePlotter
        self.fig = None            # Will hold the matplotlib figure
        self.axes = None           # Will hold the subplot axes as a list
        self.imgs = None           # Will hold the image handles (returned by imshow)
        self.bottom_texts = None   # Will hold references to text objects displayed below each subplot

    def snapNow(self, *frames):
        """
        Update and display the provided frames side by side.

        Each frame can be:
            - A single numpy array (data).
            - A 2-element tuple (data, title).
            - A 3-element tuple (data, title, bottom_text).

        If a title is provided, it is displayed above the image (subplot title).
        If bottom_text is provided, it is displayed below the image.

        Parameters:
            *frames: A variable number of frames in the above formats.
                     The order is preserved.
        """
        # Do nothing if useFramePlotter is disabled or if no frames are provided.
        if not self.useFramePlotter or len(frames) == 0:
            return

        # A helper function to parse frame into (data, title, bottom_text).
        def parse_frame(f):
            data, title, bottom_text = None, None, None
            if isinstance(f, tuple):
                if len(f) == 1:
                    (data,) = f
                elif len(f) == 2:
                    data, title = f
                elif len(f) == 3:
                    data, title, bottom_text = f
            else:
                # Not a tuple, assume it's just the data array
                data = f
            return data, title, bottom_text

        # Check if we need to create/re-create the figure/axes.
        need_new_figure = (
            self.fig is None or 
            self.axes is None or 
            len(self.axes) != len(frames)
        )

        if need_new_figure:
            plt.ion()  # Turn on interactive mode (if not already on)

            # Create a new figure with as many subplots as there are frames.
            self.fig, self.axes = plt.subplots(1, len(frames), figsize=(4 * len(frames), 4))
            
            # When there's only one subplot, plt.subplots returns a single Axes object.
            if len(frames) == 1:
                self.axes = [self.axes]

            # Initialize each subplot and keep the image handles.
            self.imgs = []
            self.bottom_texts = []

            for i, frame in enumerate(frames):
                data, title, bottom_text = parse_frame(frame)

                # If the frame data is None, use a dummy image.
                if data is None:
                    data = cv2.imread('data/not_available.jpg', cv2.IMREAD_GRAYSCALE)

                # Show the image
                img = self.axes[i].imshow(data, cmap='gray', vmin=0, vmax=255)
                
                # Set title if given
                if title is not None:
                    self.axes[i].set_title(title)
                
                # Remove axis ticks for a cleaner look
                self.axes[i].axis('off') 
                self.imgs.append(img)

                # Add bottom text if given
                txt_obj = None
                if bottom_text is not None:
                    txt_obj = self.axes[i].text(
                        0.5, -0.12, bottom_text,
                        transform=self.axes[i].transAxes,
                        ha='center', va='top'
                    )
                self.bottom_texts.append(txt_obj)

        else:
            # The figure exists and the number of subplots matches the number of frames.
            # Update only those frames that are provided.
            for i, frame in enumerate(frames):
                data, title, bottom_text = parse_frame(frame)
                # cv2.imshow('d',data)
                # cv2.waitKey(0)

                if data is not None:
                    self.imgs[i].set_data(data)

                # Update the title if provided; if None, leave it as-is
                if title is not None:
                    self.axes[i].set_title(title)

                # Update the bottom text if provided
                if bottom_text is not None:
                    if self.bottom_texts[i] is not None:
                        # Just update existing text
                        self.bottom_texts[i].set_text(bottom_text)
                    else:
                        # Create a new text object
                        self.bottom_texts[i] = self.axes[i].text(
                            0.5, -0.12, bottom_text,
                            transform=self.axes[i].transAxes,
                            ha='center', va='top'
                        )

        # Refresh the figure canvas to show updates.
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
class DynamicErrorPlot:
    """
    A class that maintains a single, persistent 5x3 figure for dynamically
    plotting:
      - Position (N, E, D)
      - Velocity (N, E, D)
      - Euler angles (roll, pitch, yaw) derived from quaternions
      - Accelerometer bias (x, y, z)
      - Gyroscope bias (x, y, z)

    Each of GT, INS, MPF is expected to be a 16-element array:
      [pos_N, pos_E, pos_D,
       vel_N, vel_E, vel_D,
       qx, qy, qz, qw,
       acc_bias_x, acc_bias_y, acc_bias_z,
       gyro_bias_x, gyro_bias_y, gyro_bias_z]

    Usage:
    -------
        dp = DynamicErrorPlot()  # create once

        # repeatedly call update(...) with data for each source:
        dp.update(ground_truth=gt_data, ins=ins_data, mpf=mpf_data)

        # You can pass None for any source if it is not available in a given call.
    """

    def __init__(self):
        # Create figure and axes: 5 rows × 3 columns
        self.fig, self.ax = plt.subplots(5, 3, figsize=(12, 12))
        self.fig.suptitle("Dynamic Plot: Position, Velocity, Euler Angles, and Biases")

        # Data fields to store for each source
        # We are adding the accelerometer bias (bias_ax, bias_ay, bias_az)
        # and gyroscope bias (bias_gx, bias_gy, bias_gz).
        self.data = {
            'gt': {
                'pos_n': [], 'pos_e': [], 'pos_d': [],
                'vel_n': [], 'vel_e': [], 'vel_d': [],
                'roll': [], 'pitch': [], 'yaw': [],
                'bias_ax': [], 'bias_ay': [], 'bias_az': [],
                'bias_gx': [], 'bias_gy': [], 'bias_gz': []
            },
            'ins': {
                'pos_n': [], 'pos_e': [], 'pos_d': [],
                'vel_n': [], 'vel_e': [], 'vel_d': [],
                'roll': [], 'pitch': [], 'yaw': [],
                'bias_ax': [], 'bias_ay': [], 'bias_az': [],
                'bias_gx': [], 'bias_gy': [], 'bias_gz': []
            },
            'mpf': {
                'pos_n': [], 'pos_e': [], 'pos_d': [],
                'vel_n': [], 'vel_e': [], 'vel_d': [],
                'roll': [], 'pitch': [], 'yaw': [],
                'bias_ax': [], 'bias_ay': [], 'bias_az': [],
                'bias_gx': [], 'bias_gy': [], 'bias_gz': []
            }
        }

        # Lines dictionary for 5 rows × 3 columns
        # We'll define the subplots in the same order:
        # 1) pos_n, pos_e, pos_d
        # 2) vel_n, vel_e, vel_d
        # 3) roll, pitch, yaw
        # 4) bias_ax, bias_ay, bias_az
        # 5) bias_gx, bias_gy, bias_gz
        self.lines = {
            'pos_n':  {'gt': None, 'ins': None, 'mpf': None},
            'pos_e':  {'gt': None, 'ins': None, 'mpf': None},
            'pos_d':  {'gt': None, 'ins': None, 'mpf': None},
            'vel_n':  {'gt': None, 'ins': None, 'mpf': None},
            'vel_e':  {'gt': None, 'ins': None, 'mpf': None},
            'vel_d':  {'gt': None, 'ins': None, 'mpf': None},
            'roll':   {'gt': None, 'ins': None, 'mpf': None},
            'pitch':  {'gt': None, 'ins': None, 'mpf': None},
            'yaw':    {'gt': None, 'ins': None, 'mpf': None},
            'bias_ax':{'gt': None, 'ins': None, 'mpf': None},
            'bias_ay':{'gt': None, 'ins': None, 'mpf': None},
            'bias_az':{'gt': None, 'ins': None, 'mpf': None},
            'bias_gx':{'gt': None, 'ins': None, 'mpf': None},
            'bias_gy':{'gt': None, 'ins': None, 'mpf': None},
            'bias_gz':{'gt': None, 'ins': None, 'mpf': None},
        }

        # Subplot layout (5 rows, 3 cols)
        # quantity_order[row][col]
        self.quantity_order = [
            ['pos_n',   'pos_e',   'pos_d'],
            ['vel_n',   'vel_e',   'vel_d'],
            ['roll',    'pitch',   'yaw' ],
            ['bias_ax', 'bias_ay', 'bias_az'],
            ['bias_gx', 'bias_gy', 'bias_gz']
        ]
        
        # Color and label maps for each data source
        self.color_map = {'gt': 'r', 'ins': 'g', 'mpf': 'b'}
        self.label_map = {'gt': 'Ground Truth', 'ins': 'INS', 'mpf': 'MPF'}

        # Initialize the line objects and format each subplot
        # We'll label them based on row
        for r in range(5):
            for c in range(3):
                q_key = self.quantity_order[r][c]
                ax_ = self.ax[r][c]
                
                # Create empty line for each data source
                for src in ['gt', 'ins', 'mpf']:
                    line_obj, = ax_.plot([], [], 
                                         color=self.color_map[src],
                                         label=self.label_map[src])
                    self.lines[q_key][src] = line_obj

                # Subplot title, y-label, etc.
                if r == 0:  # Position row
                    ax_.set_title(f"{q_key.upper()} (Position)")
                    ax_.set_ylabel("Meters")
                elif r == 1:  # Velocity row
                    ax_.set_title(f"{q_key.upper()} (Velocity)")
                    ax_.set_ylabel("m/s")
                elif r == 2:  # Euler row
                    # q_key.capitalize() => "Roll", "Pitch", "Yaw"
                    ax_.set_title(f"{q_key.capitalize()} (Euler)")
                    ax_.set_ylabel("Degrees")
                elif r == 3:  # Accel bias row
                    ax_.set_title(f"{q_key.upper()} (Accel Bias)")
                    ax_.set_ylabel("m/s^2 (Bias)")
                else:  # r == 4 => Gyro bias row
                    ax_.set_title(f"{q_key.upper()} (Gyro Bias)")
                    ax_.set_ylabel("deg/s (Bias)")

                ax_.set_xlabel("time (s)")
                ax_.grid(True)
                ax_.legend(loc="best")

        plt.tight_layout()

    def _store_data(self, src_name, data_array):
        """
        Helper method to store data into the internal dictionary.

        data_array has shape (16,):
            [pos_N, pos_E, pos_D,
             vel_N, vel_E, vel_D,
             qx, qy, qz, qw,
             acc_bias_x, acc_bias_y, acc_bias_z,
             gyro_bias_x, gyro_bias_y, gyro_bias_z]
        """
        # Extract position (0..2), velocity (3..5), quaternion (6..9),
        # accelerometer bias (10..12), gyroscope bias (13..15)
        pos = data_array[0:3]        # [pos_N, pos_E, pos_D]
        vel = data_array[3:6]        # [vel_N, vel_E, vel_D]
        quat = data_array[6:10]      # [qx, qy, qz, qw]
        acc_bias = data_array[10:13] # [bias_ax, bias_ay, bias_az]
        gyro_bias = data_array[13:16]# [bias_gx, bias_gy, bias_gz]

        # Convert quaternion to Euler angles => returns [yaw, pitch, roll] in RADIANS
        yaw, pitch, roll = quat2eul(quat)  # dummy example
        # Convert from rad to deg
        yaw_deg   = yaw   * (180.0 / np.pi)
        pitch_deg = pitch * (180.0 / np.pi)
        roll_deg  = roll  * (180.0 / np.pi)

        # Append to the lists
        self.data[src_name]['pos_n'].append(pos[0])
        self.data[src_name]['pos_e'].append(pos[1])
        self.data[src_name]['pos_d'].append(pos[2])

        self.data[src_name]['vel_n'].append(vel[0])
        self.data[src_name]['vel_e'].append(vel[1])
        self.data[src_name]['vel_d'].append(vel[2])

        self.data[src_name]['roll'].append(roll_deg)
        self.data[src_name]['pitch'].append(pitch_deg)
        self.data[src_name]['yaw'].append(yaw_deg)

        self.data[src_name]['bias_ax'].append(acc_bias[0])
        self.data[src_name]['bias_ay'].append(acc_bias[1])
        self.data[src_name]['bias_az'].append(acc_bias[2])

        self.data[src_name]['bias_gx'].append(gyro_bias[0])
        self.data[src_name]['bias_gy'].append(gyro_bias[1])
        self.data[src_name]['bias_gz'].append(gyro_bias[2])

    def update(self, ground_truth=None, ins=None, mpf=None, timeConstant=1):
        """
        Add new data to the plots. Each of the arguments can be:
          - 16-element NumPy array 
            [posN, posE, posD, velN, velE, velD, qx, qy, qz, qw,
             bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz],
          - or None (to skip updating that source).

        The plot is updated in-place and keeps growing with each call.

        timeConstant: used to scale the horizontal axis.
        """
        # 1) Store new data if provided
        if ground_truth is not None:
            self._store_data('gt', ground_truth)

        if ins is not None:
            self._store_data('ins', ins)

        if mpf is not None:
            self._store_data('mpf', mpf)

        # 2) Update line objects for each quantity
        for r in range(5):
            for c in range(3):
                q_key = self.quantity_order[r][c]
                ax_ = self.ax[r][c]

                for src in ['gt', 'ins', 'mpf']:
                    line_obj = self.lines[q_key][src]
                    data_list = self.data[src][q_key]

                    # If there's no data for this source yet, keep it empty
                    if len(data_list) == 0:
                        line_obj.set_xdata([])
                        line_obj.set_ydata([])
                        continue

                    # x-axis: [0, timeConstant, 2*timeConstant, ...]
                    x_vals = np.linspace(0, len(data_list)*timeConstant, len(data_list))
                    y_vals = data_list

                    line_obj.set_xdata(x_vals)
                    line_obj.set_ydata(y_vals)

                # Adjust the plot limits to accommodate new data
                ax_.relim()
                ax_.autoscale_view()

        # 3) Redraw the figure
        plt.tight_layout()
        plt.draw()
        plt.pause(0.01)
        
class TwoDynamicPlotter:
    """
    A class to dynamically plot VO and GT data for North, East, Down.
    Each input (vo_data, gt_data) is expected to be a 1D NumPy array 
    of shape (3, ), representing [North, East, Down].

    Usage
    -----
    1) Instantiate the class:
        dp = DynamicPlotter()

    2) Whenever new data arrives for VO and GT (as 1D arrays), call:
        dp.update_plots(vo_data, gt_data)

    3) To keep the plot open at the end of your program, call:
        dp.show()
    """
    def __init__(self):
        # Turn on interactive mode for live updates
        plt.ion()

        # Prepare figure and axes (3 subplots for North, East, Down)
        self.fig, self.axes = plt.subplots(1, 3, figsize=(12, 4))
        self.fig.suptitle("Dynamic VO vs GT Plotter", fontsize=16)

        # Set subplot titles and axis labels
        axes_titles = ["North", "East", "Down"]
        for ax, title in zip(self.axes, axes_titles):
            ax.set_title(title)
            ax.set_xlabel("Time Step")
            ax.set_ylabel("Magnitude")
            ax.grid(True)

        # Initialize storage as Python lists
        # We'll store the sequential components of VO & GT over time
        self.vo_n_list = []
        self.vo_e_list = []
        self.vo_d_list = []

        self.gt_n_list = []
        self.gt_e_list = []
        self.gt_d_list = []

        # Create empty line objects for each subplot
        # Subplot 0: North
        self.vo_line_n, = self.axes[0].plot([], [], 'r-', label='VO')
        self.gt_line_n, = self.axes[0].plot([], [], 'b-', label='GT')
        
        # Subplot 1: East
        self.vo_line_e, = self.axes[1].plot([], [], 'r-', label='VO')
        self.gt_line_e, = self.axes[1].plot([], [], 'b-', label='GT')
        
        # Subplot 2: Down
        self.vo_line_d, = self.axes[2].plot([], [], 'r-', label='VO')
        self.gt_line_d, = self.axes[2].plot([], [], 'b-', label='GT')

        # Show legend on each subplot
        for ax in self.axes:
            ax.legend()

        # Draw the initial figure
        plt.draw()
        plt.pause(0.01)

    def update_plots(self, vo_data: np.ndarray, gt_data: np.ndarray):
        """
        Appends new VO and GT data to existing lists and updates the plots.

        Parameters
        ----------
        vo_data : np.ndarray
            1D array of shape (3,) -> [VO_North, VO_East, VO_Down]
        gt_data : np.ndarray
            1D array of shape (3,) -> [GT_North, GT_East, GT_Down]
        """
        # Unpack the 1D NumPy arrays
        vo_n, vo_e, vo_d = vo_data
        gt_n, gt_e, gt_d = gt_data

        # Append new data to the running lists
        self.vo_n_list.append(vo_n)
        self.vo_e_list.append(vo_e)
        self.vo_d_list.append(vo_d)

        self.gt_n_list.append(gt_n)
        self.gt_e_list.append(gt_e)
        self.gt_d_list.append(gt_d)

        # Our "time steps" are just the index of each new data point
        x_vals = np.arange(len(self.vo_n_list))

        # Update line objects for North
        self.vo_line_n.set_xdata(x_vals)
        self.vo_line_n.set_ydata(self.vo_n_list)

        self.gt_line_n.set_xdata(x_vals)
        self.gt_line_n.set_ydata(self.gt_n_list)

        # Update line objects for East
        self.vo_line_e.set_xdata(x_vals)
        self.vo_line_e.set_ydata(self.vo_e_list)

        self.gt_line_e.set_xdata(x_vals)
        self.gt_line_e.set_ydata(self.gt_e_list)

        # Update line objects for Down
        self.vo_line_d.set_xdata(x_vals)
        self.vo_line_d.set_ydata(self.vo_d_list)

        self.gt_line_d.set_xdata(x_vals)
        self.gt_line_d.set_ydata(self.gt_d_list)

        # Rescale axes to display new data properly
        for ax in self.axes:
            ax.relim()           # Recompute limits
            ax.autoscale_view()  # Autoscale

        # Redraw the figure
        plt.draw()
        plt.pause(0.01)

    def show(self):
        """
        Switch off interactive mode and show the figure in a blocking way.
        Useful to keep the plot window open at the end of execution.
        """
        plt.ioff()
        plt.show()