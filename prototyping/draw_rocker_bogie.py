import sys
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.transforms as mtransforms

# ------ RBM Inputs ------
WHEEL_RADIUS = 50  # The wheel radius [mm]
WHEEL_BASE = 250  # Wheel to Wheel distance along the Robot Y-axis [mm]
# The back to front wheel distance along the Robot X-axis [mm]
TRACK_WIDTH = 350
IMAGE_FILE = "rocker_bogie_diagram.png"
# ------------------------

# Wheel locations for top-down view: WHEEL_BASE is Y-axis (left-right), TRACK_WIDTH is X-axis (front-back)
WHEEL_LOCATIONS = {
    "front_left": (TRACK_WIDTH / 2, WHEEL_BASE / 2),      # Front left
    "middle_left": (0, WHEEL_BASE / 2),                   # Middle left
    "rear_left": (-TRACK_WIDTH / 2, WHEEL_BASE / 2),      # Rear left
    "front_right": (TRACK_WIDTH / 2, -WHEEL_BASE / 2),    # Front right
    "middle_right": (0, -WHEEL_BASE / 2),                 # Middle right
    "rear_right": (-TRACK_WIDTH / 2, -WHEEL_BASE / 2)     # Rear right
}


ROBOT_DIAGRAM = r"""
       B
    /  |  \
   M   |   \
  /  \ |    \
A------N------C
(back)      (front)
"""

"""
A, N, C are wheel locations
B is center axle of Rocker Bogie Mechanism (RBM)
M is the rotation center of the back rocker assembly
WHEEL_BASE is A to C
Triangle BNC is a 45-45-90 triangle
NC == BN, BC = sqrt(2) * NC
AM == MN == BM, where M is the center of the back rocker assembly
AM^2 + MN^2 = AN^2
AM^2 + AM^2 = AN^2
2 * AM^2 = AN^2
AM^2 = 0.5 * AN^2
AM = sqrt(0.5) * AN

BN^2 + NC^2 = BC^2
BN is the RBM height
BN = sqrt(BC^2 - NC^2)
"""

WHEEL_BASE = TRACK_WIDTH - WHEEL_RADIUS * 2  # [mm]
NC = WHEEL_BASE / 2  # [mm]
BC = sqrt(2) * NC  # [mm]
BM = sqrt(0.5) * NC  # [mm]
AM = BM  # [mm]
MN = BM  # [mm]
BN = sqrt(BC**2 - NC**2)  # [mm]
CENTER_TO_GROUND = WHEEL_RADIUS + BN  # [mm]

print(
    f'Rover Rocker Bogie Mechanism (RBM) parameters:\n'
    f'----Robot Inputs----\n'
    f'- Robot Length: {TRACK_WIDTH} [mm]\n'
    f'- Wheel Diameter: {WHEEL_RADIUS * 2} [mm]\n'
    f'--------------------\n'
    f'- Wheel Base (A to C): {WHEEL_BASE:.3f} [mm]\n'
    f'- Center to Front Wheel (B to C): {BC:.3f} [mm]\n'
    f'- Center to Back Pivot (B to M): {BM:.3f} [mm]\n'
    f'- Back Pivot to Middle Wheel (M to N): {MN:.3f} [mm]\n'
    f'- Middle Wheel to Back Wheel (A to N): {NC:.3f} [mm]\n'
    f'- Middle Wheel to Front Wheel (N to C): {NC:.3f} [mm]\n'
    f'- Height of RBM (B to N): {BN:.3f} [mm]\n'
    f'- Center to Ground: {CENTER_TO_GROUND:.3f} [mm]\n'
    f'Rover Diagram:\n{ROBOT_DIAGRAM}'
)

# Print the equations from the inputs: Robot Length, Wheel Diameter
print(
    f'Equations with Inputs [Robot Length, Wheel Diameter] = [{TRACK_WIDTH}, {WHEEL_RADIUS * 2}] mm:\n'
    f'- Wheel Base (A to C) = RobotLength - 2 * WheelRadius\n'
    f'- Center to Front Wheel (B to C) = sqrt(2) * NC\n'
    f'- Center to Back Pivot (B to M) = sqrt(0.5) * NC\n'
    f'- Height of RBM (B to N) = sqrt(BC^2 - NC^2)\n'
    f'- Center to Ground = WheelRadius + BN'
)


def wheel_positions(x=0, y=0, theta=0):
    # Given the position of the robot's center (x, y) and orientation theta,
    # return a dictionary of the wheel positions in the robot's frame indexed by name
    positions = {}
    for name, (dx, dy) in WHEEL_LOCATIONS.items():
        # Apply rotation to wheel positions
        rotated_x = (dx * np.cos(theta) - dy * np.sin(theta)) + x
        rotated_y = (dx * np.sin(theta) + dy * np.cos(theta)) + y
        positions[name] = (rotated_x, rotated_y)
    return positions


def draw_robot_diagram():
    """
    Draw both top-down view and side view of the 6-wheeled rocker-bogie robot
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 9))

    # ===== LEFT SUBPLOT: TOP-DOWN VIEW =====
    ax = ax1

    # Draw coordinate axes (centered at robot center)
    ax.arrow(0, 0, 200, 0, head_width=30, head_length=40,
             fc='red', ec='red', linewidth=2)
    ax.text(220, -30, 'X (forward)', fontsize=12, color='red', weight='bold')

    ax.arrow(0, 0, 0, 200, head_width=30, head_length=40,
             fc='green', ec='green', linewidth=2)
    ax.text(-80, 220, 'Y (left)', fontsize=12, color='green', weight='bold')

    # Draw robot body outline (simplified rectangular body)
    body_width = WHEEL_BASE + 100  # Add some margin for visual clarity
    body_length = TRACK_WIDTH + 100
    body_rect = plt.Rectangle((-body_length/2, -body_width/2), body_length, body_width,
                              fill=False, edgecolor='black', linewidth=2, linestyle='--', alpha=0.5)
    ax.add_patch(body_rect)

    # Draw wheels at their positions (as rectangles aligned with X-axis)
    wheel_width = 20   # Wheel width (along Y-axis)
    wheel_length = 60  # Wheel length (along X-axis)
    # Different colors for left/right sides
    left_colors = ['blue', 'blue', 'blue']   # All left wheels blue
    right_colors = ['red', 'red', 'red']     # All right wheels red

    left_idx = 0
    right_idx = 0

    for i, (name, (x, y)) in enumerate(WHEEL_LOCATIONS.items()):
        # Choose color based on left/right side
        if 'left' in name:
            color = left_colors[left_idx]
            left_idx += 1
        else:
            color = right_colors[right_idx]
            right_idx += 1

        # Draw wheel as a rectangle aligned with X-axis
        wheel = plt.Rectangle((x - wheel_length/2, y - wheel_width/2),
                              wheel_length, wheel_width,
                              color=color, alpha=0.7, ec='black', linewidth=1.5)
        ax.add_patch(wheel)

        # Add wheel labels
        label_offset = 60
        if 'left' in name:
            ax.text(x + label_offset, y, name.replace('_', '\n'), fontsize=10,
                    ha='left', va='center', weight='bold')
        else:
            ax.text(x - label_offset, y, name.replace('_', '\n'), fontsize=10,
                    ha='right', va='center', weight='bold')

        # Draw position coordinates
        ax.plot([0, x], [0, y], 'k--', alpha=0.3, linewidth=1)
        coord_text = f'({x:.0f}, {y:.0f})'
        if 'left' in name:
            ax.text(x + label_offset, y - 30, coord_text, fontsize=8,
                    ha='left', va='center', style='italic', alpha=0.7)
        else:
            ax.text(x - label_offset, y + 30, coord_text, fontsize=8,
                    ha='right', va='center', style='italic', alpha=0.7)

    # Mark the robot center
    ax.plot(0, 0, 'ko', markersize=8)
    ax.text(20, 20, 'Robot Center\n(0, 0)', fontsize=10, weight='bold')

    # Add dimension annotations
    # TRACK_WIDTH annotation (X-axis: front to back)
    ax.annotate('', xy=(-TRACK_WIDTH/2, -body_width/2 - 80), xytext=(TRACK_WIDTH/2, -body_width/2 - 80),
                arrowprops=dict(arrowstyle='<->', color='purple', lw=2))
    ax.text(0, -body_width/2 - 125, f'TRACK_WIDTH = {TRACK_WIDTH} mm\n(Front to Back)',
            ha='center', fontsize=11, color='purple', weight='bold')

    # WHEEL_BASE annotation (Y-axis: left to right) - placed on the left side
    ax.annotate('', xy=(-body_length/2 - 80, -WHEEL_BASE/2), xytext=(-body_length/2 - 80, WHEEL_BASE/2),
                arrowprops=dict(arrowstyle='<->', color='orange', lw=2))
    ax.text(-body_length/2 - 120, 0, f'WHEEL_BASE = {WHEEL_BASE} mm\n(Left to Right)',
            rotation=90, ha='center', va='center', fontsize=11, color='orange', weight='bold')

    # Set equal aspect ratio and limits
    ax.set_aspect('equal')
    margin = 300
    ax.set_xlim(-TRACK_WIDTH/2 - margin, TRACK_WIDTH/2 + margin)
    ax.set_ylim(-WHEEL_BASE/2 - margin, WHEEL_BASE/2 + margin)

    # Grid and labels
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position [mm]', fontsize=12)
    ax.set_ylabel('Y Position [mm]', fontsize=12)
    ax.set_title('Top View - Robot Body Frame',
                 fontsize=14, weight='bold', pad=15)

    # Add legend
    legend_elements = [
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='blue', markersize=10,
                   label='Left Wheels', alpha=0.7),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10,
                   label='Right Wheels', alpha=0.7),
        plt.Line2D([0], [0], color='red', linewidth=2,
                   label='X-axis (forward)'),
        plt.Line2D([0], [0], color='green', linewidth=2, label='Y-axis (left)')
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)

    # ===== RIGHT SUBPLOT: SIDE VIEW (ROCKER-BOGIE MECHANISM) =====
    ax = ax2

    # Calculate positions for side view
    # Ground level (where wheels touch)
    ground_y = 0

    # Wheel positions (side view - only showing X positions)
    A_x = -NC  # Back wheel
    A_y = ground_y + WHEEL_RADIUS

    N_x = 0    # Middle wheel
    N_y = ground_y + WHEEL_RADIUS

    C_x = NC   # Front wheel
    C_y = ground_y + WHEEL_RADIUS

    # Center axle position
    B_x = 0
    B_y = A_y + BN

    # Back pivot position
    M_x = -BM * 0.7  # Adjusted for visual clarity
    M_y = A_y + BM * 0.7

    # Draw ground line
    ax.axhline(y=ground_y, color='brown',
               linewidth=3, alpha=0.7, label='Ground')

    # Draw wheels as circles
    wheel_radius_plot = WHEEL_RADIUS * 0.8  # Scale down for better visualization

    # Back wheel (A)
    wheel_A = plt.Circle((A_x, A_y), wheel_radius_plot,
                         color='gray', alpha=0.7, ec='black', linewidth=2)
    ax.add_patch(wheel_A)
    ax.text(A_x, A_y - wheel_radius_plot - 30, 'A\n(Back)', ha='center',
            va='top', fontsize=12, weight='bold', color='red')

    # Middle wheel (N)
    wheel_N = plt.Circle((N_x, N_y), wheel_radius_plot,
                         color='gray', alpha=0.7, ec='black', linewidth=2)
    ax.add_patch(wheel_N)
    ax.text(N_x, N_y - wheel_radius_plot - 30, 'N\n(Middle)',
            ha='center', va='top', fontsize=12, weight='bold', color='blue')

    # Front wheel (C)
    wheel_C = plt.Circle((C_x, C_y), wheel_radius_plot,
                         color='gray', alpha=0.7, ec='black', linewidth=2)
    ax.add_patch(wheel_C)
    ax.text(C_x, C_y - wheel_radius_plot - 30, 'C\n(Front)',
            ha='center', va='top', fontsize=12, weight='bold', color='blue')

    # Draw RBM structure
    # Front rocker assembly (blue - only B to C connection)
    ax.plot([B_x, C_x], [B_y, C_y], 'b-', linewidth=4,
            alpha=0.8, label='Front Rocker')

    # Back rocker assembly (red)
    ax.plot([A_x, M_x], [A_y, M_y], 'r-', linewidth=4,
            alpha=0.8, label='Back Rocker')
    ax.plot([M_x, N_x], [M_y, N_y], 'r-', linewidth=4, alpha=0.8)
    ax.plot([B_x, M_x], [B_y, M_y], 'r-', linewidth=4, alpha=0.8)

    # B to N measurement line (dashed - not a physical connection)
    ax.plot([B_x, N_x], [B_y, N_y], 'k--', linewidth=2,
            alpha=0.7, label='Height Measurement (B-N)')

    # Draw pivot points
    ax.plot(B_x, B_y, 'ro', markersize=10, label='Center Pivot (B)')
    ax.text(B_x + 20, B_y, 'B\n(Center)', ha='left',
            va='center', fontsize=12, weight='bold', color='red')

    ax.plot(M_x, M_y, 'o', color='orange',
            markersize=8, label='Back Pivot (M)')
    ax.text(M_x - 30, M_y, 'M\n(Pivot)', ha='right', va='center',
            fontsize=12, weight='bold', color='orange')

    # Add dimension annotations
    ax.annotate('', xy=(A_x, A_y + wheel_radius_plot + 80), xytext=(C_x, C_y + wheel_radius_plot + 80),
                arrowprops=dict(arrowstyle='<->', color='green', lw=2))
    ax.text((A_x + C_x)/2, A_y + wheel_radius_plot + 100, f'Track Width = {TRACK_WIDTH:.1f} mm',
            ha='center', fontsize=11, color='green', weight='bold')

    # Height (BN)
    ax.annotate('', xy=(B_x + 120, B_y), xytext=(N_x + 120, N_y),
                arrowprops=dict(arrowstyle='<->', color='purple', lw=2))
    ax.text(B_x + 140, (B_y + N_y)/2, f'Height = {BN:.1f} mm',
            rotation=90, ha='center', va='center', fontsize=11, color='purple', weight='bold')

    # Center to ground
    ax.annotate('', xy=(B_x - 120, ground_y), xytext=(B_x - 120, B_y),
                arrowprops=dict(arrowstyle='<->', color='navy', lw=2))
    ax.text(B_x - 140, (ground_y + B_y)/2, f'Center to Ground = {CENTER_TO_GROUND:.1f} mm',
            rotation=90, ha='center', va='center', fontsize=11, color='navy', weight='bold')

    # Set equal aspect ratio and limits
    ax.set_aspect('equal')
    margin = 100
    ax.set_xlim(A_x - margin, C_x + margin)
    ax.set_ylim(ground_y - 50, B_y + margin)

    # Grid and labels
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Position [mm]', fontsize=12)
    ax.set_ylabel('Z Position [mm]', fontsize=12)
    ax.set_title('Side View - Rocker-Bogie Mechanism',
                 fontsize=14, weight='bold', pad=15)

    # Add legend
    ax.legend(loc='upper right', fontsize=10)

    # Add overall title
    fig.suptitle('6-Wheeled Rocker-Bogie Robot Diagram',
                 fontsize=16, weight='bold', y=0.95)

    plt.tight_layout()
    plt.savefig(IMAGE_FILE, dpi=300, bbox_inches='tight')
    plt.show()


def draw_rocker_bogie_at_state(ax, show_text: bool, x, y, theta, vx, vy, omega):
    # Draw the robot chassis as a grey rectangle
    CHASSIS_LENGTH = TRACK_WIDTH * 1.5  # Length of the chassis
    CHASSIS_WIDTH = WHEEL_BASE * 0.7  # Width of the chassis
    chassis = plt.Rectangle(
        (x - CHASSIS_LENGTH / 2, y - CHASSIS_WIDTH / 2),
        CHASSIS_LENGTH, CHASSIS_WIDTH,
        angle=0, color='lightgrey', alpha=0.8, ec='black', linewidth=2
    )
    t = ax.transData
    rotate_transform = mtransforms.Affine2D().rotate_around(x, y, theta) + t
    chassis.set_transform(rotate_transform)
    ax.add_patch(chassis)

    # Draw the wheels as rectangles
    WHEEL_LENGTH = WHEEL_RADIUS * 2  # Length of the wheel
    WHEEL_WIDTH = WHEEL_RADIUS * 0.5  # Width of the wheel
    positions = wheel_positions(x, y, theta)
    for name, (wx, wy) in positions.items():
        wheel = plt.Rectangle(
            (wx - WHEEL_LENGTH / 2, wy - WHEEL_WIDTH / 2),
            WHEEL_LENGTH, WHEEL_WIDTH,
            angle=0, color='darkgrey', ec='black', linewidth=1.5
        )
        wheel_t = ax.transData
        wheel_rotate_transform = mtransforms.Affine2D().rotate_around(
            wx, wy, theta) + wheel_t
        wheel.set_transform(wheel_rotate_transform)
        ax.add_patch(wheel)

    # Draw the heading of the robot as a larger triangle at the front
    front_x = x + (CHASSIS_LENGTH / 2) * np.cos(theta)
    front_y = y + (CHASSIS_LENGTH / 2) * np.sin(theta)
    HEADING_TRIANGLE_SIZE = 40  # Size of the heading triangle
    heading_triangle = plt.Polygon(
        [
            (front_x, front_y),
            (front_x - HEADING_TRIANGLE_SIZE * np.cos(theta + np.pi / 6),
             front_y - HEADING_TRIANGLE_SIZE * np.sin(theta + np.pi / 6)),
            (front_x - HEADING_TRIANGLE_SIZE * np.cos(theta - np.pi / 6),
             front_y - HEADING_TRIANGLE_SIZE * np.sin(theta - np.pi / 6))
        ],
        closed=True, color='red', alpha=0.8, ec='black', linewidth=1.5
    )
    ax.add_patch(heading_triangle)

    # Draw a black circle at the center of the chassis
    ax.plot(x, y, 'ko', markersize=12)

    # --- Velocity Vector Visualization ---
    # The velocity (vx, vy) is in the robot's body frame.
    # We need to rotate it by theta to align it with the world frame.
    world_vx = vx * np.cos(theta) - vy * np.sin(theta)
    world_vy = vx * np.sin(theta) + vy * np.cos(theta)

    # Calculate magnitude and scale the arrow length
    magnitude = np.sqrt(world_vx**2 + world_vy**2)
    if magnitude > 1e-6:  # Avoid division by zero
        ARROW_LENGTH = (CHASSIS_LENGTH / 2) - (HEADING_TRIANGLE_SIZE * 1.2)

        # Calculate the final arrow components for plotting
        arrow_dx = (world_vx / magnitude) * ARROW_LENGTH
        arrow_dy = (world_vy / magnitude) * ARROW_LENGTH

        # Draw the velocity vector as an arrow
        ax.arrow(
            x, y,
            arrow_dx, arrow_dy,
            head_width=25, head_length=35, fc='blue', ec='blue', linewidth=2,
            length_includes_head=True
        )

    # Add a small label for the robot state above the chassis center
    if show_text:
        ax.text(
            x, y + (CHASSIS_LENGTH / 4),
            f'X=[x={x:.0f} mm, y={y:.0f} mm, θ={np.degrees(theta):.1f}°]\n'
            f'V=[vx={vx:.1f} mm/s, vy={vy:.1f} mm/s, ω={omega:.1f} rad/s]',
            fontsize=8, ha='center', va='bottom', color='black', weight='bold',
            bbox=dict(facecolor='white', edgecolor='red', linewidth=1,
                      boxstyle='round,pad=0.3', alpha=0.8)
        )


if __name__ == "__main__":
    draw_robot_diagram()

    # Create a new figure for trajectory visualization
    fig, ax = plt.subplots(figsize=(10, 10))
    MAX_SIZE = 1000
    ax.set_xlim(-MAX_SIZE, MAX_SIZE)
    ax.set_ylim(-MAX_SIZE, MAX_SIZE)
    test_robot_positions = [
        (0, 0, 0, 10, 0, 0),  # Centered at origin, no rotation
        # Diagonal position with 45-degree rotation
        (500, 500, np.pi/4, 10, 0, 0),
        (-300, -450, np.pi/2, 10, 0, 0),  # Left side with 90-degree rotation
        (300, -500, -np.pi/3, 10, 0, 0)   # Right side with -60-degree rotation
    ]
    for pos in test_robot_positions:
        draw_rocker_bogie_at_state(ax, True, *pos)
        plt.title('Top-Down View of Rocker-Bogie Robot in different states')
        plt.xlabel('X Position [mm]')
        plt.ylabel('Y Position [mm]')
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
    plt.savefig(f"top_down_rover.png",
                dpi=300, bbox_inches='tight')
