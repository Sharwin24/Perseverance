import sys
import pygame as pg
from math import sqrt
import matplotlib.pyplot as plt

# ------ RBM Inputs ------
WHEEL_RADIUS = 50  # The wheel radius [mm]
WHEEL_BASE = 250  # Wheel to Wheel distance along the Robot Y-axis [mm]
# The back to front wheel distance along the Robot X-axis [mm]
TRACK_WIDTH = 350
IMAGE_FILE = "rocker_bogie_diagram.png"
# ------------------------


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


# Initialize Pygame
pg.init()

# Screen dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pg.display.set_caption('Rover Rocker Bogie Diagram')


def pygame_color_from_hex_code(hex_code):
    """Convert hex color code to Pygame color tuple."""
    hex_code = hex_code.lstrip('#')
    return tuple(int(hex_code[i:i+2], 16) for i in (0, 2, 4))


# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = pygame_color_from_hex_code("#1f6632")
GRAY = (128, 128, 128)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)

# Font for labels
font = pg.font.Font(None, 28)
small_font = pg.font.Font(None, 22)


def draw_robot_diagram():
    """
    Draw a top-down view of the 6-wheeled rocker-bogie robot showing
    wheel positions and coordinate axes.
    """
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

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
    ax.text(0, -body_width/2 - 120, f'TRACK_WIDTH = {TRACK_WIDTH} mm\n(Front to Back)',
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
    ax.set_title('6-Wheeled Rocker-Bogie Robot - Top View\n(Robot Body Frame)',
                 fontsize=14, weight='bold', pad=20)

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

    plt.tight_layout()
    plt.show()

    return fig, ax


def draw_rbm():
    """Draw the Rocker Bogie Mechanism with labels"""
    screen.fill(WHITE)

    # Scale factor to fit on screen (converting from mm to pixels)
    scale = 1.5

    # Center the diagram on screen
    center_x = SCREEN_WIDTH // 2
    center_y = SCREEN_HEIGHT // 2 + 50

    # Calculate positions based on RBM geometry
    # Point coordinates (scaled and centered)
    A_x = center_x - (NC * scale)  # Back wheel
    A_y = center_y

    N_x = center_x  # Middle wheel
    N_y = center_y

    C_x = center_x + (NC * scale)  # Front wheel
    C_y = center_y

    B_x = center_x  # Center axle
    B_y = center_y - (BN * scale)

    # Back pivot (adjusted for visual clarity)
    M_x = center_x - (BM * scale * 0.7)
    M_y = center_y - (BM * scale * 0.7)

    # Draw wheels
    wheel_radius_scaled = WHEEL_RADIUS * scale * 0.5  # Scale down for visual clarity
    pg.draw.circle(screen, BLACK, (int(A_x), int(A_y)),
                   int(wheel_radius_scaled), 3)
    pg.draw.circle(screen, BLACK, (int(N_x), int(N_y)),
                   int(wheel_radius_scaled), 3)
    pg.draw.circle(screen, BLACK, (int(C_x), int(C_y)),
                   int(wheel_radius_scaled), 3)

    # Fill wheels
    pg.draw.circle(screen, GRAY, (int(A_x), int(A_y)),
                   int(wheel_radius_scaled))
    pg.draw.circle(screen, GRAY, (int(N_x), int(N_y)),
                   int(wheel_radius_scaled))
    pg.draw.circle(screen, GRAY, (int(C_x), int(C_y)),
                   int(wheel_radius_scaled))

    # Draw RBM structure lines
    # Main triangle BNC
    pg.draw.line(screen, BLUE, (B_x, B_y), (N_x, N_y), 4)  # B to N
    pg.draw.line(screen, BLUE, (N_x, N_y), (C_x, C_y), 4)  # N to C
    pg.draw.line(screen, BLUE, (B_x, B_y), (C_x, C_y), 4)  # B to C

    # Back rocker assembly
    pg.draw.line(screen, RED, (A_x, A_y), (M_x, M_y), 4)   # A to M
    pg.draw.line(screen, RED, (M_x, M_y), (N_x, N_y), 4)   # M to N
    pg.draw.line(screen, RED, (B_x, B_y), (M_x, M_y), 4)   # B to M

    # Draw pivot points
    pg.draw.circle(screen, RED, (int(B_x), int(B_y)), 6)    # Center pivot B
    pg.draw.circle(screen, ORANGE, (int(M_x), int(M_y)), 6)  # Back pivot M

    # Draw labels for points
    labels = [
        ("A (Back)", A_x - 30, A_y + 30, RED),
        ("N (Middle)", N_x - 30, N_y + 30, BLUE),
        ("C (Front)", C_x + 10, C_y + 30, BLUE),
        ("B (Center)", B_x + 10, B_y - 20, RED),
        ("M (Pivot)", M_x - 40, M_y - 20, ORANGE)
    ]

    for label, x, y, color in labels:
        text = font.render(label, True, color)
        screen.blit(text, (x, y))

    # Draw dimension labels
    # Wheel base
    pg.draw.line(screen, GREEN, (A_x, A_y + 60), (C_x, C_y + 60), 2)
    pg.draw.line(screen, GREEN, (A_x, A_y + 55), (A_x, A_y + 65), 2)
    pg.draw.line(screen, GREEN, (C_x, C_y + 55), (C_x, C_y + 65), 2)
    wb_text = small_font.render(f'Wheel Base: {WHEEL_BASE:.1f}mm', True, GREEN)
    screen.blit(wb_text, (center_x - 80, A_y + 70))

    # Height
    pg.draw.line(screen, GREEN, (B_x + 60, B_y), (N_x + 60, N_y), 2)
    pg.draw.line(screen, GREEN, (B_x + 55, B_y), (B_x + 65, B_y), 2)
    pg.draw.line(screen, GREEN, (N_x + 55, N_y), (N_x + 65, N_y), 2)
    height_text = small_font.render(f'Height: {BN:.1f}mm', True, GREEN)
    screen.blit(height_text, (B_x + 70, (B_y + N_y) // 2))

    # Add title and key measurements
    title = font.render('Rover Rocker Bogie Diagram', True, BLACK)
    screen.blit(title, (center_x - 150, 30))

    # Add help text
    help_text = small_font.render(
        'Press any key to close window and save image', True, BLACK)
    screen.blit(help_text, (center_x - 120, 55))

    # Display key measurements
    measurements = [
        f'Robot Length: {TRACK_WIDTH}mm',
        f'Wheel Radius: {WHEEL_RADIUS}mm',
        f'Center to Front Wheel Distance (BC): {BC:.1f}mm',
        f'Center to Aft Pivot Distance (BM): {BM:.1f}mm',
        f'Center to Ground: {CENTER_TO_GROUND:.1f}mm'
    ]

    for i, measurement in enumerate(measurements):
        text = small_font.render(measurement, True, BLACK)
        screen.blit(text, (20, 80 + i * 20))

    # Add legend
    legend_y = SCREEN_HEIGHT - 120
    legend_items = [
        ("Blue: Front Rocker Assembly", BLUE),
        ("Red: Back Rocker Assembly", RED),
        ("Orange: Pivot Point M", ORANGE),
        ("Gray: Wheels", GRAY)
    ]

    legend_title = small_font.render("Legend:", True, BLACK)
    screen.blit(legend_title, (20, legend_y - 20))

    for i, (text, color) in enumerate(legend_items):
        pg.draw.circle(screen, color, (30, legend_y + i * 20 + 10), 5)
        legend_text = small_font.render(text, True, BLACK)
        screen.blit(legend_text, (45, legend_y + i * 20))


def main():
    clock = pg.time.Clock()
    running = True

    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False
            elif event.type == pg.KEYDOWN:
                running = False

        draw_rbm()
        pg.display.flip()
        clock.tick(24)
    pg.image.save(screen, IMAGE_FILE)
    print(f"Diagram saved to '{IMAGE_FILE}'.")
    pg.quit()
    sys.exit()


# Run the visualization
if __name__ == "__main__":
    main()
