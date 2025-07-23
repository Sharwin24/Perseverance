from math import sqrt

# ------ RBM Inputs ------
ROBOT_LENGTH = 400 # [mm]
WHEEL_RADIUS = 35 # [mm]
IMAGE_FILE = "rbm.png" # Output image file name
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
B is center axle of RBM
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

WHEEL_DIAMETER = WHEEL_RADIUS * 2 # [mm]
WHEEL_BASE = ROBOT_LENGTH - WHEEL_RADIUS * 2 # [mm]
NC = WHEEL_BASE / 2 # [mm]
BC = sqrt(2) * NC # [mm]
BM = sqrt(0.5) * NC # [mm]
AM = BM # [mm]
MN = BM # [mm]
BN = sqrt(BC**2 - NC**2) # [mm]
CENTER_TO_GROUND = WHEEL_RADIUS + BN # [mm]

print(
    f'Rover Rocker Bogie Assembly (RBM) parameters:\n'
    f'----Robot Inputs----\n'
    f'- Robot Length: {ROBOT_LENGTH} [mm]\n'
    f'- Wheel Radius: {WHEEL_RADIUS} [mm]\n'
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

import pygame as pg
import sys

# Initialize Pygame
pg.init()

# Screen dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pg.display.set_caption('Rover Rocker Bogie Assembly (RBM)')

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (128, 255, 0)
GRAY = (128, 128, 128)
YELLOW = (255, 255, 0)
ORANGE = (255, 165, 0)

# Font for labels
font = pg.font.Font(None, 28)
small_font = pg.font.Font(None, 22)

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

    M_x = center_x - (BM * scale * 0.7)  # Back pivot (adjusted for visual clarity)
    M_y = center_y - (BM * scale * 0.7)

    # Draw wheels
    wheel_radius_scaled = WHEEL_RADIUS * scale * 0.5  # Scale down for visual clarity
    pg.draw.circle(screen, BLACK, (int(A_x), int(A_y)), int(wheel_radius_scaled), 3)
    pg.draw.circle(screen, BLACK, (int(N_x), int(N_y)), int(wheel_radius_scaled), 3)
    pg.draw.circle(screen, BLACK, (int(C_x), int(C_y)), int(wheel_radius_scaled), 3)

    # Fill wheels
    pg.draw.circle(screen, GRAY, (int(A_x), int(A_y)), int(wheel_radius_scaled))
    pg.draw.circle(screen, GRAY, (int(N_x), int(N_y)), int(wheel_radius_scaled))
    pg.draw.circle(screen, GRAY, (int(C_x), int(C_y)), int(wheel_radius_scaled))

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
    pg.draw.circle(screen, ORANGE, (int(M_x), int(M_y)), 6) # Back pivot M

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
    title = font.render('Rover Rocker Bogie Assembly (RBM)', True, BLACK)
    screen.blit(title, (center_x - 150, 30))

    # Add help text
    help_text = small_font.render('Press any key to close window and save image', True, BLACK)
    screen.blit(help_text, (center_x - 120, 55))

    # Display key measurements
    measurements = [
        f'Robot Length: {ROBOT_LENGTH}mm',
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

# Main game loop
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
        clock.tick(60)
    pg.image.save(screen, IMAGE_FILE)
    print(f"RBM diagram saved as '{IMAGE_FILE}'.")
    pg.quit()
    sys.exit()

# Run the visualization
if __name__ == "__main__":
    main()
