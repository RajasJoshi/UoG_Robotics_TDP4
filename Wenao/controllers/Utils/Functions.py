import math
import csv
import matplotlib.pyplot as plt
import matplotlib.patheffects as path_effects
from matplotlib.colors import LinearSegmentedColormap
from scipy.ndimage import gaussian_filter
from mplsoccer import PyPizza, FontManager, Pitch, VerticalPitch, Sbopen


def calculateDistance(coordinate1, coordinate2) -> float:
    """Calculates the distance between two 2D coordinates.

    Args:
        coordinate1 (list): x, y coordinates.
        coordinate2 (list): x, y coordinates.

    Returns:
        float: Distance between coordinates.
    """
    deltaX = math.fabs(coordinate1[0] - coordinate2[0])
    deltaY = math.fabs(coordinate1[1] - coordinate2[1])
    return math.hypot(deltaX, deltaY)


def calculateAngleAccordingToXAxis(ballCoordinate, robotCoordinate) -> float:
    """Finds the angle between robot-ball vector and x-axis.

    Args:
        ballCoordinate (list): x, y, z coordinates of the ball.
        robotCoordinate (list): x, y coordinates of the robot.

    Returns:
        float: Angle between robot-ball vector and x-axis.
    """
    hypot = calculateDistance(ballCoordinate, robotCoordinate)
    deltaX = math.fabs(ballCoordinate[0] - robotCoordinate[0])

    cosTheta = deltaX / hypot
    degree = math.degrees(math.acos(cosTheta))

    return degree


def calculateBallRegion(ballCoordinate, robotCoordinate) -> int:
    """Ball region is the region of the ball according to robot.
        We assumed the robot as origin of coordinate system.

    Args:
        ballCoordinate (list): x, y, z coordinates of the ball.
        robotCoordinate (list): x, y coordinates of the robot.

    Returns:
        int: 1 = top-right, 2 = top-left, 3 = bottom-left, 4 = bottom-right
    """
    ballRegion = 0
    # Find the location of the ball according to the robot.
    if ballCoordinate[0] > robotCoordinate[0]:
        if ballCoordinate[1] > robotCoordinate[1]:
            ballRegion = 1
        else:
            ballRegion = 4
    else:
        if ballCoordinate[1] > robotCoordinate[1]:
            ballRegion = 2
        else:
            ballRegion = 3

    return ballRegion


def calculateTurningAngleAccordingToRobotHeading(
    ballCoordinate, robotCoordinate, robotHeadingAngle
) -> float:
    """Calculates the turning angle to the ball for robot according to it's heading.

    Args:
        ballCoordinate (list): x, y, z coordinates of the ball.
        robotCoordinate (list): x, y coordinates of the robot.
        robotHeadingAngle (float): Robot heading angle as radian.

    Returns:
        float: The angle needed to turn the ball.
    """

    degree = calculateAngleAccordingToXAxis(ballCoordinate, robotCoordinate)
    ballRegion = calculateBallRegion(ballCoordinate, robotCoordinate)

    # Recalculate the degree according to ballRegion.
    if ballRegion == 2:
        degree = 180 - degree
    elif ballRegion == 3:
        degree = degree - 180
    elif ballRegion == 4:
        degree = -degree

    # Finds the angle of robots heading.
    zAxisDegree = math.degrees(robotHeadingAngle)

    turningAngle = degree - zAxisDegree
    if turningAngle > 180:
        turningAngle = turningAngle - 360
    elif turningAngle < -180:
        turningAngle = turningAngle + 360

    return turningAngle

def write_data(file_name: str, pos: list) -> None:
    """
    LOG PATTERN:

            time_steps,
            possession,
            RedForward,
            RedDefender,
            RedForwardB,
            BlueGoalkeeper,
            BlueDefender,
            BlueForwardB,
            BlueForwardA,
            BallPosition

    """
    # Write the data to the CSV file
    with open(file_name, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Write data
        writer.writerow([pos['time_steps'], pos['possession']] +
                        pos['RedForwardA'] + pos['RedDefender'] + pos['RedForwardB'] +
                        pos['BlueGoalkeeper'] + pos['BlueDefender'] + pos['BlueForwardB'] + pos['BlueForwardA'] + pos['BallPosition'])
        
def draw_pizza(params: list, values: list, file_name: str, stat_type: str) -> None:

    font_normal = FontManager('https://raw.githubusercontent.com/googlefonts/roboto/main/'
                          'src/hinted/Roboto-Regular.ttf')
    font_italic = FontManager('https://raw.githubusercontent.com/googlefonts/roboto/main/'
                                'src/hinted/Roboto-Italic.ttf')
    font_bold = FontManager('https://raw.githubusercontent.com/google/fonts/main/apache/robotoslab/'
                                'RobotoSlab[wght].ttf')
    
    slice_colors = ["#D70232"] * 4 + ["#1A78CF"] * 4
    text_colors = ["#000000"] * 4 + ["#F2F2F2"] * 4

    # instantiate PyPizza class
    baker = PyPizza(
        params=params,                  # list of parameters
        background_color="#222222",     # background color
        straight_line_color="#000000",  # color for straight lines
        straight_line_lw=1,             # linewidth for straight lines
        last_circle_color="#000000",    # color for last line
        last_circle_lw=1,               # linewidth of last circle
        other_circle_lw=0,              # linewidth for other circles
        inner_circle_size=20            # size of inner circle
    )

    # plot pizza
    fig, ax = baker.make_pizza(
        values,                          # list of values
        figsize=(8, 8.5),                # adjust the figsize according to your need
        color_blank_space="same",        # use the same color to fill blank space
        slice_colors=slice_colors,       # color for individual slices
        value_colors=text_colors,        # color for the value-text
        value_bck_colors=slice_colors,   # color for the blank spaces
        blank_alpha=0.4,                 # alpha for blank-space colors
        kwargs_slices=dict(
            edgecolor="#000000", zorder=2, linewidth=1
        ),                               # values to be used when plotting slices
        kwargs_params=dict(
            color="#F2F2F2", fontsize=11,
            fontproperties=font_normal.prop, va="center"
        ),                               # values to be used when adding parameter labels
        kwargs_values=dict(
            color="#F2F2F2", fontsize=11,
            fontproperties=font_normal.prop, zorder=3,
            bbox=dict(
                edgecolor="#000000", facecolor="cornflowerblue",
                boxstyle="round,pad=0.2", lw=1
            )
        )                                # values to be used when adding parameter-values labels
    )

    # add title
    fig.text(
        0.515, 0.975, "ROBOCUP", size=16,
        ha="center", fontproperties=font_bold.prop, color="#F2F2F2"
    )

    # add subtitle
    fig.text(
        0.515, 0.955,
        f"Red Team vs Blue Team | {stat_type} Percentage",
        size=13,
        ha="center", fontproperties=font_bold.prop, color="#F2F2F2"
    )

    # add credits
    CREDIT_1 = "TEAM 4"
    CREDIT_2 = "Sovik Ghosh, Rajas Joshi, Wan NurSabrina, James Milsom, Thimaiah Mandanna, Dhrubojyoti Mookherjee"

    fig.text(
        0.99, 0.02, f"{CREDIT_1}\n{CREDIT_2}", size=9,
        fontproperties=font_italic.prop, color="#F2F2F2",
        ha="right"
    )

    # add text
    fig.text(
        0.38, 0.923, "Red Team         Blue Team", size=14,
        fontproperties=font_bold.prop, color="#F2F2F2"
    )

    # add rectangles
    fig.patches.extend([
        plt.Rectangle(
            (0.52, 0.9225), 0.025, 0.021, fill=True, color="#1a78cf",
            transform=fig.transFigure, figure=fig
        ),
        plt.Rectangle(
            (0.35, 0.9225), 0.025, 0.021, fill=True, color="#d70232",
            transform=fig.transFigure, figure=fig
        ),
    ])
    plt.savefig(file_name)  # Save the plot as an image

def draw_heat(x: list, y: list, file_name: str) -> None:
    pitch = Pitch(pitch_type='statsbomb', line_zorder=2,
                    pitch_color='#22312b', line_color='#efefef')
        
    # fontmanager for google font (robotto)
    robotto_regular = FontManager()

    # path effects
    path_eff = [path_effects.Stroke(linewidth=1.5, foreground='black'),
                path_effects.Normal()]

    # see the custom colormaps example for more ideas on setting colormaps
    pearl_earring_cmap = LinearSegmentedColormap.from_list("Pearl Earring - 10 colors",
                                                        ['#15242e', '#4393c4'], N=10)
    
    fig, axs = pitch.grid(endnote_height=0.03, endnote_space=0,
                    # leave some space for the colorbar
                    grid_width=0.88, left=0.025,
                    title_height=0.06, title_space=0,
                    # Turn off the endnote/title axis. I usually do this after
                    # I am happy with the chart layout and text placement
                    axis=False,
                    grid_height=0.86)
    fig.set_facecolor('#22312b')

    # plot heatmap
    bin_statistic = pitch.bin_statistic(x, y, statistic='count', bins=(25, 25))
    bin_statistic['statistic'] = gaussian_filter(bin_statistic['statistic'], 1)
    pcm = pitch.heatmap(bin_statistic, ax=axs['pitch'], cmap='hot', edgecolors='#22312b')

    # add cbar
    ax_cbar = fig.add_axes((0.915, 0.093, 0.03, 0.786))
    cbar = plt.colorbar(pcm, cax=ax_cbar)
    cbar.outline.set_edgecolor('#efefef')
    cbar.ax.yaxis.set_tick_params(color='#efefef')
    plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='#efefef')
    for label in cbar.ax.get_yticklabels():
        label.set_fontproperties(robotto_regular.prop)
        label.set_fontsize(15)

    # endnote and title
    axs['endnote'].text(1, 0.5, '@your_twitter_handle', va='center', ha='right', fontsize=15,
                        fontproperties=robotto_regular.prop, color='#dee6ea')
    ax_title = axs['title'].text(0.5, 0.5, "Pressure applied by Chelsea FC Women", color='white',
                                va='center', ha='center', path_effects=path_eff,
                                fontproperties=robotto_regular.prop, fontsize=30)
    
    plt.savefig(file_name)  # Save the plot as an image