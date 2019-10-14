import numpy as np
from matplotlib import pyplot as plt

l1 = 20
l2 = 20
DIST_THRESH = 3
locus = []

# Checks if the arm comes in contact with the obstacle
def detect_collision(line_seg, circle):
    a_vec = np.array([line_seg[0][0], line_seg[0][1]])
    b_vec = np.array([line_seg[1][0], line_seg[1][1]])
    c_vec = np.array([circle[0], circle[1]])
    radius = circle[2]
    line_vec = b_vec - a_vec
    line_mag = np.linalg.norm(line_vec)
    circle_vec = c_vec - a_vec
    proj = circle_vec.dot(line_vec / line_mag)
    if proj <= 0:
        closest_point = a_vec
    elif proj >= line_mag:
        closest_point = b_vec
    else:
        closest_point = a_vec + line_vec * proj / line_mag
    if np.linalg.norm(closest_point - c_vec) > radius:
	# Case when no collision is detected 
        return False

    # Case when collision is detected
    return True

# Arm orientation representations
def arm_pos(theta1, theta2):
    shoulder = np.array([0, 0])
    elbow = shoulder + np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    wrist = elbow + np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])
    return shoulder, elbow, wrist

obs = [
[0, -25, 5],
[10, 10, 1],
[11, 10, 1],
[12, 10, 1],
[13, 10, 1],
[14, 10, 1],
[15, 10, 1],
[16, 10, 1],
[17, 10, 1],
[18, 10, 1],
[19, 10, 1],
[20, 10, 1],
[20, 11, 1],
[20, 12, 1],
[20, 13, 1],
[20, 14, 1],
[20, 15, 1],
[20, 16, 1],
[20, 17, 1],
[20, 18, 1],
[20, 19, 1],
[20, 20, 1],
[10, 20, 1],
[11, 20, 1],
[12, 20, 1],
[13, 20, 1],
[14, 20, 1],
[15, 20, 1],
[16, 20, 1],
[17, 20, 1],
[18, 20, 1],
[19, 20, 1],
[5, 15, 1],
[6, 16, 1],
[7, 17, 1],
[8, 18, 1],
[9, 19, 1],
[5, 15, 1],
[6, 14, 1],
[7, 13, 1],
[8, 12, 1],
[9, 11, 1],
]
for theta1 in range(0,360):
    for theta2 in range(0,360):
        pt1, pt2, pt3 = arm_pos(theta1,theta2)
        ptn1, ptn2, ptn3 = arm_pos(theta2+theta1,-theta2)
        for ob in obs:
            if (detect_collision([pt1,pt2],ob) or detect_collision([pt2,pt3],ob)):
                for obn in obs:
                    if (detect_collision([ptn1,ptn2],obn) or detect_collision([ptn2,ptn3],obn)):
                        locus.append(list(pt3))
                        break
    print(theta1)
locus1 = np.array(locus)

# Plots prep
ptx = locus1[:,0]
pty = locus1[:,1]
plt.scatter(ptx,pty)
plt.axis([-40,40,-40,40])
plt.show()