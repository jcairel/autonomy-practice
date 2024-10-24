def sense(p, colors, measurement):
    s = 0.0
    for i in range(len(p)):
        for j in range(len(p[i])):
            hit = (measurement == colors[i][j])
            # P(X | measurement)
            p[i][j] = p[i][j] * (hit * sensor_right + (1 - hit) * sensor_wrong)
            s += p[i][j]

    # Bayes rule applied, need to normalize distribution again
    for i in range(len(p)):
        for j in range(len(p[i])):
            p[i][j] /= s
    return p


def move(p, motion):
    aux = [[0.0 for row in range(len(p[0]))] for col in range(len(p))]
    for i in range(len(p)):
        for j in range(len(p[i])):
            # Theorem of total probability, convolution
            aux[i][j] = (p_move * p[(i - motion[0]) % len(p)][(j - motion[1]) % len(p[0])]) + (p_stay * p[i][j])
    return aux


def show(p):
    for row in p:
        print(row)


colors = [['red', 'green', 'green', 'red', 'red'],
          ['red', 'red', 'green', 'red', 'red'],
          ['red', 'red', 'green', 'green', 'red'],
          ['red', 'red', 'red', 'red', 'red']]

measurements = ['green', 'green', 'green', 'green', 'green']
motions = [[0, 0], [0, 1], [1, 0], [1, 0], [0, 1]]

# Sensors have a chance to be incorrect, and movement may fail and the bot stays in place
sensor_right = 0.7
sensor_wrong = 1.0 - sensor_right
p_move = 0.8
p_stay = 1.0 - p_move

uniform = 1.0 / float(len(colors)) / float(len(colors[0]))
p = [[uniform for row in range(len(colors[0]))] for col in range(len(colors))]

for i in range(len(motions)):
    # Each movement causes a loss of information
    p = move(p, motions[i])
    # Each sense causes a gain of information
    p = sense(p, colors, measurements[i])

show(p)