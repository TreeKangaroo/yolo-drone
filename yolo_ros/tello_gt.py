# this file provide ground truth about is Tello appears in a frame

def vc05f_gt(frame_num):
    drone_frames = [(437, 1356),
                    (1718, 2115),
                    (2153, 2170),
                    (2306, 2947),
                    (2956, 3007),
                    (3015, 3336),
                    (3407, 3961),
                    (4040, 5941),
                    (5989, 7619)]
    gt = False
    for start, stop in drone_frames:
        if start <= frame_num <= stop:
            gt = True
            break
    return gt