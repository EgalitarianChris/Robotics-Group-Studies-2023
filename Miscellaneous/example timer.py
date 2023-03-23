import time

def test_speed():
    start_time = time.time()
    knee_angle = 0
    hip_angle = 0
    for i in range(20):

        knee_angle += 1
        hip_angle -= 2
        limb_data = {"knees":knee_angle,"hips":hip_angle}

        single_time = time.time()
        move_limbs(limb_data)
        single_end

    end_time = time.time()

    print("time taken",round(end_time-start_time,3))

def move_limbs(limb_data):
        time.sleep(0.1)
        print("sent_data")

test_speed()