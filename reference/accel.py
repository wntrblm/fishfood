#https://robotics.stackexchange.com/questions/16166/how-to-implement-trapezoidal-velocity-profile-in-code
#https://hackaday.io/project/5765-flexsea-wearable-robotics-toolkit/log/24796-trajectory-generation-trapezoidal-speed-profile
#https://isaacphysics.org/concepts/cp_eq_of_motion?stage=all
#https://openstax.org/books/university-physics-volume-1/pages/3-4-motion-with-constant-acceleration

from math import sqrt
from wintertools.print import print


def us_to_s(ms):
    return ms / 1000000


def s_to_us(s):
    return s * 1000000


distance_mm = 100
velocity_mm_s = 100
accel_mm_s2 = 1000

accel_time_s = velocity_mm_s / accel_mm_s2
decel_time_s = accel_time_s
accel_distance_mm = 0.5 * accel_time_s * velocity_mm_s
decel_distance_mm = accel_distance_mm

print("> Velocity, acceleration/deceleration, and timing:")
print(
    f"{distance_mm=} mm\n"
    f"{velocity_mm_s=} mm/s\n"
    f"{accel_mm_s2=} mm/s2\n"
    f"{accel_time_s=} s\n"
    f"{decel_time_s=} s\n"
    f"{accel_distance_mm=} mm\n"
    f"{decel_distance_mm=} mm\n"
)


steps_per_mm = 80
mm_per_step = 1 / steps_per_mm
max_steps_per_s = velocity_mm_s / mm_per_step
min_s_per_step = 1 / max_steps_per_s
min_us_per_step = s_to_us(min_s_per_step)
max_us_per_step = 10000  # 10 milliseconds

print("> Step conversion constants:")
print(
    f"{steps_per_mm} steps/mm\n"
    f"{mm_per_step} mm/step\n"
    f"{max_steps_per_s} steps/s (max)\n"
    f"{min_s_per_step} s/step (min)\n"
    f"{min_us_per_step} us/step (min)\n"
    f"{max_us_per_step} us/step (max)\n"
)

distance_steps = round(distance_mm * steps_per_mm)
accel_step_count = round(accel_distance_mm * steps_per_mm)
decel_step_count = round(decel_distance_mm * steps_per_mm)
coast_step_count = distance_steps - accel_step_count - decel_step_count

if coast_step_count < 0:
    accel_step_count = round(distance_steps / 2)
    decel_step_count = distance_steps - accel_step_count
    coast_step_count = 0

print("> Step calculations:")
print(
    f"{distance_steps=} steps\n"
    f"{accel_step_count=} steps\n"
    f"{decel_step_count=} steps\n"
    f"{coast_step_count=} steps\n"
)

time = 0

for steps in range(0, distance_steps+1):
    c_distance_mm = steps * mm_per_step

    if steps < accel_step_count:
        c_velocity_mm_s = sqrt(2 * c_distance_mm * accel_mm_s2)
    elif steps < accel_step_count + coast_step_count:
        c_velocity_mm_s = velocity_mm_s
    else:
        c_velocity_mm_s = sqrt(2 * (distance_mm - c_distance_mm) * accel_mm_s2)

    if(c_velocity_mm_s > 0):
        steps_per_s = c_velocity_mm_s / mm_per_step
        s_per_step = 1 / steps_per_s
    else:
        s_per_step = 0

    us_per_step = s_to_us(s_per_step)
    us_per_step = max(us_per_step, min_us_per_step)
    us_per_step = min(us_per_step, max_us_per_step)

    # print(
    #     f"{steps=:4.0f} {c_distance_mm=:6.2f} mm {c_velocity_mm_s=:5.1f} mm/s {us_per_step=:5.0f} us"
    # )
    print(f"{steps:4.0f},{c_distance_mm:6.2f},{c_velocity_mm_s:5.1f},{us_per_step:5.2f},{time:5.5f}")

    time += s_per_step
